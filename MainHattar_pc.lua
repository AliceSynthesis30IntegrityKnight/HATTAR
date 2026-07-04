--[[ ==========================================================================
 HATTAR-PC — a standalone computer whose processor is your circuit. (Lua port)

 HATTAR is: names, T-, T+, and NAND. Nothing else.
     state(t+1) = NAND of a node's inputs at t      (no inputs -> 0)
 unless pulsed or held. This program is a MOTHERBOARD: peripherals are
 named HOOKS, and whatever HATTAR netlist you wire into them becomes the
 machine. Gates AND hooks are compiled into one generated Lua step
 function via load() — the same trick the C version plays with libtcc.

 MACHINE FILE = full HATTAR language plus hook declarations:
   node : T- a b T+ c        wiring (NAND semantics, tick-prior -> tick-next)
   node                      query (printed at load)
   node !                    pulse high for exactly one tick
   node ! 1 | ! 0 | ! -      hold high / hold low / release
   t n                       pre-run n ticks at load time
   hz N                      target ticks per second (default 240)
   screen W H SCALE          pixel-hook pane geometry
   pix X Y node              per-pixel hook: pixel lit when node high
   key NAME node [toggle]    keyboard hook (momentary; 'toggle' flips)
   maddr n0 .. n15           memory hook: address bus, LSB first
   mdin  n0 .. n7            data-in bus (written to RAM when WE high)
   mdout n0 .. n7            data-out bus (driven with RAM[addr] each tick)
   mwe   node                write-enable
   vram ADDR W H SCALE       display RAM at ADDR as 1bpp bitmap (W mult of 8,
                             LSB = leftmost pixel)
   w n1 n2 ...               waveform signals (up to 16)
   # comment

 RAM is 64 KiB, synchronous SRAM: addr/din/we sampled at tick t, dout
 driven into tick t+1. Hook-driven nodes (keys, mdout) win over both the
 NAND result and holds. Holds are baked into the generated step function;
 a pulse forces one interpreted tick.

 HEADLESS:   lua hattar_pc.lua machine.hattar --headless N
                 [--press KEY]... [--pbm out.pbm]
   runs N ticks, prints waveforms, RAM summary, pixel count; --pbm dumps
   the vram window as a PBM image.  (--press applies after file load.)

 WINDOW MODE (LÖVE): run the same file under love2d —
     love . machine.hattar        (with this file as main.lua, or
                                   require it from your main.lua)
   ESC quit | TAB pause | SPACE single tick | -/= speed halve/double

 Needs Lua 5.3+ (integer bitwise ops). Works under lua5.3/5.4, LuaJIT
 with -joff? no: LuaJIT is 5.1 — use plain Lua or texlua.
========================================================================== --]]

local MAXE, MEMSZ, NWMAX, HRING = 16, 65536, 16, 4096
local KEYMAX, BUSMAX = 32, 16

-- circuit core -------------------------------------------------------------
local NM, IDX, E = {}, {}, {}          -- id->name, name->id, id->input list
local S, X, pend, hold = {}, {}, {}, {}
local nn, anypend = 0, false
local dirty, jit_dead, run_fn = true, false, nil

-- hooks ---------------------------------------------------------------------
local MEM = {}; for i = 0, MEMSZ - 1 do MEM[i] = 0 end
local maddr, mdin, mdout, mwe = {}, {}, {}, nil
local pixb, keyb, KV = {}, {}, {}
local watch, HIST, hp = {}, {}, 0
local scrW, scrH, scrS = 0, 0, 16
local vaddr, vW, vH, vS = -1, 0, 0, 4
local hz = 240.0
local lineno = 0

local function id(s)
  local i = IDX[s]
  if i then return i end
  nn = nn + 1
  NM[nn], IDX[s], E[nn] = s, nn, {}
  S[nn], X[nn], pend[nn], hold[nn] = 0, 0, false, -1
  dirty = true
  return nn
end

-- interpreter (fallback + reference) ----------------------------------------
local function tick_one_interp()
  for i = 1, nn do
    local v, Ei = 1, E[i]
    for j = 1, #Ei do v = v & S[Ei[j]] end
    local x = (#Ei > 0) and (1 - v) or 0          -- empty AND=1, NAND -> 0
    if pend[i] then x = 1; pend[i] = false
    elseif hold[i] >= 0 then x = hold[i] end
    X[i] = x
  end
  anypend = false
  for k = 1, #keyb do X[keyb[k].node] = KV[k] end
  if #maddr > 0 then
    local ad = 0
    for i = 1, #maddr do ad = ad | (S[maddr[i]] << (i - 1)) end
    ad = ad & (MEMSZ - 1)
    if mwe and S[mwe] == 1 then
      local d = 0
      for i = 1, #mdin do d = d | (S[mdin[i]] << (i - 1)) end
      MEM[ad] = d & 0xFF
    end
    local q = MEM[ad]
    for i = 1, #mdout do X[mdout[i]] = (q >> (i - 1)) & 1 end
  end
  for w = 1, #watch do
    HIST[(hp % HRING) * NWMAX + w] = X[watch[w]]
  end
  hp = hp + 1
  for i = 1, nn do S[i] = X[i] end
end

-- "JIT": generate one Lua function for gates + hooks, compile with load() ---
local function jit_build()
  if jit_dead then return false end
  local t0 = os.clock()
  local L = {}
  local function emit(s) L[#L + 1] = s end
  emit("local floor=math.floor")
  emit("return function(S,X,k,MEM,KV,HIST,hp)")
  emit("local a,b=S,X")
  emit("for _=1,k do")
  for i = 1, nn do
    if hold[i] >= 0 then emit(("b[%d]=%d"):format(i, hold[i]))
    elseif #E[i] == 0 then emit(("b[%d]=0"):format(i))
    else
      local terms = {}
      for j = 1, #E[i] do terms[j] = ("a[%d]"):format(E[i][j]) end
      emit(("b[%d]=1-(%s)"):format(i, table.concat(terms, "&")))
    end
  end
  for k = 1, #keyb do emit(("b[%d]=KV[%d]"):format(keyb[k].node, k)) end
  if #maddr > 0 then
    local at = {}
    for i = 1, #maddr do at[i] = ("(a[%d]<<%d)"):format(maddr[i], i - 1) end
    emit(("local ad=(%s)&%d"):format(table.concat(at, "|"), MEMSZ - 1))
    if mwe and #mdin > 0 then
      local dt = {}
      for i = 1, #mdin do dt[i] = ("(a[%d]<<%d)"):format(mdin[i], i - 1) end
      emit(("if a[%d]==1 then MEM[ad]=(%s)&255 end")
           :format(mwe, table.concat(dt, "|")))
    end
    if #mdout > 0 then
      emit("local q=MEM[ad]")
      for i = 1, #mdout do
        emit(("b[%d]=(q>>%d)&1"):format(mdout[i], i - 1))
      end
    end
  end
  for w = 1, #watch do
    emit(("HIST[(hp%%%d)*%d+%d]=b[%d]"):format(HRING, NWMAX, w, watch[w]))
  end
  emit("hp=hp+1")
  emit("a,b=b,a")
  emit("end")
  emit(("if a~=S then for i=1,%d do S[i]=a[i] end end"):format(nn))
  emit("return hp")
  emit("end")
  local chunk, err = load(table.concat(L, "\n"), "hattar-step")
  if not chunk then
    io.stderr:write("  [jit failed: ", tostring(err), "; interpreting]\n")
    jit_dead = true
    return false
  end
  run_fn = chunk()
  io.stderr:write(("  [jit: %d gates + hooks -> Lua function, %.1f ms]\n")
                  :format(nn, 1000 * (os.clock() - t0)))
  return true
end

local function ticks(k)
  if k <= 0 then return end
  while anypend and k > 0 do tick_one_interp(); k = k - 1 end
  if k == 0 then return end
  if dirty and nn > 0 then if jit_build() then dirty = false end end
  if not dirty and run_fn then hp = run_fn(S, X, k, MEM, KV, HIST, hp); return end
  for _ = 1, k do tick_one_interp() end
end

-- machine file ---------------------------------------------------------------
local function toks(line)
  local out = {}
  line = line:gsub("#.*", "")
  for w in line:gmatch("[^%s,]+") do out[#out + 1] = w end
  return out
end

local function query(i)
  io.write(("  %s = %d\n  %s :"):format(NM[i], S[i], NM[i]))
  if #E[i] > 0 then
    io.write(" T-")
    for k = 1, #E[i] do io.write(k > 1 and "," or "", " ", NM[E[i][k]]) end
  end
  local first = true
  for j = 1, nn do
    for k = 1, #E[j] do
      if E[j][k] == i then
        io.write(first and " T+" or ",", " ", NM[j]); first = false; break
      end
    end
  end
  if #E[i] == 0 and first then io.write(" (floating: reads 0)") end
  io.write("\n")
  if hold[i] >= 0 then io.write(("  %s held at %d\n"):format(NM[i], hold[i])) end
end

local function need(tk, n, what)
  if #tk < n then
    io.stderr:write(("line %d: missing %s\n"):format(lineno, what))
    return false
  end
  return true
end

local function load_line(line)
  local tk = toks(line)
  local t = tk[1]
  if not t then return end

  if t == "hz" then
    if need(tk, 2, "rate") then hz = math.max(1, tonumber(tk[2]) or 240) end
  elseif t == "screen" then
    if need(tk, 3, "W H") then
      scrW, scrH = tonumber(tk[2]) or 0, tonumber(tk[3]) or 0
      scrS = tonumber(tk[4]) or 16
    end
  elseif t == "pix" then
    if need(tk, 4, "X Y node") then
      pixb[#pixb + 1] = { x = tonumber(tk[2]) or 0, y = tonumber(tk[3]) or 0,
                          node = id(tk[4]) }
    end
  elseif t == "key" then
    if need(tk, 3, "keyname node") and #keyb < KEYMAX then
      keyb[#keyb + 1] = { name = tk[2], node = id(tk[3]),
                          toggle = tk[4] == "toggle" }
      KV[#keyb] = 0
      dirty = true
    end
  elseif t == "maddr" or t == "mdin" or t == "mdout" then
    local bus = (t == "maddr") and maddr or (t == "mdin") and mdin or mdout
    for i = #bus, 1, -1 do bus[i] = nil end
    for k = 2, math.min(#tk, BUSMAX + 1) do bus[#bus + 1] = id(tk[k]) end
    dirty = true
  elseif t == "mwe" then
    if need(tk, 2, "node") then mwe = id(tk[2]); dirty = true end
  elseif t == "mem" then                                -- mem ADDR hexbyte...
    if need(tk, 2, "ADDR") then
      local ad = (tonumber(tk[2]) or 0) & (MEMSZ - 1)
      for k = 3, #tk do
        MEM[ad] = tonumber(tk[k], 16) & 0xFF
        ad = (ad + 1) & (MEMSZ - 1)
      end
    end
  elseif t == "vram" then
    if need(tk, 4, "ADDR W H") then
      vaddr = (tonumber(tk[2]) or 0) & (MEMSZ - 1)
      vW = (tonumber(tk[3]) or 0) & ~7
      vH = tonumber(tk[4]) or 0
      vS = tonumber(tk[5]) or 4
    end
  elseif t == "mem" then                              -- mem ADDR xx xx ...
    if need(tk, 2, "ADDR") then
      local ad = (tonumber(tk[2]) or 0) & (MEMSZ - 1)
      for k = 3, #tk do
        MEM[ad] = (tonumber(tk[k], 16) or 0) & 0xFF
        ad = (ad + 1) & (MEMSZ - 1)
      end
    end
  elseif t == "w" then
    for i = #watch, 1, -1 do watch[i] = nil end
    for k = 2, math.min(#tk, NWMAX + 1) do watch[#watch + 1] = id(tk[k]) end
    dirty = true
  elseif t == "t" then
    ticks(tonumber(tk[2]) or 1)
  elseif #tk == 1 then                                  -- query
    local i = IDX[t]
    if i then query(i)
    else io.write(("  no node '%s' — why is a raven like a writing-desk?\n")
                  :format(t)) end
  elseif tk[2] == "!" then                              -- pulse / hold
    local i = id(t)
    local v = tk[3]
    if not v then pend[i] = true; anypend = true
    elseif v == "1" then hold[i] = 1; dirty = true
    elseif v == "0" then hold[i] = 0; dirty = true
    else hold[i] = -1; dirty = true end
  elseif tk[2] == ":" then                              -- declare
    local i, mode, srcs, sawsrc = id(t), 0, {}, false
    for k = 3, #tk do
      local w = tk[k]
      if w == "T-" then mode = 0; sawsrc = true
      elseif w == "T+" then mode = 1
      else
        local j = id(w)
        if mode == 0 then
          if #srcs < MAXE then srcs[#srcs + 1] = j end
          sawsrc = true
        elseif #E[j] < MAXE then E[j][#E[j] + 1] = i end
      end
    end
    if sawsrc then E[i] = srcs end
    dirty = true
  else
    io.stderr:write(("line %d: ?  (node : T- a b T+ c | node ! [1 0 -]"
                     .. " | hooks)\n"):format(lineno))
  end
end

local function load_file(path)
  local f, err = io.open(path, "r")
  if not f then error(err, 0) end
  for line in f:lines() do lineno = lineno + 1; load_line(line) end
  f:close()
end

-- headless -------------------------------------------------------------------
local FULL, LOW = "\226\150\136", "\226\150\129"   -- block / low block

local function dump_pbm(path)
  if vaddr < 0 or vW == 0 or vH == 0 then
    io.stderr:write("--pbm: no vram declared\n"); return
  end
  local f = assert(io.open(path, "w"))
  f:write(("P1\n%d %d\n"):format(vW, vH))
  for y = 0, vH - 1 do
    local row = {}
    for x = 0, vW - 1 do
      local by = MEM[(vaddr + y * (vW // 8) + x // 8) & (MEMSZ - 1)]
      row[#row + 1] = ((by >> (x & 7)) & 1) == 1 and "1" or "0"
    end
    f:write(table.concat(row, " "), "\n")
  end
  f:close()
  io.write(("  vram -> %s (%dx%d)\n"):format(path, vW, vH))
end

local function headless(n, pbm)
  ticks(n)
  io.write(("[headless] %d ticks, %d gates, hp=%d\n"):format(n, nn, hp))
  for w = 1, #watch do
    local shown = math.min(n, 64, hp)
    io.write(("  %-10s "):format(NM[watch[w]]))
    for k = hp - shown, hp - 1 do
      io.write(HIST[(k % HRING) * NWMAX + w] == 1 and FULL or LOW)
    end
    io.write("\n")
  end
  local nz = 0
  for i = 0, MEMSZ - 1 do if MEM[i] ~= 0 then nz = nz + 1 end end
  io.write(("  RAM: %d nonzero bytes; MEM[0..15] ="):format(nz))
  for i = 0, 15 do io.write((" %02x"):format(MEM[i])) end
  io.write("\n")
  local lit = 0
  for p = 1, #pixb do if S[pixb[p].node] == 1 then lit = lit + 1 end end
  io.write(("  pixels lit: %d / %d\n"):format(lit, #pixb))
  if pbm then dump_pbm(pbm) end
end

-- LÖVE window mode (used automatically when run under love2d) -----------------
local function love_mode(machine)
  local paused, acc = false, 0
  function love.load()
    load_file(machine)
    local top = math.max(scrH * scrS, vH * vS)
    local w = 16 + scrW * scrS + 24 + vW * vS + 32
    local h = 16 + top + 40 + #watch * 26 + 24
    love.window.setMode(math.max(w, 16 + 1024 + 16), math.max(h, 300))
    love.window.setTitle("HATTAR-PC — it's always six o'clock here")
  end
  function love.keypressed(key)
    if key == "escape" then love.event.quit()
    elseif key == "tab" then paused = not paused
    elseif key == "space" and paused then ticks(1)
    elseif key == "-" then hz = math.max(1, hz / 2)
    elseif key == "=" then hz = math.min(2e7, hz * 2)
    else
      for k = 1, #keyb do
        if keyb[k].name:lower() == key:lower() then
          if keyb[k].toggle then KV[k] = 1 - KV[k] else KV[k] = 1 end
        end
      end
    end
  end
  function love.keyreleased(key)
    for k = 1, #keyb do
      if keyb[k].name:lower() == key:lower() and not keyb[k].toggle then
        KV[k] = 0
      end
    end
  end
  function love.update(dt)
    if paused then return end
    acc = acc + dt * hz
    local k = math.min(math.floor(acc), 5000000)
    if k > 0 then ticks(k); acc = acc - k end
  end
  function love.draw()
    local g = love.graphics
    g.clear(14/255, 14/255, 18/255)
    local ox, oy = 16, 16
    for p = 1, #pixb do
      local b = pixb[p]
      if S[b.node] == 1 then g.setColor(130/255, 240/255, 140/255)
      else g.setColor(30/255, 34/255, 40/255) end
      g.rectangle("fill", ox + b.x * scrS, oy + b.y * scrS, scrS - 1, scrS - 1)
    end
    if vaddr >= 0 and vW > 0 and vH > 0 then
      local vx = ox + scrW * scrS + (scrW > 0 and 24 or 0)
      for y = 0, vH - 1 do
        for xB = 0, vW // 8 - 1 do
          local by = MEM[(vaddr + y * (vW // 8) + xB) & (MEMSZ - 1)]
          for b = 0, 7 do
            if (by >> b) & 1 == 1 then g.setColor(235/255, 235/255, 225/255)
            else g.setColor(24/255, 24/255, 30/255) end
            g.rectangle("fill", vx + (xB * 8 + b) * vS, oy + y * vS, vS, vS)
          end
        end
      end
    end
    local wy = oy + math.max(scrH * scrS, vH * vS) + 40
    g.setColor(120/255, 220/255, 240/255)
    for w = 1, #watch do
      local rowy, view = wy + (w - 1) * 26, 512
      local start = math.max(hp - view, 0, hp - HRING)
      for k = start, hp - 1 do
        local v = HIST[(k % HRING) * NWMAX + w] or 0
        g.points(16 + (k - start), rowy + (v == 1 and 2 or 18))
      end
    end
  end
end

-- entry ------------------------------------------------------------------------
if rawget(_G, "love") then
  love_mode((love.arg and love.arg.parseGameArguments(arg)[1]) or arg[1]
            or "machine.hattar")
else
  local machine = arg and arg[1]
  if not machine then
    io.stderr:write("usage: lua hattar_pc.lua machine.hattar --headless N"
                    .. " [--press KEY]... [--pbm out.pbm]\n"
                    .. "       (window mode: run under love2d)\n")
    os.exit(1)
  end
  local headn, presses, pbm = nil, {}, nil
  local i = 2
  while arg[i] do
    if arg[i] == "--headless" and arg[i + 1] then
      headn = tonumber(arg[i + 1]); i = i + 1
    elseif arg[i] == "--press" and arg[i + 1] then
      presses[#presses + 1] = arg[i + 1]; i = i + 1
    elseif arg[i] == "--pbm" and arg[i + 1] then
      pbm = arg[i + 1]; i = i + 1
    end
    i = i + 1
  end
  load_file(machine)
  for _, name in ipairs(presses) do             -- applies after load-time 't'
    local found = false
    for k = 1, #keyb do
      if keyb[k].name == name then KV[k] = 1; found = true end
    end
    if not found then
      io.stderr:write(("--press: no key '%s' bound\n"):format(name))
    end
  end
  if headn then headless(headn, pbm)
  else
    io.stderr:write("no --headless N given and not running under love2d;"
                    .. " nothing to do.\n")
    os.exit(1)
  end
end
