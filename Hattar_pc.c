
/* =============================================================================
 * HATTAR-PC — a standalone computer whose processor is your circuit.
 *
 * HATTAR (see hattar.c) is: names, T-, T+, and NAND. Nothing else.
 *     state(t+1) = NAND of a node's inputs at t      (no inputs -> 0)
 * unless pulsed or held. This program is not an emulator OF any processor:
 * it is a MOTHERBOARD. It provides peripherals as named HOOKS; whatever
 * HATTAR netlist you wire into them becomes the machine. All hooks are
 * compiled INTO the JIT'd circuit, so screen, memory and keyboard run
 * inside the generated machine code.
 *
 * MACHINE FILE = full HATTAR language plus hook declarations:
 *   node : T- a b T+ c        wiring (NAND semantics, tick-prior -> tick-next)
 *   node                      query (printed at load)
 *   node !                    pulse high for exactly one tick
 *   node ! 1 | ! 0 | ! -      hold high / hold low / release
 *   t n                       pre-run n ticks at load time
 *   hz N                      target ticks per second (default 240)
 *   screen W H SCALE          pixel-hook pane geometry
 *   pix X Y node              per-pixel hook: pixel lit when node high
 *   key NAME node [toggle]    keyboard hook (SDL key name: A, Space, Up...)
 *                             momentary by default; 'toggle' flips per press
 *   maddr n0 n1 ... n15       memory hook: address bus, LSB first
 *   mdin  n0 ... n7           data-in bus (written to RAM when WE high)
 *   mdout n0 ... n7           data-out bus (environment DRIVES these nodes
 *                             with RAM[addr] every tick)
 *   mwe   node                write-enable
 *   vram ADDR W H SCALE       display RAM bytes at ADDR as a 1bpp bitmap
 *                             (W multiple of 8, LSB = leftmost pixel)
 *   w n1 n2 ...               waveform pane signals (up to 16)
 *   # comment
 *
 * RAM is 64 KiB. Memory semantics are synchronous SRAM: address/din/we are
 * sampled at tick t, dout is driven into tick t+1 — perfectly Markovian.
 * Hook-driven nodes (keys, mdout) are driven by the environment every tick
 * and win over both the NAND result and holds; don't also wire them.
 * Holds are baked into the JIT; a pulse forces one interpreted tick.
 *
 * ENVIRONMENT CONTROLS (window mode):
 *   ESC quit | TAB pause | SPACE single tick (paused) | -/= speed halve/double
 *   ,/. waveform zoom out/in
 *
 * HEADLESS SELFTEST:  hattar_pc machine.hattar --headless N [--press KEY]...
 *   runs N ticks with hooks live, prints waveforms, RAM summary, pixel count.
 *   (--press applies after the file loads, i.e. after any load-time 't'.)
 *
 * BUILD:
 *   cc -O2 hattar_pc.c -I$TCCSRC $TCCSRC/libtcc.a \
 *      $(sdl2-config --cflags --libs) -lpthread -lm -ldl -o hattar_pc
 * ========================================================================== */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include <SDL2/SDL.h>
#include "libtcc.h"

#define MAXN   4096
#define MAXE   16
#define NAMEL  32
#define MEMSZ  65536
#define NWMAX  16
#define HRING  4096            /* waveform ring, samples per signal (pow2) */
#define KEYMAX 32
#define PIXMAX 8192
#define BUSMAX 16
#define SRCSZ  (1 << 22)

/* ---- circuit core ---- */
static char NM[MAXN][NAMEL];
static int  E[MAXN][MAXE], ne[MAXN], nn;
static int  S[MAXN], X[MAXN];
static unsigned char pend[MAXN];
static signed char   hold[MAXN];
static int  anypend = 0;
static int  dirty = 1, jit_dead = 0;
static TCCState *live = 0;
static void (*run_fn)(int*,int*,long,unsigned char*,const int*,
                      unsigned char*,long*) = 0;
static char SRC[SRCSZ];

/* ---- hooks ---- */
static unsigned char MEM[MEMSZ];
static int maddr[BUSMAX], nmaddr, mdin[BUSMAX], nmdin, mdout[BUSMAX], nmdout;
static int mwe = -1;
static struct { int x, y, node; } pixb[PIXMAX]; static int npix;
static struct { char name[24]; int node, toggle, slot; SDL_Scancode sc; }
       keyb[KEYMAX];
static int nkey, KV[KEYMAX];
static int watch[NWMAX], nwatch;
static unsigned char HIST[HRING * NWMAX]; static long hp = 0;
static int scrW = 0, scrH = 0, scrS = 16;
static long vaddr = -1; static int vW = 0, vH = 0, vS = 4;
static double hz = 240.0;
static int lineno = 0;

static int find(const char *s) {
    for (int i = 0; i < nn; i++) if (!strcmp(NM[i], s)) return i;
    return -1;
}
static int id(const char *s) {
    int i = find(s);
    if (i >= 0) return i;
    if (nn >= MAXN) { fprintf(stderr, "hattar_pc: too many nodes\n"); exit(1); }
    strncpy(NM[nn], s, NAMEL - 1);
    ne[nn] = 0; S[nn] = 0; pend[nn] = 0; hold[nn] = -1;
    dirty = 1;
    return nn++;
}

/* =================== interpreter (fallback + reference) =================== */
static void tick_one_interp(void) {
    for (int i = 0; i < nn; i++) {
        int v = 1;                                   /* empty AND = 1     */
        for (int j = 0; j < ne[i]; j++) v &= S[E[i][j]];
        X[i] = !v;                                   /* ... NAND -> 0     */
        if (pend[i])          { X[i] = 1; pend[i] = 0; }
        else if (hold[i] >= 0)  X[i] = hold[i];
    }
    anypend = 0;
    for (int k = 0; k < nkey; k++) X[keyb[k].node] = KV[keyb[k].slot];
    if (nmaddr) {
        unsigned ad = 0;
        for (int i = 0; i < nmaddr; i++) ad |= (unsigned)S[maddr[i]] << i;
        ad &= MEMSZ - 1;
        if (mwe >= 0 && S[mwe]) {
            unsigned d = 0;
            for (int i = 0; i < nmdin; i++) d |= (unsigned)S[mdin[i]] << i;
            MEM[ad] = (unsigned char)d;
        }
        unsigned q = MEM[ad];
        for (int i = 0; i < nmdout; i++) X[mdout[i]] = (q >> i) & 1;
    }
    for (int w = 0; w < nwatch; w++)
        HIST[(hp & (HRING - 1)) * NWMAX + w] = (unsigned char)X[watch[w]];
    hp++;
    memcpy(S, X, nn * sizeof S[0]);
}

/* ============================ libtcc JIT ================================== */
static void err_cb(void *o, const char *m) { (void)o;
    fprintf(stderr, "  [tcc] %s\n", m); }

static int L;                              /* emit cursor */
static int emitf(const char *fmt, ...) {
    if (L > SRCSZ - 4096) return 0;        /* refuse to overflow */
    va_list ap; va_start(ap, fmt);
    L += vsnprintf(SRC + L, SRCSZ - L, fmt, ap);
    va_end(ap);
    return 1;
}

static int jit_build(void) {
    if (jit_dead) return 0;
    L = 0;
    int ok = emitf("static void step(const int*a,int*b){\n");
    for (int i = 0; i < nn && ok; i++) {
        if (hold[i] >= 0)  { ok = emitf("b[%d]=%d;\n", i, hold[i]); continue; }
        if (!ne[i])        { ok = emitf("b[%d]=0;\n", i); continue; }
        ok = emitf("b[%d]=!(", i);
        for (int j = 0; j < ne[i] && ok; j++)
            ok = emitf("%sa[%d]", j ? "&" : "", E[i][j]);
        if (ok) ok = emitf(");\n");
    }
    if (ok) ok = emitf(
        "}\nvoid run(int*s,int*x,long k,unsigned char*M,const int*KV,"
        "unsigned char*H,long*hp){int*a=s,*b=x;long h=*hp;\n"
        "for(;k>0;k--){step(a,b);\n");
    for (int k = 0; k < nkey && ok; k++)
        ok = emitf("b[%d]=KV[%d];\n", keyb[k].node, keyb[k].slot);
    if (nmaddr && ok) {
        ok = emitf("{unsigned ad=0;");
        for (int i = 0; i < nmaddr && ok; i++)
            ok = emitf("ad|=(unsigned)a[%d]<<%d;", maddr[i], i);
        if (ok) ok = emitf("ad&=%d;", MEMSZ - 1);
        if (mwe >= 0 && nmdin && ok) {
            ok = emitf("if(a[%d]){unsigned d=0;", mwe);
            for (int i = 0; i < nmdin && ok; i++)
                ok = emitf("d|=(unsigned)a[%d]<<%d;", mdin[i], i);
            if (ok) ok = emitf("M[ad]=(unsigned char)d;}");
        }
        if (ok) ok = emitf("unsigned q=M[ad];");
        for (int i = 0; i < nmdout && ok; i++)
            ok = emitf("b[%d]=(q>>%d)&1;", mdout[i], i);
        if (ok) ok = emitf("}\n");
    }
    for (int w = 0; w < nwatch && ok; w++)
        ok = emitf("H[(h&%d)*%d+%d]=(unsigned char)b[%d];\n",
                   HRING - 1, NWMAX, w, watch[w]);
    if (ok) ok = emitf("h++;\n");          /* hp counts ticks even unwatched */
    if (ok) ok = emitf(
        "{int*t=a;a=b;b=t;}}\n*hp=h;\n"
        "if(a!=s){int i;for(i=0;i<%d;i++)s[i]=a[i];}}\n", nn);
    if (!ok) {
        fprintf(stderr, "  [jit: source too large; interpreting]\n");
        jit_dead = 1; return 0;
    }

    clock_t t0 = clock();
    TCCState *st = tcc_new();
    if (!st) { jit_dead = 1; return 0; }
    tcc_set_error_func(st, 0, err_cb);
    tcc_set_options(st, "-nostdlib -nostdinc");
    tcc_set_output_type(st, TCC_OUTPUT_MEMORY);
    if (tcc_compile_string(st, SRC) < 0) { tcc_delete(st); jit_dead = 1; return 0; }
#ifdef TCC_RELOCATE_AUTO
    if (tcc_relocate(st, TCC_RELOCATE_AUTO) < 0) { tcc_delete(st); jit_dead = 1; return 0; }
#else
    if (tcc_relocate(st) < 0) { tcc_delete(st); jit_dead = 1; return 0; }
#endif
    void *fp = tcc_get_symbol(st, "run");
    if (!fp) { tcc_delete(st); jit_dead = 1; return 0; }
    if (live) tcc_delete(live);
    live = st;
    run_fn = (void (*)(int*,int*,long,unsigned char*,const int*,
                       unsigned char*,long*))fp;
    fprintf(stderr, "  [jit: %d gates + hooks -> machine code, %.1f ms]\n",
            nn, 1000.0 * (clock() - t0) / CLOCKS_PER_SEC);
    return 1;
}

static void ticks(long k) {
    if (k <= 0) return;
    while (anypend && k > 0) { tick_one_interp(); k--; }   /* pulses: 1 tick */
    if (!k) return;
    if (dirty && nn) { if (jit_build()) dirty = 0; }
    if (!dirty && run_fn) { run_fn(S, X, k, MEM, KV, HIST, &hp); return; }
    while (k-- > 0) tick_one_interp();
}

/* ============================ machine file ================================ */
static char *next(void) { return strtok(0, " ,\t\r\n"); }
static char *req(const char *what) {
    char *t = next();
    if (!t) fprintf(stderr, "line %d: missing %s\n", lineno, what);
    return t;
}
static int busparse(int *bus) {
    char *t; int c = 0;
    while ((t = next()) && c < BUSMAX) bus[c++] = id(t);
    dirty = 1;
    return c;
}

static void query(int i) {
    printf("  %s = %d\n  %s :", NM[i], S[i], NM[i]);
    if (ne[i]) {
        printf(" T-");
        for (int k = 0; k < ne[i]; k++) printf("%s %s", k ? "," : "", NM[E[i][k]]);
    }
    int first = 1;
    for (int j = 0; j < nn; j++)
        for (int k = 0; k < ne[j]; k++)
            if (E[j][k] == i) {
                printf("%s %s", first ? " T+" : ",", NM[j]);
                first = 0; break;
            }
    if (!ne[i] && first) printf(" (floating: reads 0)");
    putchar('\n');
    if (hold[i] >= 0) printf("  %s held at %d\n", NM[i], hold[i]);
}

static void load_line(char *Ln) {
    char *hash = strchr(Ln, '#'); if (hash) *hash = 0;
    char *t = strtok(Ln, " ,\t\r\n");
    if (!t) return;

    if (!strcmp(t, "hz")) {
        if ((t = req("rate"))) { hz = atof(t); if (hz < 1) hz = 1; }
    }
    else if (!strcmp(t, "screen")) {
        char *a = req("W"), *b = a ? req("H") : 0, *c = b ? next() : 0;
        if (b) { scrW = atoi(a); scrH = atoi(b); scrS = c ? atoi(c) : 16; }
    }
    else if (!strcmp(t, "pix")) {
        char *a = req("X"), *b = a ? req("Y") : 0, *c = b ? req("node") : 0;
        if (c && npix < PIXMAX) {
            pixb[npix].x = atoi(a); pixb[npix].y = atoi(b);
            pixb[npix].node = id(c); npix++;
        }
    }
    else if (!strcmp(t, "key")) {
        char *knm = req("keyname"), *nod = knm ? req("node") : 0, *tg = nod ? next() : 0;
        if (nod && nkey < KEYMAX) {
            strncpy(keyb[nkey].name, knm, 23);
            keyb[nkey].node = id(nod);
            keyb[nkey].toggle = tg && !strcmp(tg, "toggle");
            keyb[nkey].slot = nkey; keyb[nkey].sc = SDL_SCANCODE_UNKNOWN;
            nkey++; dirty = 1;
        }
    }
    else if (!strcmp(t, "maddr")) nmaddr = busparse(maddr);
    else if (!strcmp(t, "mdin"))  nmdin  = busparse(mdin);
    else if (!strcmp(t, "mdout")) nmdout = busparse(mdout);
    else if (!strcmp(t, "mwe"))   { if ((t = req("node"))) { mwe = id(t); dirty = 1; } }
    else if (!strcmp(t, "vram")) {
        char *a = req("ADDR"), *b = a ? req("W") : 0, *c = b ? req("H") : 0;
        char *d = c ? next() : 0;
        if (c) {
            vaddr = atol(a) & (MEMSZ - 1);
            vW = atoi(b) & ~7; vH = atoi(c); vS = d ? atoi(d) : 4;
        }
    }
    else if (!strcmp(t, "w")) {
        nwatch = 0;
        while ((t = next()) && nwatch < NWMAX) watch[nwatch++] = id(t);
        dirty = 1;
    }
    else if (!strcmp(t, "t")) {
        t = next();
        ticks(t ? atol(t) : 1);
    }
    else {                                             /* HATTAR proper */
        char name[NAMEL]; strncpy(name, t, NAMEL - 1); name[NAMEL - 1] = 0;
        char *t2 = next();
        if (!t2) {                                     /* query          */
            int i = find(name);
            if (i < 0) printf("  no node '%s' — why is a raven like a writing-desk?\n", name);
            else query(i);
        }
        else if (!strcmp(t2, "!")) {                   /* pulse / hold   */
            int i = id(name);
            char *v = next();
            if (!v)                    { pend[i] = 1; anypend = 1; }
            else if (!strcmp(v, "1"))  { hold[i] = 1;  dirty = 1; }
            else if (!strcmp(v, "0"))  { hold[i] = 0;  dirty = 1; }
            else                       { hold[i] = -1; dirty = 1; }
        }
        else if (!strcmp(t2, ":")) {                   /* declare        */
            int i = id(name), mode = 0, srcs[MAXE], ns = 0, sawsrc = 0;
            while ((t = next())) {
                if (!strcmp(t, "T-")) { mode = 0; sawsrc = 1; continue; }
                if (!strcmp(t, "T+")) { mode = 1; continue; }
                int j = id(t);
                if (mode == 0) { if (ns < MAXE) srcs[ns++] = j; sawsrc = 1; }
                else if (ne[j] < MAXE) E[j][ne[j]++] = i;
            }
            if (sawsrc) { ne[i] = ns; memcpy(E[i], srcs, sizeof srcs); }
            dirty = 1;
        }
        else fprintf(stderr, "line %d: ?  (node : T- a b T+ c | node !"
                     " [1 0 -] | hooks)\n", lineno);
    }
}

/* ============================ headless mode =============================== */
static void headless(long n) {
    ticks(n);
    printf("[headless] %ld ticks, %d gates, hp=%ld\n", n, nn, hp);
    for (int w = 0; w < nwatch; w++) {
        long shown = n < 64 ? n : 64;
        if (shown > hp) shown = hp;
        printf("  %-10s ", NM[watch[w]]);
        for (long k = hp - shown; k < hp; k++)
            fputs(HIST[(k & (HRING - 1)) * NWMAX + w] ? "\xe2\x96\x88"
                                                      : "\xe2\x96\x81", stdout);
        puts("");
    }
    int nz = 0; for (int i = 0; i < MEMSZ; i++) if (MEM[i]) nz++;
    printf("  RAM: %d nonzero bytes; MEM[0..15] =", nz);
    for (int i = 0; i < 16; i++) printf(" %02x", MEM[i]);
    puts("");
    int lit = 0; for (int p = 0; p < npix; p++) if (S[pixb[p].node]) lit++;
    printf("  pixels lit: %d / %d\n", lit, npix);
}

/* ============================== window mode =============================== */
static void draw(SDL_Renderer *R, long view) {
    SDL_SetRenderDrawColor(R, 14, 14, 18, 255);
    SDL_RenderClear(R);
    int ox = 16, oy = 16;
    /* pixel-hook pane */
    if (scrW && scrH) {
        SDL_SetRenderDrawColor(R, 40, 40, 52, 255);
        SDL_Rect bord = { ox - 2, oy - 2, scrW * scrS + 4, scrH * scrS + 4 };
        SDL_RenderDrawRect(R, &bord);
    }
    for (int p = 0; p < npix; p++) {
        SDL_Rect r = { ox + pixb[p].x * scrS, oy + pixb[p].y * scrS,
                       scrS - 1, scrS - 1 };
        if (S[pixb[p].node]) SDL_SetRenderDrawColor(R, 130, 240, 140, 255);
        else                 SDL_SetRenderDrawColor(R, 30, 34, 40, 255);
        SDL_RenderFillRect(R, &r);
    }
    /* vram pane */
    if (vaddr >= 0 && vW && vH) {
        int vx = ox + scrW * scrS + (scrW ? 24 : 0), vy = oy;
        SDL_SetRenderDrawColor(R, 40, 40, 52, 255);
        SDL_Rect vb = { vx - 2, vy - 2, vW * vS + 4, vH * vS + 4 };
        SDL_RenderDrawRect(R, &vb);
        for (int y = 0; y < vH; y++)
            for (int xB = 0; xB < vW / 8; xB++) {
                unsigned char by = MEM[(vaddr + y * (vW / 8) + xB) & (MEMSZ - 1)];
                for (int b = 0; b < 8; b++) {
                    if (by & (1u << b)) SDL_SetRenderDrawColor(R, 235, 235, 225, 255);
                    else                SDL_SetRenderDrawColor(R, 24, 24, 30, 255);
                    SDL_Rect r = { vx + (xB * 8 + b) * vS, vy + y * vS, vS, vS };
                    SDL_RenderFillRect(R, &r);
                }
            }
    }
    /* waveform pane */
    int top = scrH * scrS > vH * vS ? scrH * scrS : vH * vS;
    int wy = oy + top + 40;
    for (int w = 0; w < nwatch; w++) {
        int rowy = wy + w * 26;
        SDL_SetRenderDrawColor(R, 60, 60, 72, 255);
        SDL_RenderDrawLine(R, 16, rowy + 20, 16 + (int)view, rowy + 20);
        SDL_SetRenderDrawColor(R, 120, 220, 240, 255);
        long start = hp - view; if (start < 0) start = 0;
        long lo = hp - HRING; if (start < lo) start = lo < 0 ? 0 : lo;
        for (long k = start; k < hp; k++) {
            int v = HIST[(k & (HRING - 1)) * NWMAX + w];
            int px2 = 16 + (int)(k - start);
            SDL_RenderDrawPoint(R, px2, rowy + (v ? 2 : 18));
            if (k > start) {
                int pv = HIST[((k - 1) & (HRING - 1)) * NWMAX + w];
                if (pv != v) SDL_RenderDrawLine(R, px2, rowy + 2, px2, rowy + 18);
            }
        }
    }
    SDL_RenderPresent(R);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s machine.hattar [--headless N] [--press KEY]...\n",
                argv[0]);
        return 1;
    }
    FILE *f = fopen(argv[1], "r");
    if (!f) { perror(argv[1]); return 1; }
    long headn = 0;
    int headless_mode = 0;
    for (int i = 2; i < argc; i++)
        if (!strcmp(argv[i], "--headless") && i + 1 < argc) {
            headn = atol(argv[i + 1]); headless_mode = 1; i++;
        }

    char Ln[512];
    while (fgets(Ln, sizeof Ln, f)) { lineno++; load_line(Ln); }
    fclose(f);

    /* resolve --press by bound key name (applies after load-time ticks) */
    for (int i = 2; i < argc; i++)
        if (!strcmp(argv[i], "--press") && i + 1 < argc) {
            int found = 0;
            for (int k = 0; k < nkey; k++)
                if (!strcmp(keyb[k].name, argv[i + 1])) { KV[keyb[k].slot] = 1; found = 1; }
            if (!found) fprintf(stderr, "--press: no key '%s' bound\n", argv[i + 1]);
            i++;
        }

    if (headless_mode) { headless(headn); return 0; }

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL: %s\n", SDL_GetError()); return 1;
    }
    int winw = 16 + scrW * scrS + 24 + vW * vS + 32;
    int wavew = 16 + 1024 + 16;
    if (winw < wavew) winw = wavew;
    int winh = 16 + (scrH * scrS > vH * vS ? scrH * scrS : vH * vS)
             + 40 + nwatch * 26 + 24;
    if (winh < 300) winh = 300;
    SDL_Window *W = SDL_CreateWindow("HATTAR-PC — it's always six o'clock here",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, winw, winh, 0);
    SDL_Renderer *R = SDL_CreateRenderer(W, -1, SDL_RENDERER_ACCELERATED);
    for (int k = 0; k < nkey; k++)
        keyb[k].sc = SDL_GetScancodeFromName(keyb[k].name);

    int running = 1, paused = 0;
    long view = 512;
    double acc = 0;
    Uint64 prev = SDL_GetPerformanceCounter();
    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) running = 0;
            else if (ev.type == SDL_KEYDOWN && !ev.key.repeat) {
                SDL_Scancode sc = ev.key.keysym.scancode;
                if (sc == SDL_SCANCODE_ESCAPE) running = 0;
                else if (sc == SDL_SCANCODE_TAB) paused = !paused;
                else if (sc == SDL_SCANCODE_SPACE && paused) ticks(1);
                else if (sc == SDL_SCANCODE_MINUS)  { hz /= 2; if (hz < 1) hz = 1; }
                else if (sc == SDL_SCANCODE_EQUALS) { hz *= 2; if (hz > 2e7) hz = 2e7; }
                else if (sc == SDL_SCANCODE_COMMA)  { view *= 2; if (view > 1024) view = 1024; }
                else if (sc == SDL_SCANCODE_PERIOD) { view /= 2; if (view < 64) view = 64; }
                else for (int k = 0; k < nkey; k++) if (keyb[k].sc == sc) {
                    if (keyb[k].toggle) KV[keyb[k].slot] ^= 1;
                    else KV[keyb[k].slot] = 1;
                }
            }
            else if (ev.type == SDL_KEYUP) {
                for (int k = 0; k < nkey; k++)
                    if (keyb[k].sc == ev.key.keysym.scancode && !keyb[k].toggle)
                        KV[keyb[k].slot] = 0;
            }
        }
        Uint64 now = SDL_GetPerformanceCounter();
        double dt = (double)(now - prev) / SDL_GetPerformanceFrequency();
        prev = now;
        if (!paused) {
            acc += dt * hz;
            long k = (long)acc;
            if (k > 5000000) k = 5000000;
            if (k > 0) { ticks(k); acc -= k; }
        }
        draw(R, view);
        SDL_Delay(1000 / 60);
    }
    SDL_DestroyRenderer(R); SDL_DestroyWindow(W); SDL_Quit();
    return 0;
}
