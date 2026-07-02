# =============================================================================
# HATTAR/RV64GC — the full IDE, bare metal, with MMU.
#
#   names, T-, T+, NAND. it's always six o'clock here.
#
# Boot path: hart 0 in M-mode -> PMP opened -> mret to S-mode ->
#            Sv39 page tables built & satp written (MMU ON) -> REPL on UART.
#
# Language (identical to the C deluxe version):
#   node : T- a b T+ c     read a,b at tick-prior; write into c at tick-next
#   node                   query: answers in the language (state, T-, T+)
#   node !                 pulse high for exactly one tick
#   node ! 1 | ! 0 | ! -   hold high / hold low / release
#   w a b c                watch list (w alone clears)
#   t [n]                  run n ticks, print waveforms of watched nodes
#   help                   grammar        quit    power off
#
# Semantics, entire: every node, every tick, becomes NAND of its inputs
# at the previous tick (no inputs -> 0), unless pulsed or held.
#
# Build:  riscv64-unknown-elf-as -march=rv64gc -mabi=lp64 -o hattar.o hattar.S
#         riscv64-unknown-elf-ld -T hattar.ld -o hattar.elf hattar.o
# Run:    qemu-system-riscv64 -machine virt -nographic -bios none -kernel hattar.elf
# =============================================================================

.equ UART,      0x10000000        # NS16550A on qemu-virt / many SoCs
.equ TESTDEV,   0x100000          # sifive_test: write 0x5555 = poweroff
.equ MAXN,      1024              # max nodes
.equ NAMEL,     16                # bytes per name (15 chars + NUL)
.equ MAXIN,     8                 # max fan-in per node
.equ HCAP,      256               # waveform history ring (power of two)
.equ MAXTOK,    64
.equ MAXWATCH,  64
.equ PTE_V,     1
.equ PTE_R,     2
.equ PTE_W,     4
.equ PTE_X,     8
.equ PTE_A,     64
.equ PTE_D,     128

# =============================================================================
# M-mode boot
# =============================================================================
.section .text.init
.globl _start
_start:
        csrr    t0, mhartid
        bnez    t0, park                  # secondary harts sleep
        la      sp, stack_top

        # ---- clear BSS ----
        la      t0, __bss_start
        la      t1, __bss_end
1:      bgeu    t0, t1, 2f
        sd      zero, 0(t0)
        addi    t0, t0, 8
        j       1b
2:
        # ---- hold[] = 0xFF (no hold) ----
        la      t0, hold_arr
        li      t1, MAXN
        li      t2, 0xFF
3:      beqz    t1, 4f
        sb      t2, 0(t0)
        addi    t0, t0, 1
        addi    t1, t1, -1
        j       3b
4:
        # ---- PMP: open all physical memory to S-mode ----
        li      t0, -1
        csrw    pmpaddr0, t0
        li      t0, 0x1f                  # NAPOT | X | W | R
        csrw    pmpcfg0, t0

        # ---- park traps ----
        la      t0, mspin
        csrw    mtvec, t0
        la      t0, sspin
        csrw    stvec, t0

        # ---- M -> S ----
        li      t0, (1 << 11)             # MPP = 01 (S-mode)
        csrs    mstatus, t0
        li      t0, (1 << 12)
        csrc    mstatus, t0
        la      t0, s_entry
        csrw    mepc, t0
        mret

park:   wfi
        j       park
mspin:  j       mspin
sspin:  j       sspin

# =============================================================================
# S-mode: build Sv39 page tables, enable MMU, run the REPL
# =============================================================================
.text
s_entry:
        la      sp, stack_top

        # ---- root page table: two 1 GiB gigapages, identity mapped ----
        la      t0, root_pt
        # VPN2=0 : PA 0x00000000  (UART + test device)  R+W, no X
        li      t1, (PTE_V | PTE_R | PTE_W | PTE_A | PTE_D)
        sd      t1, 0(t0)
        # VPN2=2 : PA 0x80000000  (RAM: us)             R+W+X
        li      t1, 0x20000000            # 0x80000000 >> 12 << 10
        ori     t1, t1, (PTE_V | PTE_R | PTE_W | PTE_X | PTE_A | PTE_D)
        sd      t1, 16(t0)

        # ---- satp = Sv39 | root ----
        srli    t1, t0, 12
        li      t2, 8
        slli    t2, t2, 60
        or      t1, t1, t2
        sfence.vma
        csrw    satp, t1
        sfence.vma                        # MMU is ON from here

        # ---- UART 8N1 (do NOT touch FCR: toggling FIFO-enable flushes any
        #      byte that already arrived, eating the first input character) ----
        li      t0, UART
        li      t1, 0x03
        sb      t1, 3(t0)                 # LCR: 8N1

        # ---- banner ----
        la      a0, str_banner
        call    puts_
        la      a0, str_satp
        call    puts_
        csrr    a0, satp
        call    puthex
        la      a0, str_crlf
        call    puts_
        la      a0, str_hint
        call    puts_

# -----------------------------------------------------------------------------
# REPL main loop
# -----------------------------------------------------------------------------
repl:
        la      a0, str_prompt
        call    puts_
        la      a0, linebuf
        li      a1, 250
        call    getline
        la      a0, linebuf
        call    tokenize                  # a0 = ntok, tokv[] filled
        la      t0, ntok_g
        sd      a0, 0(t0)
        beqz    a0, repl

        # ---- tok0 ----
        la      t0, tokv
        ld      s0, 0(t0)                 # s0 = tok0

        la      a1, str_quit
        mv      a0, s0
        call    streq
        bnez    a0, do_quit
        la      a1, str_help
        mv      a0, s0
        call    streq
        bnez    a0, do_help
        la      a1, str_t
        mv      a0, s0
        call    streq
        bnez    a0, do_tickcmd
        la      a1, str_w
        mv      a0, s0
        call    streq
        bnez    a0, do_watch

        # ---- name-led commands: need ntok ----
        la      t0, ntok_g
        ld      t1, 0(t0)
        li      t2, 1
        bgt     t1, t2, name_multi

        # bare name: query (find only; never create)
        mv      a0, s0
        call    find
        bltz    a0, q_raven
        call    query
        j       repl
q_raven:
        la      a0, str_raven1
        call    puts_
        mv      a0, s0
        call    puts_
        la      a0, str_raven2
        call    puts_
        j       repl

name_multi:
        la      t0, tokv
        ld      s1, 8(t0)                 # s1 = tok1
        la      a1, str_bang
        mv      a0, s1
        call    streq
        bnez    a0, do_poke
        la      a1, str_colon
        mv      a0, s1
        call    streq
        bnez    a0, do_declare
        la      a0, str_usage
        call    puts_
        j       repl

# -----------------------------------------------------------------------------
# quit / help
# -----------------------------------------------------------------------------
do_quit:
        la      a0, str_bye
        call    puts_
        li      t0, TESTDEV
        li      t1, 0x5555
        sw      t1, 0(t0)                 # qemu: exit. hardware: falls through
1:      wfi
        j       1b

do_help:
        la      a0, str_helptext
        call    puts_
        j       repl

# -----------------------------------------------------------------------------
# t [n] : run ticks, then waveforms of watch list
# -----------------------------------------------------------------------------
do_tickcmd:
        li      s2, 1                     # default n
        la      t0, ntok_g
        ld      t1, 0(t0)
        li      t2, 2
        blt     t1, t2, 1f
        la      t0, tokv
        ld      a0, 8(t0)
        call    atoi_
        mv      s2, a0
1:      li      t0, 1
        bge     s2, t0, 2f
        li      s2, 1
2:      li      t0, HCAP
        ble     s2, t0, 3f
        li      s2, HCAP
3:      mv      s3, s2                    # remember n for waves
4:      beqz    s2, 5f
        call    do_tick
        addi    s2, s2, -1
        j       4b
5:      la      a0, str_teq
        call    puts_
        la      t0, tick_cnt
        ld      a0, 0(t0)
        call    putdec
        la      a0, str_crlf
        call    puts_
        mv      a0, s3
        call    waves
        j       repl

# -----------------------------------------------------------------------------
# w [names...] : set watch list
# -----------------------------------------------------------------------------
do_watch:
        la      t0, nwatch_g
        sd      zero, 0(t0)
        li      s2, 1                     # token index
        la      t0, ntok_g
        ld      s3, 0(t0)
1:      bge     s2, s3, 2f
        la      t0, nwatch_g
        ld      t1, 0(t0)
        li      t2, MAXWATCH
        bge     t1, t2, 2f
        la      t0, tokv
        slli    t1, s2, 3
        add     t0, t0, t1
        ld      a0, 0(t0)
        call    intern
        la      t0, nwatch_g
        ld      t1, 0(t0)
        la      t2, watch_arr
        slli    t3, t1, 2
        add     t2, t2, t3
        sw      a0, 0(t2)
        addi    t1, t1, 1
        sd      t1, 0(t0)
        addi    s2, s2, 1
        j       1b
2:      j       repl

# -----------------------------------------------------------------------------
# name ! [1|0|-] : pulse / hold / release        (s0 = tok0)
# -----------------------------------------------------------------------------
do_poke:
        mv      a0, s0
        call    intern
        mv      s1, a0                    # s1 = id
        la      t0, ntok_g
        ld      t1, 0(t0)
        li      t2, 2
        bgt     t1, t2, 1f
        # bare "!" : pulse
        la      t0, pend_arr
        add     t0, t0, s1
        li      t1, 1
        sb      t1, 0(t0)
        j       repl
1:      la      t0, tokv
        ld      t0, 16(t0)                # tok2
        lbu     t1, 0(t0)
        la      t2, hold_arr
        add     t2, t2, s1
        li      t3, '1'
        beq     t1, t3, 2f
        li      t3, '0'
        beq     t1, t3, 3f
        li      t1, 0xFF                  # anything else: release
        sb      t1, 0(t2)
        j       repl
2:      li      t1, 1
        sb      t1, 0(t2)
        j       repl
3:      sb      zero, 0(t2)
        j       repl

# -----------------------------------------------------------------------------
# name : [T- a b ...] [T+ c d ...]                (s0 = tok0)
#   plain tokens are implicitly T-.  T- clause replaces inputs.
#   T+ appends this node into each target's inputs.
# -----------------------------------------------------------------------------
do_declare:
        mv      a0, s0
        call    intern
        mv      s1, a0                    # s1 = this id
        li      s2, 2                     # token index
        li      s4, 0                     # mode: 0 = T-, 1 = T+
        li      s5, 0                     # ns (collected sources)
        li      s6, 0                     # sawsrc
        la      t0, ntok_g
        ld      s3, 0(t0)
decl_loop:
        bge     s2, s3, decl_done
        la      t0, tokv
        slli    t1, s2, 3
        add     t0, t0, t1
        ld      s7, 0(t0)                 # s7 = token
        la      a1, str_tminus
        mv      a0, s7
        call    streq
        beqz    a0, 1f
        li      s4, 0
        li      s6, 1
        j       decl_next
1:      la      a1, str_tplus
        mv      a0, s7
        call    streq
        beqz    a0, 2f
        li      s4, 1
        j       decl_next
2:      mv      a0, s7
        call    intern
        mv      s8, a0                    # s8 = other id
        bnez    s4, decl_tplus
        # T- : collect into ts_tmp
        li      t0, MAXIN
        bge     s5, t0, decl_next
        la      t0, ts_tmp
        slli    t1, s5, 2
        add     t0, t0, t1
        sw      s8, 0(t0)
        addi    s5, s5, 1
        li      s6, 1
        j       decl_next
decl_tplus:
        # append s1 into inputs of s8
        la      t0, nin_arr
        slli    t1, s8, 2
        add     t0, t0, t1
        lw      t2, 0(t0)
        li      t3, MAXIN
        bge     t2, t3, decl_next
        la      t3, in_arr
        slli    t4, s8, 5                 # id * MAXIN * 4
        add     t3, t3, t4
        slli    t4, t2, 2
        add     t3, t3, t4
        sw      s1, 0(t3)
        addi    t2, t2, 1
        sw      t2, 0(t0)
decl_next:
        addi    s2, s2, 1
        j       decl_loop
decl_done:
        beqz    s6, decl_echo             # no T- clause: keep old inputs
        # replace inputs of s1 with ts_tmp[0..s5)
        la      t0, nin_arr
        slli    t1, s1, 2
        add     t0, t0, t1
        sw      s5, 0(t0)
        li      t2, 0
1:      bge     t2, s5, decl_echo
        la      t3, ts_tmp
        slli    t4, t2, 2
        add     t3, t3, t4
        lw      t5, 0(t3)
        la      t3, in_arr
        slli    t4, s1, 5
        add     t3, t3, t4
        slli    t4, t2, 2
        add     t3, t3, t4
        sw      t5, 0(t3)
        addi    t2, t2, 1
        j       1b
decl_echo:
        mv      a0, s1
        call    query
        j       repl

# =============================================================================
# do_tick — the physics. NAND everything, land pulses, apply holds, commit,
#           record history column in the ring.
# =============================================================================
do_tick:
        la      t0, nn_g
        ld      t1, 0(t0)                 # t1 = nn
        li      t2, 0                     # i
tick_p1:
        bge     t2, t1, tick_commit
        # v = AND of inputs (empty = 1)
        la      t3, nin_arr
        slli    t4, t2, 2
        add     t3, t3, t4
        lw      t3, 0(t3)                 # t3 = nin[i]
        li      t4, 1                     # v
        li      t5, 0                     # k
1:      bge     t5, t3, 2f
        la      t6, in_arr
        slli    a1, t2, 5
        add     t6, t6, a1
        slli    a1, t5, 2
        add     t6, t6, a1
        lw      t6, 0(t6)                 # src id
        la      a1, s_arr
        add     a1, a1, t6
        lbu     a1, 0(a1)
        and     t4, t4, a1
        addi    t5, t5, 1
        j       1b
2:      xori    t4, t4, 1                 # NAND
        # pulse?
        la      t5, pend_arr
        add     t5, t5, t2
        lbu     t6, 0(t5)
        beqz    t6, 3f
        li      t4, 1
        sb      zero, 0(t5)
        j       4f
3:      # hold?
        la      t5, hold_arr
        add     t5, t5, t2
        lbu     t6, 0(t5)
        li      a1, 0xFF
        beq     t6, a1, 4f
        mv      t4, t6
4:      la      t5, x_arr
        add     t5, t5, t2
        sb      t4, 0(t5)
        addi    t2, t2, 1
        j       tick_p1

tick_commit:
        # ring column
        la      t2, hlen_g
        ld      t3, 0(t2)
        la      t4, hbase_g
        ld      t5, 0(t4)
        li      t6, HCAP
        blt     t3, t6, 1f
        mv      a1, t5                    # col = hbase
        addi    t5, t5, 1
        andi    t5, t5, HCAP-1
        sd      t5, 0(t4)
        j       2f
1:      add     a1, t5, t3
        andi    a1, a1, HCAP-1            # col = (hbase+hlen)&mask
        addi    t3, t3, 1
        sd      t3, 0(t2)
2:      # commit + history
        li      t2, 0
3:      bge     t2, t1, 4f
        la      t3, x_arr
        add     t3, t3, t2
        lbu     t4, 0(t3)
        la      t3, s_arr
        add     t3, t3, t2
        sb      t4, 0(t3)
        la      t3, hist_arr
        slli    t5, t2, 8                 # id * HCAP
        add     t3, t3, t5
        add     t3, t3, a1
        sb      t4, 0(t3)
        addi    t2, t2, 1
        j       3b
4:      la      t2, tick_cnt
        ld      t3, 0(t2)
        addi    t3, t3, 1
        sd      t3, 0(t2)
        ret

# =============================================================================
# waves(a0 = n) — print last n history columns for each watched node
# =============================================================================
waves:
        addi    sp, sp, -64
        sd      ra, 0(sp)
        sd      s0, 8(sp)
        sd      s1, 16(sp)
        sd      s2, 24(sp)
        sd      s3, 32(sp)
        sd      s4, 40(sp)
        sd      s5, 48(sp)
        sd      s6, 56(sp)
        mv      s0, a0                    # n
        la      t0, nwatch_g
        ld      s1, 0(t0)
        beqz    s1, wv_out
        la      t0, hlen_g
        ld      t1, 0(t0)
        ble     s0, t1, 1f
        mv      s0, t1                    # clamp n to hlen
1:      beqz    s0, wv_out
        # width = max name len among watched
        li      s2, 1                     # width
        li      s3, 0                     # j
2:      bge     s3, s1, 3f
        la      t0, watch_arr
        slli    t1, s3, 2
        add     t0, t0, t1
        lw      t0, 0(t0)
        la      a0, names
        slli    t1, t0, 4
        add     a0, a0, t1
        call    strlen_
        ble     a0, s2, 22f
        mv      s2, a0
22:     addi    s3, s3, 1
        j       2b
3:      # rows
        li      s3, 0                     # j
wv_row: bge     s3, s1, wv_out
        la      t0, watch_arr
        slli    t1, s3, 2
        add     t0, t0, t1
        lw      s4, 0(t0)                 # id
        la      a0, str_2sp
        call    puts_
        la      a0, names
        slli    t1, s4, 4
        add     a0, a0, t1
        call    puts_
        # pad to width+1
        la      a0, names
        slli    t1, s4, 4
        add     a0, a0, t1
        call    strlen_
        sub     s5, s2, a0
        addi    s5, s5, 1
4:      blez    s5, 5f
        li      a0, ' '
        call    uart_putc
        addi    s5, s5, -1
        j       4b
5:      # cols: k from hlen-n .. hlen-1
        la      t0, hlen_g
        ld      s5, 0(t0)
        sub     s5, s5, s0                # k = hlen - n
        la      t0, hlen_g
        ld      s6, 0(t0)                 # end
6:      bge     s5, s6, 7f
        la      t0, hbase_g
        ld      t1, 0(t0)
        add     t1, t1, s5
        andi    t1, t1, HCAP-1            # col
        la      t2, hist_arr
        slli    t3, s4, 8
        add     t2, t2, t3
        add     t2, t2, t1
        lbu     t3, 0(t2)
        la      a0, wv_lo
        beqz    t3, 66f
        la      a0, wv_hi
66:     call    puts_
        addi    s5, s5, 1
        j       6b
7:      la      a0, str_crlf
        call    puts_
        addi    s3, s3, 1
        j       wv_row
wv_out:
        ld      ra, 0(sp)
        ld      s0, 8(sp)
        ld      s1, 16(sp)
        ld      s2, 24(sp)
        ld      s3, 32(sp)
        ld      s4, 40(sp)
        ld      s5, 48(sp)
        ld      s6, 56(sp)
        addi    sp, sp, 64
        ret

# =============================================================================
# query(a0 = id) — answer in the language:  "  name = v"  /  "  name : T- ... T+ ..."
# =============================================================================
query:
        addi    sp, sp, -64
        sd      ra, 0(sp)
        sd      s0, 8(sp)
        sd      s1, 16(sp)
        sd      s2, 24(sp)
        sd      s3, 32(sp)
        sd      s4, 40(sp)
        sd      s5, 48(sp)
        mv      s0, a0                    # id
        la      a0, str_2sp
        call    puts_
        call    q_name
        la      a0, str_eq
        call    puts_
        la      t0, s_arr
        add     t0, t0, s0
        lbu     a0, 0(t0)
        call    putdec
        la      a0, str_crlf
        call    puts_
        la      a0, str_2sp
        call    puts_
        call    q_name
        la      a0, str_colsp
        call    puts_
        # ---- T- list ----
        la      t0, nin_arr
        slli    t1, s0, 2
        add     t0, t0, t1
        lw      s1, 0(t0)                 # nin
        beqz    s1, q_fanout
        la      a0, str_tmi
        call    puts_
        li      s2, 0
1:      bge     s2, s1, q_fanout
        beqz    s2, 2f
        la      a0, str_comma
        call    puts_
2:      li      a0, ' '
        call    uart_putc
        la      t0, in_arr
        slli    t1, s0, 5
        add     t0, t0, t1
        slli    t1, s2, 2
        add     t0, t0, t1
        lw      t0, 0(t0)
        la      a0, names
        slli    t1, t0, 4
        add     a0, a0, t1
        call    puts_
        addi    s2, s2, 1
        j       1b
q_fanout:
        li      s3, 1                     # first
        la      t0, nn_g
        ld      s4, 0(t0)
        li      s2, 0                     # j
3:      bge     s2, s4, q_fin
        la      t0, nin_arr
        slli    t1, s2, 2
        add     t0, t0, t1
        lw      t2, 0(t0)                 # nin[j]
        li      t3, 0                     # k
4:      bge     t3, t2, 5f
        la      t4, in_arr
        slli    t5, s2, 5
        add     t4, t4, t5
        slli    t5, t3, 2
        add     t4, t4, t5
        lw      t4, 0(t4)
        beq     t4, s0, 6f
        addi    t3, t3, 1
        j       4b
6:      beqz    s3, 7f
        la      a0, str_tpl
        call    puts_
        li      s3, 0
        j       8f
7:      la      a0, str_comma
        call    puts_
8:      li      a0, ' '
        call    uart_putc
        la      a0, names
        slli    t1, s2, 4
        add     a0, a0, t1
        call    puts_
5:      addi    s2, s2, 1
        j       3b
q_fin:
        beqz    s3, 9f                    # had fan-out?
        la      t0, nin_arr
        slli    t1, s0, 2
        add     t0, t0, t1
        lw      t0, 0(t0)
        bnez    t0, 9f
        la      a0, str_float
        call    puts_
9:      la      a0, str_crlf
        call    puts_
        # ---- held? ----
        la      t0, hold_arr
        add     t0, t0, s0
        lbu     s1, 0(t0)
        li      t0, 0xFF
        beq     s1, t0, q_out
        la      a0, str_2sp
        call    puts_
        call    q_name
        la      a0, str_held
        call    puts_
        mv      a0, s1
        call    putdec
        la      a0, str_crlf
        call    puts_
q_out:
        ld      ra, 0(sp)
        ld      s0, 8(sp)
        ld      s1, 16(sp)
        ld      s2, 24(sp)
        ld      s3, 32(sp)
        ld      s4, 40(sp)
        ld      s5, 48(sp)
        addi    sp, sp, 64
        ret

q_name: # print name of s0 (helper inside query; clobbers a0,t*)
        addi    sp, sp, -16
        sd      ra, 0(sp)
        la      a0, names
        slli    t1, s0, 4
        add     a0, a0, t1
        call    puts_
        ld      ra, 0(sp)
        addi    sp, sp, 16
        ret

# =============================================================================
# symbol table
# =============================================================================
find:   # a0 = name ptr -> a0 = id or -1
        addi    sp, sp, -32
        sd      ra, 0(sp)
        sd      s0, 8(sp)
        sd      s1, 16(sp)
        sd      s2, 24(sp)
        mv      s0, a0
        la      t0, nn_g
        ld      s1, 0(t0)
        li      s2, 0
1:      bge     s2, s1, 3f
        la      a0, names
        slli    t1, s2, 4
        add     a0, a0, t1
        mv      a1, s0
        call    streq
        bnez    a0, 2f
        addi    s2, s2, 1
        j       1b
2:      mv      a0, s2
        j       4f
3:      li      a0, -1
4:      ld      ra, 0(sp)
        ld      s0, 8(sp)
        ld      s1, 16(sp)
        ld      s2, 24(sp)
        addi    sp, sp, 32
        ret

intern: # a0 = name ptr -> a0 = id (creating if new)
        addi    sp, sp, -32
        sd      ra, 0(sp)
        sd      s0, 8(sp)
        sd      s1, 16(sp)
        mv      s0, a0
        call    find
        bgez    a0, 9f
        la      t0, nn_g
        ld      s1, 0(t0)
        li      t1, MAXN
        blt     s1, t1, 1f
        li      a0, 0                     # table full: alias node 0
        j       9f
1:      # copy up to NAMEL-1 chars
        la      t0, names
        slli    t1, s1, 4
        add     t0, t0, t1
        li      t2, NAMEL-1
2:      lbu     t3, 0(s0)
        beqz    t3, 3f
        beqz    t2, 3f
        sb      t3, 0(t0)
        addi    s0, s0, 1
        addi    t0, t0, 1
        addi    t2, t2, -1
        j       2b
3:      sb      zero, 0(t0)
        la      t0, nn_g
        addi    t1, s1, 1
        sd      t1, 0(t0)
        mv      a0, s1
9:      ld      ra, 0(sp)
        ld      s0, 8(sp)
        ld      s1, 16(sp)
        addi    sp, sp, 32
        ret

# =============================================================================
# tokenizer: splits linebuf in place on space/comma/tab, fills tokv
# =============================================================================
tokenize: # a0 = buf -> a0 = ntok
        la      t0, tokv
        li      t1, 0                     # count
        mv      t2, a0
tk_skip:
        lbu     t3, 0(t2)
        beqz    t3, tk_done
        li      t4, ' '
        beq     t3, t4, tk_adv
        li      t4, ','
        beq     t3, t4, tk_adv
        li      t4, 9
        beq     t3, t4, tk_adv
        li      t5, MAXTOK
        bge     t1, t5, tk_done
        slli    t6, t1, 3
        add     t6, t0, t6
        sd      t2, 0(t6)
        addi    t1, t1, 1
tk_scan:
        addi    t2, t2, 1
        lbu     t3, 0(t2)
        beqz    t3, tk_done
        li      t4, ' '
        beq     t3, t4, tk_end
        li      t4, ','
        beq     t3, t4, tk_end
        li      t4, 9
        beq     t3, t4, tk_end
        j       tk_scan
tk_end: sb      zero, 0(t2)
        addi    t2, t2, 1
        j       tk_skip
tk_adv: addi    t2, t2, 1
        j       tk_skip
tk_done:
        mv      a0, t1
        ret

# =============================================================================
# line input with echo + backspace
# =============================================================================
getline: # a0 = buf, a1 = max -> a0 = len
        addi    sp, sp, -40
        sd      ra, 0(sp)
        sd      s0, 8(sp)
        sd      s1, 16(sp)
        sd      s2, 24(sp)
        sd      s3, 32(sp)
        mv      s0, a0
        li      s1, 0
        addi    s2, a1, -1
gl_loop:
        call    uart_getc
        mv      s3, a0
        li      t0, 13
        beq     s3, t0, gl_done
        li      t0, 10
        beq     s3, t0, gl_done
        li      t0, 8
        beq     s3, t0, gl_bs
        li      t0, 127
        beq     s3, t0, gl_bs
        bge     s1, s2, gl_loop
        add     t0, s0, s1
        sb      s3, 0(t0)
        addi    s1, s1, 1
        mv      a0, s3
        call    uart_putc
        j       gl_loop
gl_bs:
        beqz    s1, gl_loop
        addi    s1, s1, -1
        li      a0, 8
        call    uart_putc
        li      a0, ' '
        call    uart_putc
        li      a0, 8
        call    uart_putc
        j       gl_loop
gl_done:
        add     t0, s0, s1
        sb      zero, 0(t0)
        la      a0, str_crlf
        call    puts_
        mv      a0, s1
        ld      ra, 0(sp)
        ld      s0, 8(sp)
        ld      s1, 16(sp)
        ld      s2, 24(sp)
        ld      s3, 32(sp)
        addi    sp, sp, 40
        ret

# =============================================================================
# UART + string + number primitives
# =============================================================================
uart_putc: # a0 = char
        li      t0, UART
1:      lbu     t1, 5(t0)
        andi    t1, t1, 0x20
        beqz    t1, 1b
        sb      a0, 0(t0)
        ret

uart_getc: # -> a0
        li      t0, UART
1:      lbu     t1, 5(t0)
        andi    t1, t1, 1
        beqz    t1, 1b
        lbu     a0, 0(t0)
        ret

puts_:  # a0 = NUL-terminated string
        addi    sp, sp, -16
        sd      ra, 0(sp)
        sd      s0, 8(sp)
        mv      s0, a0
1:      lbu     a0, 0(s0)
        beqz    a0, 2f
        call    uart_putc
        addi    s0, s0, 1
        j       1b
2:      ld      ra, 0(sp)
        ld      s0, 8(sp)
        addi    sp, sp, 16
        ret

strlen_: # a0 = str -> a0 = len
        mv      t0, a0
        li      a0, 0
1:      lbu     t1, 0(t0)
        beqz    t1, 2f
        addi    a0, a0, 1
        addi    t0, t0, 1
        j       1b
2:      ret

streq:  # a0, a1 -> a0 = 1/0
1:      lbu     t0, 0(a0)
        lbu     t1, 0(a1)
        bne     t0, t1, 2f
        beqz    t0, 3f
        addi    a0, a0, 1
        addi    a1, a1, 1
        j       1b
2:      li      a0, 0
        ret
3:      li      a0, 1
        ret

atoi_:  # a0 = str -> a0 = unsigned value
        mv      t0, a0
        li      a0, 0
        li      t2, 10
1:      lbu     t1, 0(t0)
        li      t3, '0'
        blt     t1, t3, 2f
        li      t3, '9'
        bgt     t1, t3, 2f
        mul     a0, a0, t2
        addi    t1, t1, -'0'
        add     a0, a0, t1
        addi    t0, t0, 1
        j       1b
2:      ret

putdec: # a0 = unsigned 64-bit
        addi    sp, sp, -48
        sd      ra, 40(sp)
        addi    t0, sp, 31
        sb      zero, 0(t0)
        li      t1, 10
1:      remu    t2, a0, t1
        addi    t2, t2, '0'
        addi    t0, t0, -1
        sb      t2, 0(t0)
        divu    a0, a0, t1
        bnez    a0, 1b
        mv      a0, t0
        call    puts_
        ld      ra, 40(sp)
        addi    sp, sp, 48
        ret

puthex: # a0 = 64-bit
        addi    sp, sp, -48
        sd      ra, 40(sp)
        addi    t0, sp, 0
        li      t1, 16                    # digits
        addi    t2, sp, 16
        sb      zero, 0(t2)
1:      beqz    t1, 2f
        andi    t3, a0, 0xF
        li      t4, 10
        blt     t3, t4, 11f
        addi    t3, t3, 'a'-10
        j       12f
11:     addi    t3, t3, '0'
12:     addi    t2, t2, -1
        sb      t3, 0(t2)
        srli    a0, a0, 4
        addi    t1, t1, -1
        j       1b
2:      mv      a0, t2
        call    puts_
        ld      ra, 40(sp)
        addi    sp, sp, 48
        ret

# =============================================================================
# strings
# =============================================================================
.section .rodata
str_banner:   .asciz "\r\nHATTAR/RV64GC -- names, T-, T+, NAND. it's always six o'clock here.\r\n"
str_satp:     .asciz "sv39 paging: ON   satp=0x"
str_hint:     .asciz "type 'help' for the grammar.\r\n\r\n"
str_prompt:   .asciz "hattar> "
str_crlf:     .asciz "\r\n"
str_2sp:      .asciz "  "
str_eq:       .asciz " = "
str_colsp:    .asciz " :"
str_tmi:      .asciz " T-"
str_tpl:      .asciz " T+"
str_comma:    .asciz ","
str_float:    .asciz " (floating: reads 0)"
str_held:     .asciz " held at "
str_teq:      .asciz "  t="
str_quit:     .asciz "quit"
str_help:     .asciz "help"
str_t:        .asciz "t"
str_w:        .asciz "w"
str_bang:     .asciz "!"
str_colon:    .asciz ":"
str_tminus:   .asciz "T-"
str_tplus:    .asciz "T+"
str_raven1:   .asciz "  no node '"
str_raven2:   .asciz "' -- why is a raven like a writing-desk?\r\n"
str_usage:    .asciz "  ?  node : T- a b T+ c | node | node ! [1 0 -] | w ... | t n | help | quit\r\n"
str_bye:      .asciz "  and what is the use of a book without pictures? goodbye.\r\n"
str_helptext: .asciz "  node : T- a b T+ c   read a,b at tick-prior; write into c at tick-next\r\n  node                 query (answers in the language)\r\n  node !               pulse high for exactly one tick\r\n  node ! 1|0|-         hold high / hold low / release\r\n  w a b c              watch list      t [n]   run ticks + waveforms\r\n  every node, every tick: NAND of its inputs at the prior tick. that's all.\r\n"
wv_hi:        .byte 0xe2, 0x96, 0x88, 0x00
wv_lo:        .byte 0xe2, 0x96, 0x81, 0x00

# =============================================================================
# state
# =============================================================================
.bss
.balign 4096
root_pt:    .skip 4096
.balign 8
names:      .skip MAXN*NAMEL
in_arr:     .skip MAXN*MAXIN*4
nin_arr:    .skip MAXN*4
s_arr:      .skip MAXN
x_arr:      .skip MAXN
pend_arr:   .skip MAXN
hold_arr:   .skip MAXN
hist_arr:   .skip MAXN*HCAP
watch_arr:  .skip MAXWATCH*4
tokv:       .skip MAXTOK*8
ts_tmp:     .skip MAXIN*4
linebuf:    .skip 256
.balign 8
nn_g:       .dword 0
nwatch_g:   .dword 0
hlen_g:     .dword 0
hbase_g:    .dword 0
tick_cnt:   .dword 0
ntok_g:     .dword 0
.balign 16
stack:      .skip 16384
stack_top:
