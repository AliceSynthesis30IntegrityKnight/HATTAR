/* =============================================================================
 * HATTAR-PC — a standalone computer whose processor is your circuit.
 *
 * Not an emulator OF any processor: a MOTHERBOARD. The environment provides
 * the peripherals as named HOOKS; whatever HATTAR netlist you wire into them
 * becomes the machine. All hooks are compiled INTO the JIT'd circuit, so the
 * screen, memory and keyboard run inside the generated machine code.
 *
 * MACHINE FILE = full HATTAR language plus hook declarations:
 *   node : T- a b T+ c        wiring (NAND semantics, tick-prior -> tick-next)
 *   node ! v                  poke        |  t n     pre-run ticks at load
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
 *   vram ADDR W H SCALE       screen manipulation at scale: display RAM
 *                             bytes at ADDR as a 1bpp bitmap (W multiple
 *                             of 8, LSB = leftmost pixel)
 *   w n1 n2 ...               waveform pane signals (up to 16)
 *
 * RAM is 64 KiB. Memory semantics are synchronous SRAM: address/din/we are
 * sampled at tick t, dout is driven into tick t+1 — perfectly Markovian.
 *
 * ENVIRONMENT CONTROLS (window mode):
 *   ESC quit | TAB pause | SPACE single tick (paused) | -/= speed halve/double
 *   ,/. waveform zoom out/in
 *
 * HEADLESS SELFTEST:  hattar_pc machine.hattar --headless N [--press KEY]...
 *   runs N ticks with hooks live, prints waveforms, RAM summary, pixel count.
 *
 * BUILD:
 *   cc -O2 hattar_pc.c -I$TCCSRC $TCCSRC/libtcc.a \
 *      $(sdl2-config --cflags --libs) -lpthread -lm -ldl -o hattar_pc
 * ========================================================================== */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <SDL2/SDL.h>
#include "libtcc.h"

#define MAXN   4096
#define MAXE   16
#define MEMSZ  65536
#define NWMAX  16
#define HRING  4096            /* waveform ring, samples per signal (pow2) */
#define KEYMAX 32
#define PIXMAX 8192
#define BUSMAX 16

/* ---- circuit core ---- */
char NM[MAXN][32]; int E[MAXN][MAXE], ne[MAXN], nn, S[MAXN], X[MAXN];
int dirty = 1;
TCCState *live = 0;
void (*run_fn)(int*,int*,long,unsigned char*,const int*,unsigned char*,long*) = 0;
char SRC[1 << 22];

/* ---- hooks ---- */
unsigned char MEM[MEMSZ];
int maddr[BUSMAX], nmaddr, mdin[BUSMAX], nmdin, mdout[BUSMAX], nmdout, mwe = -1;
struct { int x, y, node; } pixb[PIXMAX]; int npix;
struct { char name[24]; int node, toggle, slot; SDL_Scancode sc; } keyb[KEYMAX];
int nkey; int KV[KEYMAX];
int watch[NWMAX], nwatch;
unsigned char HIST[HRING * NWMAX]; long hp = 0;
int scrW = 0, scrH = 0, scrS = 16;
long vaddr = -1; int vW = 0, vH = 0, vS = 4;
double hz = 240.0;

int id(char *w) {
    for (int i = 0; i < nn; i++) if (!strcmp(NM[i], w)) return i;
    dirty = 1;
    return strcpy(NM[nn], w), nn++;
}

/* =================== interpreter (fallback + reference) =================== */
void tick_one_interp(void) {
    for (int i = 0; i < nn; i++) {
        int v = 1;
        for (int j = 0; j < ne[i]; j++) v &= S[E[i][j]];
        X[i] = !v;
    }
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
    memcpy(S, X, sizeof S);
}

/* ============================ libtcc JIT ================================== */
void err_cb(void *o, const char *m) { fprintf(stderr, "  [tcc] %s\n", m); }

int jit_build(void) {
    int L = 0;
    L += sprintf(SRC + L, "static void step(const int*a,int*b){\n");
    for (int i = 0; i < nn; i++) {
        if (!ne[i]) { L += sprintf(SRC + L, "b[%d]=0;\n", i); continue; }
        L += sprintf(SRC + L, "b[%d]=!(", i);
        for (int j = 0; j < ne[i]; j++)
            L += sprintf(SRC + L, "%sa[%d]", j ? "&" : "", E[i][j]);
        L += sprintf(SRC + L, ");\n");
    }
    L += sprintf(SRC + L,
        "}\nvoid run(int*s,int*x,long k,unsigned char*M,const int*KV,"
        "unsigned char*H,long*hp){int*a=s,*b=x;long h=*hp;\n"
        "for(;k>0;k--){step(a,b);\n");
    for (int k = 0; k < nkey; k++)
        L += sprintf(SRC + L, "b[%d]=KV[%d];\n", keyb[k].node, keyb[k].slot);
    if (nmaddr) {
        L += sprintf(SRC + L, "{unsigned ad=0;");
        for (int i = 0; i < nmaddr; i++)
            L += sprintf(SRC + L, "ad|=(unsigned)a[%d]<<%d;", maddr[i], i);
        L += sprintf(SRC + L, "ad&=%d;", MEMSZ - 1);
        if (mwe >= 0 && nmdin) {
            L += sprintf(SRC + L, "if(a[%d]){unsigned d=0;", mwe);
            for (int i = 0; i < nmdin; i++)
                L += sprintf(SRC + L, "d|=(unsigned)a[%d]<<%d;", mdin[i], i);
            L += sprintf(SRC + L, "M[ad]=(unsigned char)d;}");
        }
        L += sprintf(SRC + L, "unsigned q=M[ad];");
        for (int i = 0; i < nmdout; i++)
            L += sprintf(SRC + L, "b[%d]=(q>>%d)&1;", mdout[i], i);
        L += sprintf(SRC + L, "}\n");
    }
    for (int w = 0; w < nwatch; w++)
        L += sprintf(SRC + L, "H[(h&%d)*%d+%d]=(unsigned char)b[%d];\n",
                     HRING - 1, NWMAX, w, watch[w]);
    if (nwatch) L += sprintf(SRC + L, "h++;\n");
    L += sprintf(SRC + L,
        "{int*t=a;a=b;b=t;}}\n*hp=h;\n"
        "if(a!=s){int i;for(i=0;i<%d;i++)s[i]=a[i];}}\n", nn);

    clock_t t0 = clock();
    TCCState *st = tcc_new();
    if (!st) return 0;
    tcc_set_error_func(st, 0, err_cb);
    tcc_set_options(st, "-nostdlib -nostdinc");
    tcc_set_output_type(st, TCC_OUTPUT_MEMORY);
    if (tcc_compile_string(st, SRC) < 0) { tcc_delete(st); return 0; }
#ifdef TCC_RELOCATE_AUTO
    if (tcc_relocate(st, TCC_RELOCATE_AUTO) < 0) { tcc_delete(st); return 0; }
#else
    if (tcc_relocate(st) < 0) { tcc_delete(st); return 0; }
#endif
    void *fp = tcc_get_symbol(st, "run");
    if (!fp) { tcc_delete(st); return 0; }
    if (live) tcc_delete(live);
    live = st;
    run_fn = (void (*)(int*,int*,long,unsigned char*,const int*,
                       unsigned char*,long*))fp;
    fprintf(stderr, "  [jit: %d gates + hooks -> machine code, %.1f ms]\n",
            nn, 1000.0 * (clock() - t0) / CLOCKS_PER_SEC);
    return 1;
}

void ticks(long k) {
    if (nn && dirty && jit_build()) dirty = 0;
    if (!dirty && run_fn) { run_fn(S, X, k, MEM, KV, HIST, &hp); return; }
    while (k-- > 0) tick_one_interp();
}

/* ============================ machine file ================================ */
int busparse(char **t, int *bus) {         /* fills bus with node ids */
    int c = 0;
    while ((*t = strtok(0, " ,\t\n")) && c < BUSMAX) bus[c++] = id(*t);
    dirty = 1;
    return c;
}

void load_line(char *Ln) {
    char *t = strtok(Ln, " ,\t\n");
    if (!t || *t == '#') return;
    if (!strcmp(t, "hz"))      { if ((t = strtok(0, " \n"))) hz = atof(t); }
    else if (!strcmp(t, "screen")) {
        scrW = atoi(strtok(0, " \n")); scrH = atoi(strtok(0, " \n"));
        t = strtok(0, " \n"); scrS = t ? atoi(t) : 16;
    }
    else if (!strcmp(t, "pix")) {
        int px = atoi(strtok(0, " ,\n")), py = atoi(strtok(0, " ,\n"));
        char *nmv = strtok(0, " ,\n");
        if (nmv && npix < PIXMAX) {
            pixb[npix].x = px; pixb[npix].y = py; pixb[npix].node = id(nmv);
            npix++;
        }
    }
    else if (!strcmp(t, "key")) {
        char *knm = strtok(0, " ,\n"), *nod = strtok(0, " ,\n");
        char *tg = strtok(0, " ,\n");
        if (knm && nod && nkey < KEYMAX) {
            strncpy(keyb[nkey].name, knm, 23);
            keyb[nkey].node = id(nod);
            keyb[nkey].toggle = tg && !strcmp(tg, "toggle");
            keyb[nkey].slot = nkey; keyb[nkey].sc = SDL_SCANCODE_UNKNOWN;
            nkey++; dirty = 1;
        }
    }
    else if (!strcmp(t, "maddr")) nmaddr = busparse(&t, maddr);
    else if (!strcmp(t, "mdin"))  nmdin  = busparse(&t, mdin);
    else if (!strcmp(t, "mdout")) nmdout = busparse(&t, mdout);
    else if (!strcmp(t, "mwe"))   { if ((t = strtok(0, " \n"))) { mwe = id(t); dirty = 1; } }
    else if (!strcmp(t, "vram")) {
        vaddr = atol(strtok(0, " \n")); vW = atoi(strtok(0, " \n"));
        vH = atoi(strtok(0, " \n")); t = strtok(0, " \n"); vS = t ? atoi(t) : 4;
        vW &= ~7;
    }
    else if (!strcmp(t, "w")) {
        nwatch = 0;
        while ((t = strtok(0, " ,\n")) && nwatch < NWMAX) watch[nwatch++] = id(t);
        dirty = 1;
    }
    else if (!strcmp(t, "t")) ticks((t = strtok(0, " \n")) ? atol(t) : 1);
    else {                                     /* HATTAR proper */
        int a = id(t), m = 0;
        char *t2 = strtok(0, " ,\t\n");
        if (!t2) {                             /* query */
            printf("%s=%d  T-", NM[a], S[a]);
            for (int j = 0; j < ne[a]; j++) printf(" %s", NM[E[a][j]]);
            printf("  T+");
            for (int i = 0; i < nn; i++) for (int j = 0; j < ne[i]; j++)
                if (E[i][j] == a) { printf(" %s", NM[i]); break; }
            puts("");
        } else if (!strcmp(t2, "!")) {
            S[a] = (t2 = strtok(0, " \n")) ? atoi(t2) : 1;
        } else {
            dirty = 1;
            for (ne[a] = 0, t = t2; t; t = strtok(0, " ,\t\n")) {
                if (!strcmp(t, ":") || !strcmp(t, "T-")) m = 0;
                else if (!strcmp(t, "T+")) m = 1;
                else { int b = id(t); if (m) E[b][ne[b]++] = a; else E[a][ne[a]++] = b; }
            }
        }
    }
}

/* ============================ headless mode =============================== */
void headless(long n) {
    ticks(n);
    printf("[headless] %ld ticks, %d gates, hp=%ld\n", n, nn, hp);
    for (int w = 0; w < nwatch; w++) {
        long shown = n < 64 ? n : 64;
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
void draw(SDL_Renderer *R, long view) {
    SDL_SetRenderDrawColor(R, 14, 14, 18, 255);
    SDL_RenderClear(R);
    int ox = 16, oy = 16;
    /* pixel-hook pane */
    SDL_SetRenderDrawColor(R, 40, 40, 52, 255);
    SDL_Rect bord = { ox - 2, oy - 2, scrW * scrS + 4, scrH * scrS + 4 };
    SDL_RenderDrawRect(R, &bord);
    for (int p = 0; p < npix; p++) {
        SDL_Rect r = { ox + pixb[p].x * scrS, oy + pixb[p].y * scrS,
                       scrS - 1, scrS - 1 };
        if (S[pixb[p].node]) SDL_SetRenderDrawColor(R, 130, 240, 140, 255);
        else                 SDL_SetRenderDrawColor(R, 30, 34, 40, 255);
        SDL_RenderFillRect(R, &r);
    }
    /* vram pane */
    if (vaddr >= 0 && vW && vH) {
        int vx = ox + scrW * scrS + 24, vy = oy;
        SDL_SetRenderDrawColor(R, 40, 40, 52, 255);
        SDL_Rect vb = { vx - 2, vy - 2, vW * vS + 4, vH * vS + 4 };
        SDL_RenderDrawRect(R, &vb);
        for (int y = 0; y < vH; y++)
            for (int xB = 0; xB < vW / 8; xB++) {
                unsigned char by = MEM[(vaddr + y * (vW / 8) + xB) & (MEMSZ - 1)];
                for (int b = 0; b < 8; b++) {
                    if (by & (1 << b)) SDL_SetRenderDrawColor(R, 235, 235, 225, 255);
                    else               SDL_SetRenderDrawColor(R, 24, 24, 30, 255);
                    SDL_Rect r = { vx + (xB * 8 + b) * vS, vy + y * vS, vS, vS };
                    SDL_RenderFillRect(R, &r);
                }
            }
    }
    /* waveform pane */
    int wy = oy + (scrH * scrS > vH * vS ? scrH * scrS : vH * vS) + 40;
    for (int w = 0; w < nwatch; w++) {
        int rowy = wy + w * 26;
        SDL_SetRenderDrawColor(R, 60, 60, 72, 255);
        SDL_RenderDrawLine(R, 16, rowy + 20, 16 + (int)view, rowy + 20);
        SDL_SetRenderDrawColor(R, 120, 220, 240, 255);
        long start = hp - view; if (start < 0) start = 0;
        for (long k = start; k < hp; k++) {
            int v = HIST[(k & (HRING - 1)) * NWMAX + w];
            int px2 = 16 + (int)(k - start);
            SDL_RenderDrawPoint(R, px2, rowy + (v ? 2 : 18));
            if (k > start) {
                int pv = HIST[((k - 1) & (HRING - 1)) * NWMAX + w];
                if (pv != v) SDL_RenderDrawLine(R, px2, rowy + 2, px2, rowy + 18);
                SDL_RenderDrawLine(R, px2 - 1, rowy + (pv ? 2 : 18),
                                   px2, rowy + (v ? 2 : 18));
            }
        }
    }
    SDL_RenderPresent(R);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s machine.hattar [--headless N] [--press KEY]\n",
                argv[0]);
        return 1;
    }
    FILE *f = fopen(argv[1], "r");
    if (!f) { perror(argv[1]); return 1; }
    char Ln[512];
    long headn = 0;
    for (int i = 2; i < argc; i++) {
        if (!strcmp(argv[i], "--headless") && i + 1 < argc) headn = atol(argv[++i]);
        if (!strcmp(argv[i], "--press") && i + 1 < argc) {
            char *k = argv[++i];
            /* resolved after load */
            (void)k;
        }
    }
    while (fgets(Ln, sizeof Ln, f)) load_line(Ln);
    fclose(f);
    /* resolve --press by bound key name */
    for (int i = 2; i < argc; i++)
        if (!strcmp(argv[i], "--press") && i + 1 < argc) {
            for (int k = 0; k < nkey; k++)
                if (!strcmp(keyb[k].name, argv[i + 1])) KV[keyb[k].slot] = 1;
            i++;
        }

    if (headn) { headless(headn); return 0; }

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
