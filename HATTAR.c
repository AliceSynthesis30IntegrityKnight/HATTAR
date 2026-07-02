/* HATTAR — names, T-, T+, and NAND. Nothing else.
 *
 *   node : T- a b T+ c     read a,b at tick-prior; write into c at tick-next
 *   node                   query: answers in the language itself
 *   node !                 pulse high for exactly one tick
 *   node ! 1 | ! 0 | ! -   hold high / hold low / release
 *   w a b c                watch list
 *   t [n]                  run n ticks, print waveforms of watched nodes
 *   quit                   
 *
 * THE ENTIRE SEMANTICS:  every node, every tick:
 *     state(t+1) = NAND of its inputs at t     (no inputs -> 0)
 * unless pulsed or held. That is the whole language. NOT is a 1-input
 * NAND. Memory is two cross-coupled NANDs. A clock is a self-NAND.
 * Constants are floating nodes (0) and their inversion (1).
 *
 *   cc hattar.c -o hattar && ./hattar
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#ifdef __unix__
#include <unistd.h>
#endif

#define MAXN  4096
#define MAXIN 16
#define NAMEL 32
#define HCAP  512

static char  nm[MAXN][NAMEL];
static int   in[MAXN][MAXIN], nin[MAXN], nn = 0;
static unsigned char st[MAXN], nx[MAXN], pend[MAXN], hist[MAXN][HCAP];
static signed char   hold[MAXN];
static int   hlen = 0, watch[64], nwatch = 0, ticks = 0;

static int find(const char *s) {
    for (int i = 0; i < nn; i++) if (!strcmp(nm[i], s)) return i;
    return -1;
}
static int intern(const char *s) {
    int i = find(s);
    if (i >= 0) return i;
    strncpy(nm[nn], s, NAMEL - 1);
    nin[nn] = 0; st[nn] = 0; pend[nn] = 0; hold[nn] = -1;
    return nn++;
}

static void tick(void) {
    for (int i = 0; i < nn; i++) {
        int v = 1;                                  /* empty AND = 1   */
        for (int k = 0; k < nin[i]; k++) v &= st[in[i][k]];
        nx[i] = !v;                                 /* ... NAND -> 0   */
        if (pend[i])            { nx[i] = 1; pend[i] = 0; }
        else if (hold[i] >= 0)    nx[i] = (unsigned char)hold[i];
    }
    if (hlen == HCAP) {                             /* slide history   */
        for (int i = 0; i < nn; i++)
            memmove(hist[i], hist[i] + 1, HCAP - 1);
        hlen--;
    }
    for (int i = 0; i < nn; i++) { st[i] = nx[i]; hist[i][hlen] = st[i]; }
    hlen++; ticks++;
}

static void waves(int n) {
    if (n > hlen) n = hlen;
    int w = 1;
    for (int j = 0; j < nwatch; j++)
        if ((int)strlen(nm[watch[j]]) > w) w = strlen(nm[watch[j]]);
    for (int j = 0; j < nwatch; j++) {
        int id = watch[j];
        printf("  %-*s ", w, nm[id]);
        for (int k = hlen - n; k < hlen; k++)
            fputs(hist[id][k] ? "\xe2\x96\x88" : "\xe2\x96\x81", stdout);
        putchar('\n');
    }
}

static void query(const char *s) {
    int i = find(s);
    if (i < 0) { printf("  no node '%s' — why is a raven like a writing-desk?\n", s); return; }
    printf("  %s = %d\n  %s :", nm[i], st[i], nm[i]);
    if (nin[i]) {
        printf(" T-");
        for (int k = 0; k < nin[i]; k++)
            printf("%s %s", k ? "," : "", nm[in[i][k]]);
    }
    int first = 1;
    for (int j = 0; j < nn; j++)
        for (int k = 0; k < nin[j]; k++)
            if (in[j][k] == i) {
                printf("%s %s", first ? " T+" : ",", nm[j]);
                first = 0; k = nin[j];
            }
    if (!nin[i] && first) printf(" (floating: reads 0)");
    putchar('\n');
    if (hold[i] >= 0) printf("  %s held at %d\n", nm[i], hold[i]);
}

int main(void) {
    char line[512], *tok[64];
    int tty = 1;
#ifdef __unix__
    tty = isatty(0);
#endif
    if (tty) puts("HATTAR — names, T-, T+, NAND. it's always six o'clock here.");
    for (;;) {
        if (tty) { fputs("hattar> ", stdout); fflush(stdout); }
        if (!fgets(line, sizeof line, stdin)) break;
        if (!tty) printf("hattar> %s", line);
        char *hash = strchr(line, '#'); if (hash) *hash = 0;
        int nt = 0;
        for (char *p = strtok(line, " ,\t\r\n"); p && nt < 64;
             p = strtok(NULL, " ,\t\r\n")) tok[nt++] = p;
        if (!nt) continue;

        if (!strcmp(tok[0], "quit") || !strcmp(tok[0], "exit")) break;

        if (!strcmp(tok[0], "t")) {
            int n = (nt > 1) ? atoi(tok[1]) : 1;
            if (n < 1) n = 1; if (n > HCAP) n = HCAP;
            for (int k = 0; k < n; k++) tick();
            printf("  t=%d\n", ticks);
            waves(n);
            continue;
        }
        if (!strcmp(tok[0], "w")) {
            nwatch = 0;
            for (int k = 1; k < nt && nwatch < 64; k++)
                watch[nwatch++] = intern(tok[k]);
            continue;
        }
        if (nt >= 2 && !strcmp(tok[1], "!")) {             /* pulse/hold */
            int i = intern(tok[0]);
            if (nt == 2)                    { pend[i] = 1; }
            else if (!strcmp(tok[2], "1"))  hold[i] = 1;
            else if (!strcmp(tok[2], "0"))  hold[i] = 0;
            else                            hold[i] = -1;   /* release  */
            continue;
        }
        if (nt >= 2 && !strcmp(tok[1], ":")) {             /* declare   */
            int i = intern(tok[0]), mode = 0;              /* 0=T- 1=T+ */
            int srcs[MAXIN], ns = 0, sawsrc = 0;
            for (int k = 2; k < nt; k++) {
                if (!strcmp(tok[k], "T-")) { mode = 0; sawsrc = 1; continue; }
                if (!strcmp(tok[k], "T+")) { mode = 1; continue; }
                int j = intern(tok[k]);
                if (mode == 0) { if (ns < MAXIN) srcs[ns++] = j; sawsrc = 1; }
                else if (nin[j] < MAXIN) in[j][nin[j]++] = i;
            }
            if (sawsrc) { nin[i] = ns; memcpy(in[i], srcs, sizeof srcs); }
            query(nm[i]);
            continue;
        }
        if (nt == 1) { query(tok[0]); continue; }
        puts("  ?  (node : T- a b T+ c | node | node ! [1 0 -] | w ... | t n | quit)");
    }
    return 0;
}
