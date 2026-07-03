#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "libtcc.h"

#define MAXN 999
char N[MAXN][32]; int E[MAXN][16], ne[MAXN], n, s[MAXN], x[MAXN];
int dirty = 1, jit_on = 1;
TCCState *live = 0;
void (*run_fn)(int *, int *, long) = 0;
char SRC[1 << 20];

int id(char *w) {
    for (int i = 0; i < n; i++) if (!strcmp(N[i], w)) return i;
    dirty = 1;
    return strcpy(N[n], w), n++;
}

void tick_interp(long k) {
    for (; k > 0; k--, memcpy(s, x, sizeof s))
        for (int i = 0; i < n; i++) {
            int v = 1;
            for (int j = 0; j < ne[i]; j++) v &= s[E[i][j]];
            x[i] = !v;
        }
}

void err_cb(void *opq, const char *msg) { fprintf(stderr, "  [tcc] %s\n", msg); }

int jit_build(void) {
    int len = 0;
    len += sprintf(SRC + len,
        "static void step(const int*a,int*b){\n");
    for (int i = 0; i < n; i++) {
        if (!ne[i]) { len += sprintf(SRC + len, "b[%d]=0;\n", i); continue; }
        len += sprintf(SRC + len, "b[%d]=!(", i);
        for (int j = 0; j < ne[i]; j++)
            len += sprintf(SRC + len, "%sa[%d]", j ? "&" : "", E[i][j]);
        len += sprintf(SRC + len, ");\n");
    }
    len += sprintf(SRC + len,
        "}\nvoid run(int*s,int*x,long k){int*a=s,*b=x;\n"
        "for(;k>0;k--){step(a,b);int*t=a;a=b;b=t;}\n"
        "if(a!=s)for(int i=0;i<%d;i++)s[i]=a[i];}\n", n);

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
    if (live) tcc_delete(live);           /* old circuit's memory dies here */
    live = st;
    run_fn = (void (*)(int *, int *, long))fp;
    printf("  [libtcc: %d gates -> executable memory in %.1f ms]\n",
           n, 1000.0 * (clock() - t0) / CLOCKS_PER_SEC);
    return 1;
}

void ticks(long k) {
    if (jit_on && n) {
        if (dirty && jit_build()) dirty = 0;
        if (!dirty && run_fn) { run_fn(s, x, k); return; }
    }
    tick_interp(k);
}

int main() {
    char L[256], *t;
    while (fgets(L, 256, stdin)) {
        if (!(t = strtok(L, " ,\n"))) continue;
        if (!strcmp(t, "t")) {
            ticks((t = strtok(0, " \n")) ? atol(t) : 1);
        } else if (!strcmp(t, "jit")) {
            jit_on = (t = strtok(0, " \n")) ? atoi(t) : 1;
            dirty = 1;
        } else {
            int a = id(t), m = 0;
            if (!(t = strtok(0, " ,\n"))) {
                printf("%s=%d  T-", N[a], s[a]);
                for (int j = 0; j < ne[a]; j++) printf(" %s", N[E[a][j]]);
                printf("  T+");
                for (int i = 0; i < n; i++) for (int j = 0; j < ne[i]; j++)
                    if (E[i][j] == a) { printf(" %s", N[i]); break; }
                puts("");
            } else if (!strcmp(t, "!")) s[a] = (t = strtok(0, " \n")) ? atoi(t) : 1;
            else { dirty = 1;
                for (ne[a] = 0; t; t = strtok(0, " ,\n")) {
                    if (!strcmp(t, ":") || !strcmp(t, "T-")) m = 0;
                    else if (!strcmp(t, "T+")) m = 1;
                    else { int b = id(t); if (m) E[b][ne[b]++] = a; else E[a][ne[a]++] = b; }
                }
            }
        }
    }
    return 0;
}
