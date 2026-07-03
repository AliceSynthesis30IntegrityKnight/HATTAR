/* HATTAR-JIT — the netlist compiles itself.
 *
 * Same language as the kernel:  name : T- a b T+ c | name | name ! v | t n
 * Plus:                         jit 0|1     (default 1)
 *
 * On the first `t` after any structural change, the engine emits a flat,
 * fully unrolled C function for the whole circuit:
 *
 *     b[0] = !(a[3] & a[7]);      // every gate, one expression
 *     b[1] = !(a[0]);
 *     b[2] = 0;                   // floating: constant
 *
 * ...compiles it with `cc -O2 -shared`, dlopen()s the result into itself,
 * and runs ticks at native speed with double-buffer pointer swapping (no
 * memcpy per tick). gcc becomes the logic synthesizer: constant folding,
 * CSE and vectorization all happen to your circuit for free.
 *
 * Structural edits set a dirty flag -> transparent recompile on next `t`.
 * If the compiler is unavailable, falls back to the interpreter.
 *
 *   cc -O2 -o hattar_jit hattar_jit.c
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dlfcn.h>
#include <unistd.h>

#define MAXN 999
char N[MAXN][32]; int E[MAXN][16], ne[MAXN], n, s[MAXN], x[MAXN];
int dirty = 1, jit_on = 1;
void *dlh = 0;
void (*run_fn)(int *, int *, long) = 0;

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

int jit_build(void) {
    char cpath[64], sopath[64], cmd[192];
    snprintf(cpath, 64, "/tmp/hattar_%d.c", getpid());
    snprintf(sopath, 64, "/tmp/hattar_%d.so", getpid());
    FILE *f = fopen(cpath, "w");
    if (!f) return 0;
    fprintf(f, "#include <string.h>\n"
               "static void step(const int*restrict a,int*restrict b){\n");
    for (int i = 0; i < n; i++) {
        if (!ne[i]) { fprintf(f, "b[%d]=0;\n", i); continue; }
        fprintf(f, "b[%d]=!(", i);
        for (int j = 0; j < ne[i]; j++)
            fprintf(f, "%sa[%d]", j ? "&" : "", E[i][j]);
        fprintf(f, ");\n");
    }
    fprintf(f, "}\n"
               "void run(int*s,int*x,long k){int*a=s,*b=x;\n"
               "for(;k>0;k--){step(a,b);int*t=a;a=b;b=t;}\n"
               "if(a!=s)memcpy(s,a,%d*sizeof(int));}\n", n);
    fclose(f);
    snprintf(cmd, 192, "cc -O2 -shared -fPIC -o %s %s 2>/dev/null", sopath, cpath);
    if (system(cmd)) return 0;
    if (dlh) dlclose(dlh);
    dlh = dlopen(sopath, RTLD_NOW);
    if (!dlh) return 0;
    run_fn = (void (*)(int *, int *, long))dlsym(dlh, "run");
    return run_fn != 0;
}

void ticks(long k) {
    if (jit_on && n) {
        if (dirty) {
            if (jit_build()) {
                printf("  [jit: %d gates compiled -> native circuit]\n", n);
                dirty = 0;
            } else {
                printf("  [jit: compiler unavailable, interpreting]\n");
                jit_on = 0;
            }
        }
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
            if (!(t = strtok(0, " ,\n"))) {                      /* query */
                printf("%s=%d  T-", N[a], s[a]);
                for (int j = 0; j < ne[a]; j++) printf(" %s", N[E[a][j]]);
                printf("  T+");
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
