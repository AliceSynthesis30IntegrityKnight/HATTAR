"""

validity_tester.py -- bespoke processor-block verification for HATTAR hardware.


DUT:     4-bit ripple-carry adder, 60 NAND gates, generated as HATTAR netlist

Golden:  plain Python arithmetic (a + b + cin)

Method:  exhaustive stimulus -- all 512 input vectors (16 x 16 x 2),

         inputs re-asserted every settle tick (HATTAR inputs are floating

         nodes, so the testbench drives them like a pattern generator),

         outputs read back via HATTAR queries and checked bit-for-bit.


Usage:

    python3 validity_tester.py ./hattar_jit            # verify good design

    python3 validity_tester.py ./hattar_jit --mutate   # plant a wiring bug,

                                                       # prove the tester sees it


Every NAND in HATTAR is registered (one tick of latency), so combinational

logic settles after 'depth' ticks -- the harness allows SETTLE=30, well past

the adder's ~12-gate critical path through the carry chain.

"""

import re

import subprocess

import sys


SETTLE = 30

INPUTS = ["a0", "a1", "a2", "a3", "b0", "b1", "b2", "b3", "cin"]

OUTPUTS = ["s0", "s1", "s2", "s3", "cout"]



def xor_nand(out, x, y):

    """XOR from 4 NANDs."""

    return [

        f"{out}_n1 : T- {x} {y}",

        f"{out}_n2 : T- {x} {out}_n1",

        f"{out}_n3 : T- {y} {out}_n1",

        f"{out} : T- {out}_n2 {out}_n3",

    ]



def full_adder(i, a, b, c, s, cout):

    """sum = a^b^c ; cout = NAND(NAND(a,b), NAND(c, a^b)). 9 NANDs."""

    lines = []

    lines += xor_nand(f"x{i}", a, b)

    lines += xor_nand(s, f"x{i}", c)

    lines += [

        f"nab{i} : T- {a} {b}",

        f"ncx{i} : T- {c} x{i}",

        f"{cout} : T- nab{i} ncx{i}",

    ]

    return lines



def build_netlist(mutate=False):

    lines = []

    carries = ["cin", "c1", "c2", "c3"]

    for i in range(4):

        cout = f"c{i+1}" if i < 3 else "cout"

        lines += full_adder(i, f"a{i}", f"b{i}", carries[i], f"s{i}", cout)

    if mutate:

        # planted fault: bit 2's sum taps carry c1 instead of c2

        for k, ln in enumerate(lines):

            if ln.startswith("s2_n1 :"):

                lines[k] = "s2_n1 : T- x2 c1"

            if ln.startswith("s2_n2 :"):

                lines[k] = "s2_n2 : T- x2 s2_n1"

            if ln.startswith("s2_n3 :"):

                lines[k] = "s2_n3 : T- c1 s2_n1"

    return lines



def stimulus(a, b, cin):

    """Drive one vector: re-poke inputs each settle tick, then read outputs."""

    bits = {f"a{i}": (a >> i) & 1 for i in range(4)}

    bits.update({f"b{i}": (b >> i) & 1 for i in range(4)})

    bits["cin"] = cin

    lines = []

    for _ in range(SETTLE):

        lines += [f"{name} ! {v}" for name, v in bits.items()]

        lines.append("t 1")

    lines += OUTPUTS  # bare-name queries

    return lines



def golden(a, b, cin):

    total = a + b + cin

    return {**{f"s{i}": (total >> i) & 1 for i in range(4)},

            "cout": (total >> 4) & 1}



def main():

    engine = sys.argv[1] if len(sys.argv) > 1 else "./hattar_jit"

    mutate = "--mutate" in sys.argv


    netlist = build_netlist(mutate)

    ngates = sum(1 for ln in netlist)

    session = list(netlist)

    vectors = [(a, b, c) for a in range(16) for b in range(16) for c in (0, 1)]

    for a, b, c in vectors:

        session += stimulus(a, b, c)


    proc = subprocess.run([engine], input="\n".join(session) + "\n",

                          capture_output=True, text=True)

    reads = re.findall(r"^(\w+)=([01])", proc.stdout, re.M)

    per_vec = len(OUTPUTS)

    assert len(reads) == len(vectors) * per_vec, \

        f"expected {len(vectors)*per_vec} readbacks, got {len(reads)}"


    fails = []

    for vi, (a, b, c) in enumerate(vectors):

        got = {name: int(v) for name, v in reads[vi*per_vec:(vi+1)*per_vec]}

        want = golden(a, b, c)

        if got != want:

            fails.append((a, b, c, got, want))


    tag = "MUTANT" if mutate else "DUT"

    print(f"[{tag}] 4-bit ripple adder: {ngates} NAND declarations, "

          f"{len(vectors)} exhaustive vectors, settle={SETTLE} ticks")

    if not fails:

        print(f"[{tag}] PASS {len(vectors)}/{len(vectors)} -- "

              f"design is arithmetically valid")

    else:

        print(f"[{tag}] FAIL {len(fails)}/{len(vectors)} vectors:")

        for a, b, c, got, want in fails[:5]:

            gs = sum(got[f's{i}'] << i for i in range(4)) | (got['cout'] << 4)

            print(f"    {a:2d} + {b:2d} + {c}  ->  got {gs:2d}, "

                  f"want {a+b+c:2d}")

        if len(fails) > 5:

            print(f"    ... and {len(fails)-5} more")

    sys.exit(1 if fails else 0)



if __name__ == "__main__":

    main()
