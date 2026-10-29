#!/usr/bin/env python3
"""gen_icrc_fast.py IN OUT: icrc.sv + single-cycle CRC stages for 44 B (CRC352: an ACK) and 56 B (CRC448: WRITE_ONLY /
FIRST with a 4 KiB payload) last words, as the existing CRC320 (40 B) stage; those keeps no longer stall the input for the
CRC32 pipeline; and the first word of a packet is masked by an input-side flag (stage 2's flag was one cycle late). 2026-10-28 (Loom both-way limit: WRITE_ONLY + ACK cost icrc 95 input cycles per packet, line is ~84).
Equations: reflected CRC32 (0xEDB88320), data bits LSB first, as the file's CRC512/CRC320 (checked against them)."""
import re, sys

src = open(sys.argv[1]).read()
RPOLY = 0xEDB88320


def deps(N):
    # crc' = L(d[0..N-1], c[0..31]); track each crc bit as a set of input symbols
    crc = [{('c', i)} for i in range(32)]
    for i in range(N):
        fb = crc[0] ^ {('d', i)}
        crc = [crc[k + 1] if k < 31 else set() for k in range(32)]
        crc = [crc[k] ^ fb if (RPOLY >> k) & 1 else crc[k] for k in range(32)]
    return crc


def stage_eqs(name, N):
    lines = []
    for b in range(31, -1, -1):
        s = deps(N)[b] if False else None
    D = deps(N)
    for b in range(31, -1, -1):
        ds = sorted(i for t, i in D[b] if t == 'd')
        cs = sorted(i for t, i in D[b] if t == 'c')
        terms = [f'd[{N - 1}-{N - 1 - i}]' for i in ds] + [f'c[31-{31 - i}]' for i in cs]
        lines.append(f'                stage_{name}_crc[{b}] <= ' + ' ^ '.join(terms) + '; ')
    return '\n'.join(lines)


def regs(name):
    return (f'// Regs for the {name.upper()}-pipeline stage (2026-10-28)\n'
            f'logic stage_{name}_valid; \nDataWord stage_{name}_data_bypass; \nCRCWord stage_{name}_crc; \n'
            f'KeepWord stage_{name}_keep; \nlogic stage_{name}_last; \n\n')


def clr(name, ind):
    return ''.join(f'{ind}stage_{name}_{f} <= {v}; \n' for f, v in
                   (('valid', "1'b0"), ('last', "1'b0"), ('keep', "64'b0"), ('data_bypass', "512'b0"), ('crc', "32'b0")))


NEW = (('crc352', 352, "64'h00000fffffffffff"), ('crc448', 448, "64'h00ffffffffffffff"))

# 1 registers, after the CRC320 regs
anchor = 'logic stage_crc320_last; \n'
assert src.count(anchor) == 1
src = src.replace(anchor, anchor + '\n' + ''.join(regs(n) for n, _, _ in NEW), 1)

# 2 reset, after the CRC320 reset block
anchor = "        stage_crc320_last <= 1'b0; \n"
i = src.index(anchor) + len(anchor)
src = src[:i] + ''.join(clr(n, '        ') for n, _, _ in NEW) + src[i:]

# 3 stall condition: the new keeps no longer wait for the CRC32 pipeline
old = "(stage_masking_keep != 64'h000000ffffffffff) && (stage_masking_keep != 64'h0))"
assert src.count(old) == 1
src = src.replace(old, "(stage_masking_keep != 64'h000000ffffffffff) && " +
                  ' && '.join(f'(stage_masking_keep != {k})' for _, _, k in NEW) + " && (stage_masking_keep != 64'h0))", 1)

# 4 stage 2: defaults (a later assignment in the matching branch wins), then one branch each before the CRC32 branch
anchor = '        if(stage_masking_valid) begin \n'
assert src.count(anchor) == 1
src = src.replace(anchor, '        // CRC352 / CRC448 idle unless their branch below fires (2026-10-28)\n' +
                  ''.join(f"        stage_{n}_valid <= 1'b0; \n        stage_{n}_last <= 1'b0; \n" for n, _, _ in NEW) + anchor, 1)
anchor = '            end else begin \n\n                // === CRC32 === '
assert src.count(anchor) == 1
br = ''
for n, N, k in NEW:
    br += (f'            end else if(stage_masking_keep == {k}) begin\n\n'
           f'                // === {n.upper()} (2026-10-28) ===\n\n'
           f'                stage_{n}_valid <= stage_masking_valid; \n                stage_{n}_last <= stage_masking_last; \n'
           f'                stage_{n}_keep <= stage_masking_keep; \n                stage_{n}_data_bypass <= stage_masking_data; \n\n'
           + clr('crc512', '                ') + clr('crc320', '                ') +
           '\n                for(integer pipeline_stage = 0; pipeline_stage < 16; pipeline_stage++) begin\n'
           "                    stage_crc32_valid[pipeline_stage] <= 1'b0; \n                    stage_crc32_last[pipeline_stage] <= 1'b0; \n"
           "                    stage_crc32_keep[pipeline_stage] <= 64'b0; \n                    stage_crc32_data_bypass[pipeline_stage] <= 512'b0; \n"
           "                    stage_crc32_crc[pipeline_stage] <= 32'b0; \n                end \n\n"
           + stage_eqs(n, N) + '\n\n')
src = src.replace(anchor, br + anchor, 1)

# 5 join: after the CRC320 branch
anchor = '        end else if(stage_crc32_valid[15]) begin \n'
assert src.count(anchor) == 1
j = ''
for n, _, _ in NEW:
    j += (f'        end else if(stage_{n}_valid) begin \n            stage_join_valid <= stage_{n}_valid; \n'
          f'            stage_join_data <= stage_{n}_data_bypass; \n            stage_join_crc <= stage_{n}_crc ^ 32\'hffffffff; \n'
          f'            stage_join_keep <= stage_{n}_keep; \n            stage_join_last <= stage_{n}_last; \n')
src = src.replace(anchor, j + anchor, 1)
# 6 stage 1 masks a packet's FIRST word, decided at the input: stage_crc_follow_up_word is updated by stage 2 in the same
#   cycle, so a first word right behind a last word that did not stall the input (a 40/44/56 B tail; before: 40 B only)
#   was loaded unmasked -> wrong ICRC. in_first = the next input word starts a packet.
old = '                if(stage_crc_follow_up_word == 0) begin \n'
assert src.count(old) == 1
src = src.replace(old, '                if(in_first) begin   // (2026-10-28: was stage_crc_follow_up_word == 0, one cycle late)\n', 1)
anchor = 'logic stage_crc_follow_up_word; \n'
assert src.count(anchor) == 1
src = src.replace(anchor, anchor + "logic in_first;   // 2026-10-28: the next input word is a packet's first (set by the last accepted word's tlast)\n", 1)
anchor = "assign m_axis_rx.tready = ~halffull_signal & ~crc32_in_flight_comb & ~full_signal; \n"
assert src.count(anchor) == 1
src = src.replace(anchor, anchor + "always_ff @(posedge nclk) begin\n    if(!nresetn) in_first <= 1'b1;\n"
                  "    else if(m_axis_rx.tvalid && m_axis_rx.tready) in_first <= m_axis_rx.tlast;\nend\n", 1)
open(sys.argv[2], 'w').write(src)
print('written', sys.argv[2])
