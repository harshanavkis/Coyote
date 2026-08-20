"""
Encrypt bank -> decrypt bank loopback.

Checks three things that lint cannot:

  1. Framing. Every frame keeps its byte count across both banks: the encrypt
     bank drops the 16-byte pad slot and its tag refills it, the decrypt bank
     drops the received tag and the computed tag refills it. Payload must come
     back byte-for-byte.

  2. It really is AES-GCM. The ciphertext between the banks is compared against
     Python's AES-256-GCM with the IV the positional scheme should have used,
     so a pipeline that is merely self-consistent cannot pass.

  3. Tag verification. tag_ok must pulse exactly once per frame, and the
     quarantine line must never assert.

Run:  make            (NUM_AES_ENGINES=2)
      make ENGINES=1  /  make ENGINES=4
"""

import os
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.backends import default_backend

DW = 512
BYTES_PER_BEAT = DW // 8
TAG = 16
KEY = bytes(32)          # upstream PoC key
IV_DIR = 0

# 16-byte-aligned, >= 32 so the tag gets its own 128-bit beat. Deliberately
# includes sizes that are not a multiple of the 64-byte bus width, to exercise
# a partial final beat, and a repeat so the IV counter has to keep stepping.
FRAME_SIZES = [64, 64, 128, 80, 256, 64, 176, 128]


def gcm_encrypt(iv_int, plaintext):
    enc = Cipher(algorithms.AES(KEY),
                 modes.GCM(iv_int.to_bytes(12, "big")),
                 backend=default_backend()).encryptor()
    ct = enc.update(plaintext) + enc.finalize()
    return ct, enc.tag


def beats(payload):
    """Split a byte string into (tdata_int, tkeep_int, last) bus beats."""
    out = []
    for off in range(0, len(payload), BYTES_PER_BEAT):
        chunk = payload[off:off + BYTES_PER_BEAT]
        data = int.from_bytes(chunk.ljust(BYTES_PER_BEAT, b"\x00"), "little")
        keep = (1 << len(chunk)) - 1
        out.append((data, keep, off + BYTES_PER_BEAT >= len(payload)))
    return out


async def send_frames(dut, frames):
    dut.s_axis_tvalid.value = 0
    for frame in frames:
        for data, keep, last in beats(frame):
            dut.s_axis_tdata.value = data
            dut.s_axis_tkeep.value = keep
            dut.s_axis_tlast.value = 1 if last else 0
            dut.s_axis_tvalid.value = 1
            await RisingEdge(dut.clk)
            while dut.s_axis_tready.value != 1:
                await RisingEdge(dut.clk)
        dut.s_axis_tvalid.value = 0
        dut.s_axis_tlast.value = 0


async def collect(sig_prefix, dut, n_frames, sink):
    """Reassemble n_frames from an AXI-Stream, returning a list of byte strings."""
    tdata = getattr(dut, sig_prefix + "_tdata")
    tkeep = getattr(dut, sig_prefix + "_tkeep")
    tvalid = getattr(dut, sig_prefix + "_tvalid")
    tready = getattr(dut, sig_prefix + "_tready")
    tlast = getattr(dut, sig_prefix + "_tlast")

    cur = bytearray()
    while len(sink) < n_frames:
        await RisingEdge(dut.clk)
        if tvalid.value == 1 and tready.value == 1:
            raw = int(tdata.value).to_bytes(BYTES_PER_BEAT, "little")
            keep = int(tkeep.value)
            cur += bytes(raw[i] for i in range(BYTES_PER_BEAT) if (keep >> i) & 1)
            if tlast.value == 1:
                sink.append(bytes(cur))
                cur = bytearray()


async def count_tag_ok(dut, counter):
    while True:
        await RisingEdge(dut.clk)
        counter[0] += int(dut.tag_ok_cnt.value)   # engines verifying this cycle
        if dut.quarantine.value == 1:
            counter[1] += 1


async def watch_tags(dut, log):
    """Record every tag comparison decrypt engine 0 performs."""
    while True:
        await RisingEdge(dut.clk)
        if (dut.dbg_eng_out_tvalid.value == 1 and dut.dbg_eng_out_tready.value == 1
                and dut.dbg_is_tag.value == 1):
            comp = int(dut.dbg_computed_tag.value).to_bytes(16, "little")
            exp = int(dut.dbg_expected_tag.value).to_bytes(16, "little")
            log.append((int(dut.dbg_rx_tag_rd.value), comp, exp))


@cocotb.test()
async def bank_loopback(dut):
    n_eng = int(os.environ.get("ENGINES", "2"))
    dut._log.info("NUM_AES_ENGINES = %d", n_eng)

    cocotb.start_soon(Clock(dut.clk, 4, unit="ns").start())

    dut.iv_dir.value = IV_DIR          # must be stable before reset releases
    dut.rst.value = 1
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tlast.value = 0
    dut.s_axis_tdata.value = 0
    dut.s_axis_tkeep.value = 0
    dut.m_axis_tready.value = 1
    await ClockCycles(dut.clk, 20)
    dut.rst.value = 0

    # The engines stream 64 warmup blocks through the AES pipe after reset and
    # hold s_tready low meanwhile (~150 cycles).
    for _ in range(2000):
        await RisingEdge(dut.clk)
        if dut.s_axis_tready.value == 1:
            break
    else:
        assert False, "engines never asserted tready after reset"
    dut._log.info("engines ready")

    # Frame k carries (size - 16) bytes of payload plus a 16-byte pad slot.
    payloads = [bytes((i * 7 + j) & 0xFF for j in range(sz - TAG))
                for i, sz in enumerate(FRAME_SIZES)]
    frames = [p + b"\xA5" * TAG for p in payloads]

    tags = [0, 0]
    tag_log = []
    cocotb.start_soon(count_tag_ok(dut, tags))
    cocotb.start_soon(watch_tags(dut, tag_log))

    ct_frames, pt_frames = [], []
    cocotb.start_soon(collect("mid", dut, len(frames), ct_frames))
    collector = cocotb.start_soon(collect("m_axis", dut, len(frames), pt_frames))

    await send_frames(dut, frames)

    dut._log.info("all %d frames driven in", len(frames))

    def snap():
        g = lambda n: int(getattr(dut, n).value)
        return (f"cr={g('dbg_credits')} pause={g('dbg_pause')} tagok={g('dbg_tag_ok0')} "
                f"engout(v/r/last/istag)={g('dbg_eng_out_tvalid')}/{g('dbg_eng_out_tready')}/"
                f"{g('dbg_eng_out_tlast')}/{g('dbg_is_tag')} "
                f"rxtag wr/rd={g('dbg_rx_tag_wr')}/{g('dbg_rx_tag_rd')} "
                f"ct(v/r)={g('dbg_ct_tvalid')}/{g('dbg_ct_tready')} "
                f"fifo(v/r)={g('dbg_fifo_tvalid')}/{g('dbg_fifo_tready')} "
                f"coll(v/r)={g('dbg_coll_tvalid')}/{g('dbg_coll_tready')}")

    last_seen = (-1, -1)
    stalled = 0
    for i in range(4000):
        await RisingEdge(dut.clk)
        if (len(ct_frames), len(pt_frames)) != last_seen:
            last_seen = (len(ct_frames), len(pt_frames))
            stalled = 0
            dut._log.info("cycle %d: ct=%d pt=%d | %s", i, *last_seen, snap())
        else:
            stalled += 1
            if 300 < stalled < 316:
                dut._log.info("STALL+%d cycle %d | %s", stalled, i, snap())
        if (len(ct_frames), len(pt_frames)) != last_seen:
            last_seen = (len(ct_frames), len(pt_frames))
            dut._log.info("cycle %d: ciphertext frames=%d, plaintext frames=%d",
                          i, last_seen[0], last_seen[1])
        if len(pt_frames) == len(frames):
            break
        timed_out = False
    else:
        timed_out = True
    collector.kill()

    errors = []
    if timed_out:
        errors.append(f"timeout: {len(pt_frames)}/{len(frames)} plaintext frames, "
                      f"{len(ct_frames)}/{len(frames)} ciphertext frames, "
                      f"tag_ok={tags[0]}, quarantine={tags[1]} cycles")

    # engine 0 handles frames 0, 2, 4, ... -- show every tag comparison it made
    dut._log.info("decrypt engine 0 tag comparisons (%d):", len(tag_log))
    for idx, (rd, comp, exp) in enumerate(tag_log):
        frame_k = idx * n_eng
        pt = payloads[frame_k] if frame_k < len(payloads) else b""
        _, sw_tag = gcm_encrypt((IV_DIR << 95) | frame_k, pt)
        dut._log.info("  rd=%d frame%d %s computed=%s received=%s sw(iv=%d)=%s",
                      rd, frame_k, "MATCH" if comp == exp else "MISMATCH",
                      comp.hex(), exp.hex(), frame_k, sw_tag.hex())

    # 1. framing + payload round trip
    for k, (want, got) in enumerate(zip(payloads, pt_frames)):
        if len(got) != len(want) + TAG:
            errors.append(f"frame {k}: length {len(got)}, expected {len(want) + TAG}")
            continue
        if got[:len(want)] != want:
            errors.append(f"frame {k}: payload mismatch")

    # 2. the ciphertext is real AES-GCM under the positional IV
    for k, (pt, ct_frame) in enumerate(zip(payloads, ct_frames)):
        iv = (IV_DIR << 95) | k
        exp_ct, exp_tag = gcm_encrypt(iv, pt)
        if len(ct_frame) != len(pt) + TAG:
            errors.append(f"frame {k}: ciphertext length {len(ct_frame)}")
            continue
        if ct_frame[:len(pt)] != exp_ct:
            errors.append(f"frame {k}: ciphertext != AES-GCM(iv={k})")
        if ct_frame[len(pt):] != exp_tag:
            errors.append(f"frame {k}: tag != AES-GCM tag(iv={k})")

    # 3. tag verification
    if tags[0] != len(frames):
        errors.append(f"tag_ok pulsed {tags[0]}x, expected {len(frames)}")
    if tags[1] != 0:
        errors.append(f"quarantine asserted for {tags[1]} cycles")

    for e in errors:
        dut._log.error(e)
    assert not errors, f"{len(errors)} check(s) failed"
    dut._log.info("all %d frames round-tripped, ciphertext matches AES-GCM, "
                  "%d tags verified", len(frames), tags[0])
