#!/usr/bin/env python3
"""Measure mwaveform frame rate / loss on the RTT waveform TCP channel.

Connects to the OpenOCD RTT server (default localhost:9091), parses the
AA55-framed mwaveform protocol, and reports once per second:

  ok      - valid data frames received in this second
  crc     - frames dropped due to CRC mismatch (corrupted byte stream)
  seqgap  - data frames lost between device and host (chSeq discontinuity)

Cross-check with the device-side counters (shell: `wave drop`):
  produced (Total delta) = ok + seqgap + device-side drops
If device Drop=0/RTTFull=0 but seqgap or crc > 0, the loss is in the
OpenOCD/SWD/TCP path, not on the MCU.
"""

import socket
import sys
import time

SYNC_H, SYNC_L = 0xAA, 0x55
FRAME_TYPE_DESC = 0xFD
FRAME_TYPE_META = 0xFE
FRAME_TYPE_BATCH = 0xFC
FRAME_TYPE_SNAPSHOT = 0xFA


def crc8(data):
    c = 0xFF
    for b in data:
        c ^= b
    return c


def crc16(data):
    c = 0xFFFF
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def run(host, port):
    sock = socket.create_connection((host, port))
    sock.settimeout(2.0)
    buf = bytearray()
    mask_bytes = 1          # learned from descriptor frames
    last_seq = None
    n_ok = n_crc = n_gap = 0
    t_report = time.monotonic()

    print(f"connected to {host}:{port}, reporting every 1 s "
          f"(ok / crc_err / seq_lost)")
    while True:
        try:
            chunk = sock.recv(4096)
            if not chunk:
                print("connection closed by peer")
                break
        except socket.timeout:
            chunk = b""
        buf += chunk

        # parse as many complete frames as possible
        i = 0
        while True:
            # resync
            while i + 1 < len(buf) and not (buf[i] == SYNC_H and buf[i + 1] == SYNC_L):
                i += 1
            if i + 3 > len(buf):
                break
            third = buf[i + 2]
            if third == FRAME_TYPE_DESC:
                if i + 4 > len(buf):
                    break
                ch_count = buf[i + 3]
                flen = 4 + ch_count * 12 + 1
                if i + flen > len(buf):
                    break
                if crc8(buf[i + 2:i + flen - 1]) == buf[i + flen - 1]:
                    mask_bytes = max(1, (ch_count + 7) // 8)
                    i += flen
                else:
                    # Not a valid descriptor (could be a data frame with
                    # seq 0xFD on old firmware): resync by one byte.
                    n_crc += 1
                    i += 1
                continue
            if third == FRAME_TYPE_META:
                if i + 13 > len(buf):
                    break
                if crc8(buf[i + 2:i + 12]) == buf[i + 12]:
                    i += 13
                else:
                    n_crc += 1
                    i += 1
                continue
            if third in (FRAME_TYPE_BATCH, FRAME_TYPE_SNAPSHOT):
                min_header = 19 if third == FRAME_TYPE_SNAPSHOT else 15
                if i + min_header > len(buf):
                    break
                ch_count = buf[i + 4]
                sample_count = int.from_bytes(buf[i + 9:i + 11], "little")
                local_mask = max(1, (ch_count + 7) // 8)
                off = i + min_header
                incomplete = False
                for _ in range(sample_count):
                    if off + local_mask > len(buf):
                        incomplete = True
                        break
                    mask = buf[off:off + local_mask]
                    active = sum(bin(b).count("1") for b in mask)
                    off += local_mask + 2 * active
                if incomplete:
                    break
                flen = off - i + 2
                if i + flen > len(buf):
                    break
                if crc16(bytes(buf[i + 2:off])) == int.from_bytes(
                        buf[off:off + 2], "little"):
                    n_ok += 1
                    i += flen
                else:
                    n_crc += 1
                    i += flen
                continue
            # data frame: len = 3 + mask + 2*popcount(mask) + 1
            if i + 3 + mask_bytes > len(buf):
                break
            mask = buf[i + 3:i + 3 + mask_bytes]
            n_ch = sum(bin(b).count("1") for b in mask)
            flen = 3 + mask_bytes + 2 * n_ch + 1
            if i + flen > len(buf):
                break
            if crc8(buf[i + 2:i + flen - 1]) == buf[i + flen - 1]:
                n_ok += 1
                seq = third
                if last_seq is not None:
                    gap = (seq - last_seq - 1) & 0xFF
                    # Firmware never emits seq 0xFD (reserved frame type),
                    # so a gap spanning exactly that value is not a loss.
                    if gap == 1 and ((last_seq + 1) & 0xFF) == FRAME_TYPE_DESC:
                        gap = 0
                    n_gap += gap
                last_seq = seq
                i += flen
            else:
                n_crc += 1
                i += 1  # skip one byte, resync
        del buf[:i]

        now = time.monotonic()
        if now - t_report >= 1.0:
            dt = now - t_report
            t_report = now
            print(f"rate {n_ok / dt:7.1f} f/s   crc_err {n_crc:4d}   "
                  f"seq_lost {n_gap:4d}")
            n_ok = n_crc = n_gap = 0


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] in ("-h", "--help"):
        print("usage: wave_rate_check.py [host] [port]")
        sys.exit(0)
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 9091
    run(host, port)
