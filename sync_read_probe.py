#!/usr/bin/env python3
"""
sync_read_probe.py — wire-level diagnostic for the Feetech STS sync_read failure.

Answers the one question nothing so far has settled: when sync_read fails, is the
reply ABSENT (servos never answered) or MISFRAMED (bytes arrived, parser lost them)?

It bypasses the ROS driver entirely and talks raw bytes with pyserial, dumping
everything received as hex. It runs three probes back-to-back on the same port,
same baud, same session, so they are directly comparable:

  PROBE 1  single READ  of each servo, one at a time   (known-good baseline)
  PROBE 2  SYNC READ    of all servos in one packet    (the failing case)
  PROBE 3  SYNC READ    again, with the RX buffer NOT pre-flushed, to expose
                        half-duplex TX echo if the adapter loops it back

STOP THE ROS STACK FIRST — the port must be free:
    sudo lsof /dev/so101_follower      # should print nothing
    pip install pyserial --user

Usage:
    python3 sync_read_probe.py                       # defaults below
    python3 sync_read_probe.py --port /dev/ttyACM0 --baud 1000000 --ids 1 2 3 4 5 6
    python3 sync_read_probe.py --baud 500000         # for the CH340 clock-error test

Read-only: sends no write instructions, touches no EEPROM.
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing:  pip install pyserial --user")

# --- STS/SMS protocol constants ---
HDR = b"\xff\xff"
INST_PING = 0x01
INST_READ = 0x02
INST_SYNC_READ = 0x82
BROADCAST_ID = 0xFE
ADDR_PRESENT_POSITION = 56   # STS3215 present position, 2 bytes
LEN_PRESENT_POSITION = 4


def checksum(payload: bytes) -> int:
    # Feetech: ~(ID + Length + Instruction + params) & 0xFF
    return (~sum(payload)) & 0xFF


def build_packet(dev_id: int, instruction: int, params: bytes = b"") -> bytes:
    length = len(params) + 2                      # instruction + params + checksum
    body = bytes([dev_id, length, instruction]) + params
    return HDR + body + bytes([checksum(body)])


def build_read(dev_id: int, addr: int, n: int) -> bytes:
    return build_packet(dev_id, INST_READ, bytes([addr, n]))


def build_sync_read(ids, addr: int, n: int) -> bytes:
    # SYNC READ params: start_addr, data_len, id1..idN  (broadcast ID)
    return build_packet(BROADCAST_ID, INST_SYNC_READ, bytes([addr, n]) + bytes(ids))


def hexdump(b: bytes) -> str:
    if not b:
        return "<nothing>"
    return " ".join(f"{x:02X}" for x in b)


def parse_status_packets(buf: bytes):
    """Walk the buffer and pull out every well-formed status packet found."""
    out, i = [], 0
    while i < len(buf) - 3:
        if buf[i] == 0xFF and buf[i + 1] == 0xFF:
            dev_id, length = buf[i + 2], buf[i + 3]
            end = i + 4 + length
            if length >= 2 and end <= len(buf):
                body = buf[i + 2: end - 1]
                got, want = buf[end - 1], checksum(body)
                err = buf[i + 4]
                data = buf[i + 5: end - 1]
                out.append({
                    "id": dev_id, "err": err, "data": bytes(data),
                    "csum_ok": got == want, "raw": bytes(buf[i:end]),
                })
                i = end
                continue
        i += 1
    return out


def decode_pos(data: bytes):
    if len(data) >= 2:
        return data[0] | (data[1] << 8)      # little-endian
    return None


def drain(ser, settle=0.05):
    time.sleep(settle)
    n = ser.in_waiting
    return ser.read(n) if n else b""


def collect(ser, wait_s: float) -> bytes:
    """Read everything that arrives within wait_s, stopping on a quiet gap."""
    deadline = time.time() + wait_s
    buf = b""
    quiet = 0
    while time.time() < deadline:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
            quiet = 0
        else:
            time.sleep(0.002)
            quiet += 1
            if buf and quiet > 25:      # ~50 ms of silence after data = done
                break
    return buf


def probe_single(ser, ids, wait_s):
    print("\n" + "=" * 70)
    print("PROBE 1 — single READ per servo (the known-good path)")
    print("=" * 70)
    ok = 0
    for i in ids:
        pre = drain(ser)
        if pre:
            print(f"  [id {i}] stale bytes before TX: {hexdump(pre)}")
        pkt = build_read(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        ser.write(pkt)
        ser.flush()
        buf = collect(ser, wait_s)
        pkts = parse_status_packets(buf)
        mine = [p for p in pkts if p["id"] == i]
        status = "OK " if mine else "MISS"
        pos = decode_pos(mine[0]["data"]) if mine else None
        print(f"  [id {i}] TX {hexdump(pkt)}")
        print(f"         RX {hexdump(buf)}")
        print(f"         -> {status}  packets={len(pkts)}  pos={pos}")
        if mine:
            ok += 1
    print(f"\n  RESULT: {ok}/{len(ids)} servos answered individually.")
    return ok


def probe_sync(ser, ids, wait_s, flush_first=True, label="PROBE 2"):
    print("\n" + "=" * 70)
    print(f"{label} — SYNC READ, all servos in one packet"
          f"{'' if flush_first else '  (RX NOT pre-flushed — echo check)'}")
    print("=" * 70)
    if flush_first:
        pre = drain(ser)
        if pre:
            print(f"  stale bytes before TX: {hexdump(pre)}")
    pkt = build_sync_read(ids, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    ser.write(pkt)
    ser.flush()
    buf = collect(ser, wait_s)
    print(f"  TX ({len(pkt)} B) {hexdump(pkt)}")
    print(f"  RX ({len(buf)} B) {hexdump(buf)}")

    echoed = buf.startswith(pkt)
    if echoed:
        print("\n  >>> TX ECHO DETECTED: the reply stream begins with our own "
              "request. Half-duplex loopback. <<<")
        buf_after = buf[len(pkt):]
        print(f"  after stripping echo ({len(buf_after)} B): {hexdump(buf_after)}")
    else:
        buf_after = buf

    pkts = parse_status_packets(buf_after)
    answered = {p["id"] for p in pkts}
    for p in pkts:
        print(f"    id={p['id']} err={p['err']} pos={decode_pos(p['data'])} "
              f"csum={'ok' if p['csum_ok'] else 'BAD'}")
    missing = [i for i in ids if i not in answered]
    print(f"\n  RESULT: {len(answered)}/{len(ids)} answered."
          f"{'  missing: ' + str(missing) if missing else ''}")
    return len(buf), len(answered), echoed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/so101_follower")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6])
    ap.add_argument("--wait", type=float, default=0.5,
                    help="seconds to listen after each request")
    args = ap.parse_args()

    print(f"port={args.port}  baud={args.baud}  ids={args.ids}  wait={args.wait}s")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0)
    except Exception as e:
        sys.exit(f"could not open port: {e}\n"
                 "Is the ROS stack still running?  sudo lsof " + args.port)

    time.sleep(0.2)
    drain(ser)

    n_single = probe_single(ser, args.ids, args.wait)
    rx_len, n_sync, echoed = probe_sync(ser, args.ids, args.wait, True, "PROBE 2")
    probe_sync(ser, args.ids, args.wait, False, "PROBE 3")

    ser.close()

    print("\n" + "=" * 70)
    print("VERDICT")
    print("=" * 70)
    if n_single == 0:
        print("  Single reads failed too -> not a sync_read problem. Check port,")
        print("  baud, power, and that nothing else holds the bus.")
    elif n_sync == len(args.ids):
        print("  sync_read WORKS at the wire level.")
        print("  -> The servos and adapter are fine. The bug is in the ROS driver's")
        print("     sync_read implementation or its timing, NOT the hardware.")
        print("     Next: hypothesis E (patch driver to loop single reads) is safe,")
        print("     but first compare this TX packet against what the driver emits.")
    elif rx_len == 0:
        print("  Servos returned ABSOLUTELY NOTHING to sync_read, but answer single")
        print("  reads -> firmware likely does not implement SYNC READ (0x82).")
        print("  -> Hypothesis B confirmed. No timeout value will ever fix this.")
        print("     Go straight to hypothesis E: loop single reads in the driver.")
    elif echoed:
        print("  Bytes came back but began with our OWN request (half-duplex echo).")
        print("  -> Framing/echo problem, not absence. The driver's check_head is")
        print("     locking onto the echo. Fix = flush RX after TX, or skip len(TX)")
        print("     bytes before parsing.")
    else:
        print(f"  PARTIAL/GARBLED: {rx_len} bytes back, {n_sync}/{len(args.ids)}")
        print("  parsed. Bytes ARE arriving -> not absence, it's framing or")
        print("  buffering (hypothesis C or D). Retry at --baud 500000: if it")
        print("  cleans up, it's the CH340 clock error at 1 Mbaud.")


if __name__ == "__main__":
    main()
