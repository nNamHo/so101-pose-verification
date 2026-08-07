#!/usr/bin/env python3
# bus_check.py — standalone Feetech STS3215 bus diagnostic. NO ROS REQUIRED.
#
# Run this BEFORE any ROS bringup, to prove the lowest layer works:
#   does the USB adapter exist, can we open it, and do the servos answer?
#
# IMPORTANT: a serial port can only be opened by ONE process. Stop any ROS
# hardware bringup before running this, or you'll get "device busy".
#
# Usage:
#   python3 bus_check.py                    # auto-detect port, scan IDs 1-10
#   python3 bus_check.py --port /dev/ttyUSB0
#   python3 bus_check.py --max-id 20 --baud 1000000
#   python3 bus_check.py --read-pos         # also read present position

import argparse
import glob
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing. Install with:  pip install pyserial --break-system-packages\n"
             "                            or:  sudo apt install python3-serial")


# ---------------- Feetech STS/SMS protocol (Dynamixel-like) ----------------
# Packet: 0xFF 0xFF <id> <length> <instruction> [params...] <checksum>
#   length   = number of bytes after it (instruction + params + checksum)
#   checksum = (~(id + length + instruction + params)) & 0xFF
INST_PING = 0x01
INST_READ = 0x02

# STS3215 memory table. Present_Position is 2 bytes, LITTLE-endian on the
# STS/SMS series (the older SCS series is big-endian — a classic mix-up).
ADDR_PRESENT_POSITION = 56
STEPS_PER_REV = 4096


def checksum(values):
    return (~sum(values)) & 0xFF


def ping_packet(servo_id):
    length, inst = 2, INST_PING
    return bytes([0xFF, 0xFF, servo_id, length, inst,
                  checksum([servo_id, length, inst])])


def read_packet(servo_id, addr, size):
    length, inst = 4, INST_READ
    return bytes([0xFF, 0xFF, servo_id, length, inst, addr, size,
                  checksum([servo_id, length, inst, addr, size])])


def find_response(buf, servo_id, sent=None):
    # Many half-duplex adapters echo the transmitted bytes back on RX. If the
    # buffer starts with exactly what we sent, drop it first — otherwise we'd
    # match our OWN packet and report every ID as "responding" (false positive).
    if sent and buf[:len(sent)] == sent:
        buf = buf[len(sent):]
    for i in range(len(buf) - 3):
        if buf[i] == 0xFF and buf[i + 1] == 0xFF and buf[i + 2] == servo_id:
            length = buf[i + 3]
            end = i + 4 + length
            if end <= len(buf):
                return buf[i:end]
    return None


def txrx(ser, packet, servo_id, wait=0.02):
    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()
    time.sleep(wait)
    return find_response(ser.read(ser.in_waiting or 1), servo_id, sent=packet)


def autodetect_ports():
    return sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=1000000)  # SO-101 default
    ap.add_argument("--max-id", type=int, default=10)
    ap.add_argument("--read-pos", action="store_true")
    args = ap.parse_args()

    # --- step 1: does a serial device exist at all? ---
    ports = autodetect_ports()
    print(f"[1] serial devices found: {ports if ports else 'NONE'}")
    if not ports and not args.port:
        sys.exit("    -> No /dev/ttyUSB* or /dev/ttyACM*. The adapter is not reaching\n"
                 "       Ubuntu. Check VirtualBox: Devices > USB > (tick your adapter),\n"
                 "       and that the Extension Pack is installed on the Windows host.")
    port = args.port or ports[0]

    # --- step 2: can we open it? (permissions live here) ---
    print(f"[2] opening {port} at {args.baud} baud...")
    try:
        ser = serial.Serial(port, args.baud, timeout=0.05)
    except serial.SerialException as e:
        sys.exit(f"    -> FAILED: {e}\n"
                 "       If 'permission denied': sudo usermod -aG dialout $USER,\n"
                 "       then LOG OUT and back in.\n"
                 "       If 'device busy': a ROS bringup already owns the port.")
    print("    -> port opened OK")

    # --- step 3: do servos answer? this is the real proof ---
    print(f"[3] pinging IDs 1..{args.max_id} (this is the actual bus test)")
    found = []
    for sid in range(1, args.max_id + 1):
        if txrx(ser, ping_packet(sid), sid):
            found.append(sid)
            print(f"    ID {sid:>3}: RESPONDING")
    if not found:
        ser.close()
        sys.exit("    -> No servo replied. The port opened but the bus is silent.\n"
                 "       Most likely: servos unpowered (they need their own supply,\n"
                 "       USB alone is not enough), wrong baud rate, or TX/RX swapped.")

    print(f"\n    {len(found)} servo(s) responding: {found}")
    if found == list(range(1, 7)):
        print("    -> IDs 1-6 present: matches the expected SO-101 layout.")
    else:
        print("    -> NOTE: expected IDs 1-6 for the SO-101. Check for ID collisions\n"
              "       (unconfigured servos all default to ID 1) or a break in the chain.")

    # --- step 4 (optional): read live positions ---
    if args.read_pos:
        print("\n[4] present positions:")
        for sid in found:
            resp = txrx(ser, read_packet(sid, ADDR_PRESENT_POSITION, 2), sid)
            if resp and len(resp) >= 7:
                raw = resp[5] | (resp[6] << 8)          # little-endian
                deg = raw * 360.0 / STEPS_PER_REV
                print(f"    ID {sid:>3}: {raw:>5} steps  ({deg:7.2f} deg)")
            else:
                print(f"    ID {sid:>3}: read failed (verify register address 56 "
                      "against your servo's datasheet)")

    ser.close()
    print("\nBus check complete.")


if __name__ == "__main__":
    main()
