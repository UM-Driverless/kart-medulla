#!/usr/bin/env python3
"""Read Hall diagnostics from binary health frames; never send motor commands.

Usage: python3 tools/monitor_halls.py /dev/ttyACM0
Requires pyserial. Stop other users of the serial port before opening it.
Opening a USB serial port can reset the board: isolate actuator power first.
"""

import argparse
import struct


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (0x07 if crc & 0x80 else 0)) & 0xFF
    return crc


def frames(buffer):
    """Consume complete, CRC-checked frames; retain a fragmented tail."""
    while buffer:
        if buffer[0] != 0xAA:
            del buffer[0]
            continue
        if len(buffer) < 3:
            return
        size = buffer[1]
        if size > 248 or size % 4:
            del buffer[0]
            continue
        if len(buffer) < size + 4:
            return
        if crc8(buffer[1:size + 3]) != buffer[size + 3]:
            del buffer[0]
            continue
        kind = buffer[2]
        payload = struct.unpack(f">{size // 4}i", buffer[3:size + 3])
        del buffer[:size + 4]
        yield kind, payload


def describe(payload):
    if len(payload) < 15:
        return "Health received; firmware has no Hall diagnostics."
    status, bits, h1, h2, h3, age, interval, multi = payload[7:15]
    if status:
        return f"Hall capture unavailable: initialization error {status:#x}"
    if not 0 <= bits <= 7 or age < -1 or interval < -1:
        return "Invalid Hall telemetry fields"
    states = "/".join(str((bits >> i) & 1) for i in range(3))
    counts = "/".join(str(n & 0xFFFFFFFF) for n in (h1, h2, h3))
    age_text = "no edges yet" if age == -1 else f"last edge {age} ms ago"
    interval_text = "unknown" if interval == -1 else f"{interval} us"
    return (f"H1/H2/H3={states}  edges={counts}  {age_text}  "
            f"last interval={interval_text}  multi-bit changes={multi & 0xFFFFFFFF}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="explicit serial port; no automatic board selection")
    args = parser.parse_args()
    import serial

    # Set control lines before opening to avoid requesting a board reset.
    # Some OS/drivers can still pulse them at open, hence the power warning.
    port = serial.Serial(port=None, baudrate=115200, timeout=1, exclusive=True)
    port.dtr = False
    port.rts = False
    port.port = args.port
    with port:
        buffer = bytearray()
        while True:
            data = port.read(max(1, port.in_waiting))
            if not data:
                print("No serial data received in the last second.", flush=True)
                continue
            buffer.extend(data)
            for kind, payload in frames(buffer):
                if kind == 0x0B:
                    print(describe(payload), flush=True)


if __name__ == "__main__":
    main()
