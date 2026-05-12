#!/usr/bin/env python3
"""
Pack a Remora JSON config for upload to the RP2350 ETH firmware.

Produces a binary file with a 512-byte metadata header followed by the JSON.
The packed file is then uploaded with the standard tftp client in binary mode.

Usage:
    python3 pack_config.py config.txt          # writes config_packed.bin
    python3 pack_config.py config.txt out.bin  # custom output name

Then on the LinuxCNC machine (binary mode is required):
    tftp 10.10.10.10 -m binary -c put config_packed.bin
  or interactively:
    tftp 10.10.10.10
    binary
    put config_packed.bin
    quit
"""

import binascii
import struct
import sys

METADATA_LEN = 512  # must match firmware metadata_len


def pack(input_path, output_path):
    with open(input_path, "rb") as f:
        data = f.read()

    json_len = len(data)

    # Pad to next 4-byte boundary for CRC (matches firmware crc32 calculation)
    pad = (4 - (json_len % 4)) % 4
    padded = data + b"\x00" * pad
    length_words = len(padded) // 4

    # CRC32 — polynomial 0xEDB88320, matches crc32.h in the firmware
    crc = binascii.crc32(padded) & 0xFFFFFFFF

    # 512-byte metadata block: crc32, length (words), jsonLength (bytes), rest zeros
    meta = struct.pack("<III", crc, length_words, json_len)
    meta += b"\x00" * (METADATA_LEN - len(meta))

    with open(output_path, "wb") as f:
        f.write(meta)
        f.write(data)

    print(f"JSON        : {json_len} bytes, padded to {length_words} words")
    print(f"CRC32       : 0x{crc:08X}")
    print(f"Output      : {output_path}  ({METADATA_LEN + json_len} bytes)")
    print()
    print("Upload (binary mode required):")
    print(f"  tftp 10.10.10.10 -m binary -c put {output_path}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <config.txt> [output.bin]")
        sys.exit(1)
    inp = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) >= 3 else "config_packed.bin"
    pack(inp, out)
