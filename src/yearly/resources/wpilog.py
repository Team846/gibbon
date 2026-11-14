#!/usr/bin/env python3
import csv
import struct
import sys

SCALARS = {"double": "<d", "int64": "<q", "float": "<f"}
ARRAYS = {"double[]": "d", "int64[]": "q", "float[]": "f"}


def read(path):
    data = open(path, "rb").read()
    if data[:6] != b"WPILOG":
        sys.exit(f"{path}: not a .wpilog file")
    pos = 12 + struct.unpack_from("<I", data, 8)[0]
    entries = {}
    while pos < len(data):
        header = data[pos]
        pos += 1
        fields = []
        for size in ((header & 3) + 1, ((header >> 2) & 3) + 1, ((header >> 4) & 7) + 1):
            fields.append(int.from_bytes(data[pos:pos + size], "little"))
            pos += size
        entry, length, timestamp = fields
        if pos + length > len(data):
            break
        payload = data[pos:pos + length]
        pos += length
        if entry == 0:
            if payload[0] == 0:
                entry_id, name_len = struct.unpack_from("<II", payload, 1)
                name = payload[9:9 + name_len].decode()
                type_len = struct.unpack_from("<I", payload, 9 + name_len)[0]
                start = 13 + name_len
                entries[entry_id] = (name, payload[start:start + type_len].decode())
            continue
        if entry not in entries:
            continue
        name, kind = entries[entry]
        yield timestamp / 1e6, name, kind, decode(kind, payload)


def decode(kind, payload):
    if kind in SCALARS:
        return struct.unpack(SCALARS[kind], payload)[0]
    if kind == "boolean":
        return payload[0] != 0
    if kind in ("string", "json"):
        return payload.decode(errors="replace")
    if kind in ARRAYS:
        return list(struct.unpack(f"<{len(payload) // struct.calcsize(ARRAYS[kind])}{ARRAYS[kind]}", payload))
    return f"<{len(payload)} bytes>"


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: wpilog.py LOG.wpilog [KEY_SUBSTRING ...]\n"
                 "  no keys: list every entry with its type and sample count\n"
                 "  keys:    print time_s,name,value CSV for entries containing any key")
    path, keys = sys.argv[1], sys.argv[2:]
    if not keys:
        counts = {}
        for _, name, kind, _ in read(path):
            counts[(name, kind)] = counts.get((name, kind), 0) + 1
        for (name, kind), count in sorted(counts.items()):
            print(f"{count:8d}  {kind:10s}  {name}")
        return
    out = csv.writer(sys.stdout)
    out.writerow(["time_s", "name", "value"])
    for time_s, name, _, value in read(path):
        if any(key in name for key in keys):
            out.writerow([f"{time_s:.6f}", name, value])


if __name__ == "__main__":
    main()
