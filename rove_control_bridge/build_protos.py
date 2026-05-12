"""Compile the .proto files into Python modules.

Run once before starting the bridge, and again whenever .proto files change:

    python build_protos.py

Output lands alongside the .proto files as ``<Name>_pb2.py``.
"""
from __future__ import annotations

import sys
from pathlib import Path

from grpc_tools import protoc

PKG_ROOT = Path(__file__).resolve().parent
PROTO_DIR = PKG_ROOT / "proto" / "core"


def main() -> int:
    proto_files = sorted(PROTO_DIR.glob("*.proto"))
    if not proto_files:
        print(f"No .proto files found in {PROTO_DIR}", file=sys.stderr)
        return 1

    args = [
        "protoc",
        f"--proto_path={PKG_ROOT}",
        f"--python_out={PKG_ROOT}",
    ]
    args += [str(p.relative_to(PKG_ROOT)) for p in proto_files]

    print("Running:", " ".join(args))
    rc = protoc.main(args)
    if rc != 0:
        print("protoc failed", file=sys.stderr)
        return rc

    print("Generated:")
    for pb2 in sorted(PROTO_DIR.glob("*_pb2.py")):
        print(f"  {pb2.relative_to(PKG_ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
