"""
GPU-aware requirements installer for Osiris engines.

Auto-detects the host NVIDIA GPU and installs accelerator builds that match the
driver's CUDA version:
  * torch / torchvision / torchaudio  -> the matching PyTorch wheel channel
    (cu128 / cu126 / cu124 / cu121 / cu118), or the CPU channel.
  * onnxruntime                       -> `onnxruntime-gpu` + the CUDA-12 runtime
    libs it loads at runtime, or plain `onnxruntime` (CPU).
Everything else installs normally from PyPI.

No GPU / no driver  -> CPU wheels (clean degrade). GPU but missing CUDA runtime
in the venv -> the correct CUDA wheels are installed here.

Usage:  python osiris_install.py <requirements.txt>
Run with the engine's venv python, so installs land in that venv.
"""

import os
import re
import subprocess
import sys

# Set OSIRIS_INSTALL_DRYRUN=1 to print the pip commands without running them.
DRYRUN = os.environ.get("OSIRIS_INSTALL_DRYRUN") == "1"

PYTORCH = "https://download.pytorch.org/whl"
TORCH_PKGS = {"torch", "torchvision", "torchaudio"}
# CUDA-12 runtime libs that onnxruntime-gpu loads via onnxruntime.preload_dlls().
ONNX_CUDA_LIBS = [
    "nvidia-cudnn-cu12",
    "nvidia-cublas-cu12",
    "nvidia-cuda-runtime-cu12",
    "nvidia-cufft-cu12",
    "nvidia-curand-cu12",
    "nvidia-cuda-nvrtc-cu12",
]


def _run(args):
    print("[osiris-install] $ " + " ".join(args), flush=True)
    if DRYRUN:
        return
    subprocess.check_call(args)


def _pip(*pkgs, index_url=None):
    args = [sys.executable, "-m", "pip", "install", *pkgs]
    if index_url:
        args += ["--index-url", index_url]
    _run(args)


# nvidia-smi locations to try (the last two cover Flatpak/containerized hosts).
NVIDIA_SMI = ["nvidia-smi", "/run/host/usr/bin/nvidia-smi", "/usr/bin/nvidia-smi"]


def detect_cuda():
    """Return the driver's max CUDA as (major, minor), or None if no usable GPU."""
    for smi in NVIDIA_SMI:
        try:
            out = subprocess.run([smi], capture_output=True, text=True, timeout=15)
        except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
            continue
        if out.returncode != 0:
            continue
        m = re.search(r"CUDA Version:\s*([0-9]+)\.([0-9]+)", out.stdout)
        if m:
            return (int(m.group(1)), int(m.group(2)))
        return (12, 4)  # GPU present, version unreadable -> safe CUDA 12 baseline
    return None


def torch_index(cuda):
    """Pick a PyTorch wheel channel for the detected CUDA (driver is backward
    compatible, so we target the highest channel <= the driver's CUDA)."""
    if cuda is None:
        return f"{PYTORCH}/cpu"
    major, minor = cuda
    if major >= 13:
        tag = "cu128"            # newest broadly-published torch channel
    elif major == 12:
        tag = ("cu128" if minor >= 8 else
               "cu126" if minor >= 6 else
               "cu124" if minor >= 4 else "cu121")
    elif major == 11:
        tag = "cu118"
    else:
        tag = "cpu"
    return f"{PYTORCH}/{tag}"


def _base_name(spec):
    return re.split(r"[<>=!~;\[ ]", spec, maxsplit=1)[0].strip().lower()


def _version_spec(spec):
    m = re.search(r"(>=|==|~=|>)\s*[0-9][0-9A-Za-z.\-]*", spec)
    return m.group(0).replace(" ", "") if m else ""


def parse_requirements(path):
    reqs = []
    with open(path) as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if line and not line.startswith("-"):
                reqs.append(line)
    return reqs


def main():
    if len(sys.argv) < 2:
        print("usage: osiris_install.py <requirements.txt>", file=sys.stderr)
        sys.exit(1)

    cuda = detect_cuda()
    print(f"[osiris-install] GPU/CUDA detected: {cuda if cuda else 'none (CPU)'}", flush=True)

    reqs = parse_requirements(sys.argv[1])
    torch_specs, onnx_seen, onnx_ver, others = [], False, "", []
    for spec in reqs:
        name = _base_name(spec)
        if name in TORCH_PKGS:
            torch_specs.append(spec.split(";", 1)[0].strip())   # drop env markers
        elif name in ("onnxruntime", "onnxruntime-gpu"):
            onnx_seen = True
            onnx_ver = onnx_ver or _version_spec(spec)
        else:
            others.append(spec)

    # 1) Plain deps first (numpy, opencv, transformers, ...). transformers does
    #    not hard-depend on torch, so this won't drag in a CPU torch.
    if others:
        _pip(*others)

    # 2) torch family from the CUDA/CPU-matched channel.
    if torch_specs:
        _pip(*torch_specs, index_url=torch_index(cuda))

    # 3) onnxruntime: GPU build + CUDA libs, or CPU build.
    if onnx_seen:
        if cuda is not None:
            _pip(f"onnxruntime-gpu{onnx_ver}")
            _pip(*ONNX_CUDA_LIBS)
        else:
            _pip(f"onnxruntime{onnx_ver}")

    print("[osiris-install] done", flush=True)


if __name__ == "__main__":
    main()
