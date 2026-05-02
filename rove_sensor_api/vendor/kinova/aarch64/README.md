# Kinova legacy SDK — aarch64 binaries

These four `.so` files are recompiled aarch64 builds of the legacy Kinova SDK
(pre-Kortex, ~v5.03), used to drive the Capra Roboguard's Kinova Gen2 custom
6DOF spherical arm.

**Source:** `clubcapra/ovis` repo at
`kinova_driver/lib/aarch64-linux-gnu/`
(commit current as of 2026-04-30).

Kinova never released aarch64 builds for this SDK. The Capra robotics club
recompiled them; we vendor the same artifacts here so this repo can build and
deploy without depending on an external git submodule.

**Internal dlopen requirement:** `EthCommandLayerUbuntu.so` internally
`dlopen`s `Kinova.API.EthCommLayerUbuntu.so` (the SONAME of
`EthCommLayerUbuntu.so`). The driver loads the comm layer first with
`RTLD_NOW | RTLD_GLOBAL` so the SONAME registers, then loads the command
layer — at which point the internal dlopen finds the already-loaded library
by SONAME match and reuses it. No symlinks or filename rewrites needed.

**Header reference:** the C ABI we bind against is documented in
`Kinovarobotics/kinova_sdk_recompiled` at `raspberry pi 3/_HEADERS/`.
Headers there are armhf-targeted but the C ABI is identical on aarch64.

| File | Purpose |
| ---- | ------- |
| `EthCommandLayerUbuntu.so`  | High-level `Ethernet_*` functions we call |
| `EthCommLayerUbuntu.so`     | UDP transport, loaded by command layer internally |
| `USBCommandLayerUbuntu.so`  | USB equivalent — unused on this platform |
| `USBCommLayerUbuntu.so`     | USB transport — unused on this platform |
