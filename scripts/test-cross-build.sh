#!/usr/bin/env bash
set -euo pipefail

TARGETS=(
    x86_64-unknown-linux-gnu
    aarch64-unknown-linux-gnu
    aarch64-apple-darwin
    x86_64-pc-windows-gnu
)

RED='\033[0;31m'
GREEN='\033[0;32m'
BOLD='\033[1m'
RESET='\033[0m'

pass() { echo -e "${GREEN}${BOLD}PASS${RESET} $1"; }
fail() { echo -e "${RED}${BOLD}FAIL${RESET} $1"; exit 1; }
step() { echo -e "\n${BOLD}--- $1 ---${RESET}"; }

# Ensure we're at the repo root
cd "$(git rev-parse --show-toplevel)"

# Install Rust targets if missing
step "Checking Rust targets"
rustup target add "${TARGETS[@]}"

# --- Rust crate builds ---

step "Cargo build (x86_64-unknown-linux-gnu)"
if cargo build --release --target x86_64-unknown-linux-gnu; then
    pass "cargo build --target x86_64-unknown-linux-gnu"
else
    fail "cargo build --target x86_64-unknown-linux-gnu"
fi

step "Cargo build (aarch64-unknown-linux-gnu)"
if CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc \
   cargo build --release --target aarch64-unknown-linux-gnu; then
    pass "cargo build --target aarch64-unknown-linux-gnu"
else
    fail "cargo build --target aarch64-unknown-linux-gnu"
fi

step "Cargo build (aarch64-apple-darwin)"
if cargo zigbuild --release --target aarch64-apple-darwin; then
    pass "cargo zigbuild --target aarch64-apple-darwin"
else
    fail "cargo zigbuild --target aarch64-apple-darwin"
fi

step "Cargo build (x86_64-pc-windows-gnu)"
if CARGO_TARGET_X86_64_PC_WINDOWS_GNU_LINKER=x86_64-w64-mingw32-gcc \
   cargo build --release --target x86_64-pc-windows-gnu; then
    pass "cargo build --target x86_64-pc-windows-gnu"
else
    fail "cargo build --target x86_64-pc-windows-gnu"
fi

# --- Python wheel builds ---

step "Maturin build (x86_64-unknown-linux-gnu)"
if maturin build --release --target x86_64-unknown-linux-gnu --out dist; then
    pass "maturin build --target x86_64-unknown-linux-gnu"
else
    fail "maturin build --target x86_64-unknown-linux-gnu"
fi

step "Maturin build (aarch64-unknown-linux-gnu)"
if CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc \
   maturin build --release --target aarch64-unknown-linux-gnu --out dist; then
    pass "maturin build --target aarch64-unknown-linux-gnu"
else
    fail "maturin build --target aarch64-unknown-linux-gnu"
fi

step "Maturin build (aarch64-apple-darwin)"
if maturin build --release --target aarch64-apple-darwin --zig --out dist; then
    pass "maturin build --target aarch64-apple-darwin"
else
    fail "maturin build --target aarch64-apple-darwin"
fi

step "Maturin build (x86_64-pc-windows-gnu)"
if CARGO_TARGET_X86_64_PC_WINDOWS_GNU_LINKER=x86_64-w64-mingw32-gcc \
   maturin build --release --target x86_64-pc-windows-gnu --out dist; then
    pass "maturin build --target x86_64-pc-windows-gnu"
else
    fail "maturin build --target x86_64-pc-windows-gnu"
fi

# --- Summary ---

step "Built artifacts"
ls -lh dist/*.whl

echo -e "\n${GREEN}${BOLD}All builds passed.${RESET}"
