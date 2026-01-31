#!/bin/bash
set -e
cd "$(dirname "$0")"
echo "Building ARM Wizard (GTK GUI)..."
source "$HOME/.cargo/env" 2>/dev/null || true
cargo build --release
mkdir -p dist
cp target/release/arm_wizard dist/
echo ""
echo "Build complete."
echo "Executable: dist/arm_wizard ($(du -h dist/arm_wizard | cut -f1))"
echo "Run: dist/arm_wizard"
