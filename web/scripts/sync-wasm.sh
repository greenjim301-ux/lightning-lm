#!/usr/bin/env bash
# Copies the Emscripten build output (src/ui/wasm/CMakeLists.txt's lightning_ui_wasm_client
# target) from ../../build-wasm into public/wasm/, where LightningUiViewer.tsx dynamically
# imports it from. Run this after every `emmake make` in build-wasm/. See ../README.md.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
web_dir="$(dirname "$script_dir")"
build_wasm_dir="$web_dir/../build-wasm"

if [[ ! -f "$build_wasm_dir/lightning_ui_wasm_client.mjs" ]]; then
    echo "error: $build_wasm_dir/lightning_ui_wasm_client.mjs not found." >&2
    echo "Build it first — see web/README.md for the emcmake/emmake steps." >&2
    exit 1
fi

mkdir -p "$web_dir/public/wasm"
cp "$build_wasm_dir/lightning_ui_wasm_client.mjs" "$build_wasm_dir/lightning_ui_wasm_client.wasm" "$web_dir/public/wasm/"
echo "Copied lightning_ui_wasm_client.{mjs,wasm} into $web_dir/public/wasm/"
