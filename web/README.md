# lightning-lm web UI

React + Vite + TypeScript host page for `src/ui/wasm_ui_client.cc` (see
`src/ui/wasm/CMakeLists.txt`). It doesn't reimplement any rendering itself — it just
provides a `<canvas id="canvas">` (the DOM id Pangolin's Emscripten window backend looks
for) and dynamically `import()`s the built Emscripten module, which then owns the canvas,
opens a WebSocket to a `UiWireServer` (`src/ui/ui_wire_server.h`), and renders everything.

## Build the wasm module and sync it in

```bash
source ~/tools/emsdk/emsdk_env.sh
cd ../  # repo root
mkdir -p build-wasm && cd build-wasm
emcmake cmake ../src/ui/wasm \
    -DPangolin_DIR=~/tools/pangolin-em-src/build-em \
    -DEigen3_DIR=/usr/share/eigen3/cmake
emmake make
cd ../web
npm run sync-wasm   # copies lightning_ui_wasm_client.{mjs,wasm} into public/wasm/
```

`public/wasm/*.mjs`/`*.wasm` are gitignored (build output, same as `build-wasm/` itself) —
re-run `npm run sync-wasm` after every wasm rebuild.

## Run a UiWireServer to point it at

The wasm client needs a live `UiWireServer` to connect to — it renders nothing on its own.
Any process that constructs `lightning::ui::PangolinWindow` opens one (that's the whole
point of the branch: `slam.cc`/`localization.cpp` do this already). For local dev without a
full SLAM pipeline running, `src/app/test_ui.cc` drives `PangolinWindow` with synthetic
data and is enough to see the UI move.

## Dev server

```bash
npm install
npm run dev
```

Open the printed URL. By default the page connects to `ws://localhost:9877`; override with
either the `?ws=` query param (e.g. `?ws=ws://192.168.1.50:9877` for a real robot) or the
"UiWireServer URL" field in the toolbar, which reloads the page with the new `?ws=` value —
`wasm_ui_client.cc`'s `main()` only reads the server URL once at startup, so changing it
means reloading rather than hot-swapping the connection.
