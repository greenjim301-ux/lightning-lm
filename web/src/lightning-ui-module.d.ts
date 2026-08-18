// Type shape for the MODULARIZE=1/EXPORT_ES6=1 Emscripten glue emitted by
// src/ui/wasm/CMakeLists.txt (lightning_ui_wasm_client.mjs, copied into public/wasm/
// by scripts/sync-wasm.sh). The module itself isn't type-checked — this just describes
// enough of the default Emscripten factory export shape for the dynamic import below.
export interface LightningUiModule {
  // main() runs automatically on instantiation and drives everything through the
  // canvas + the UiWireServer websocket connection opened in wasm_ui_client.cc.
  // These two are the only calls the host page makes into the module: embind bindings
  // (EMSCRIPTEN_BINDINGS in wasm_ui_client.cc) that write straight into the same
  // pangolin::Var<bool> state Pangolin's own (non-rendering, under this ES3/wasm build)
  // on-canvas menu panel would — see LightningUiViewer.tsx for why.
  setFollow(follow: boolean): void
  resetView(): void
}

export type CreateLightningUiModule = (overrides?: Record<string, unknown>) => Promise<LightningUiModule>;
