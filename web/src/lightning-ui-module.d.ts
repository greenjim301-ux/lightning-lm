// Type shape for the MODULARIZE=1/EXPORT_ES6=1 Emscripten glue emitted by
// src/ui/wasm/CMakeLists.txt (lightning_ui_wasm_client.mjs, copied into public/wasm/
// by scripts/sync-wasm.sh). The module itself isn't type-checked — this just describes
// enough of the default Emscripten factory export shape for the dynamic import below.
export interface LightningUiModule {
  // Emscripten's default MODULARIZE export doesn't expose anything we call directly:
  // main() runs automatically on instantiation and drives everything through the
  // canvas + the UiWireServer websocket connection opened in wasm_ui_client.cc.
  [key: string]: unknown;
}

export type CreateLightningUiModule = (overrides?: Record<string, unknown>) => Promise<LightningUiModule>;
