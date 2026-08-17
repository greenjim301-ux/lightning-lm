import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import wasm from "vite-plugin-wasm";
import topLevelAwait from "vite-plugin-top-level-await";

// @rerun-io/web-viewer-react ships a WASM module loaded via top-level await,
// which Vite doesn't support out of the box -- these two plugins add it.
export default defineConfig({
  plugins: [react(), wasm(), topLevelAwait()],
  server: {
    port: 5173,
  },
  // Vite's esbuild-based dep pre-bundler doesn't understand vite-plugin-wasm's import syntax and
  // ends up serving the .wasm file with the wrong MIME type -- exclude so it's handled by the plugin instead.
  optimizeDeps: {
    exclude: ["@rerun-io/web-viewer-react", "@rerun-io/web-viewer"],
  },
});
