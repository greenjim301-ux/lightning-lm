import WebViewer from "@rerun-io/web-viewer-react";

// ui::PangolinWindow::Init() (src/ui/pangolin_window.cc) starts a gRPC server on :9876.
// Override VITE_RERUN_URL (in a .env.local) if the SLAM process runs on a different host.
const RERUN_URL = import.meta.env.VITE_RERUN_URL ?? "rerun+http://localhost:9876/proxy";

export default function App() {
  return (
    <WebViewer
      width="100%"
      height="100%"
      rrd={RERUN_URL}
      hide_welcome_screen
      theme="dark"
    />
  );
}
