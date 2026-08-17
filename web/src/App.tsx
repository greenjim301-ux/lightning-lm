import WebViewer from "@rerun-io/web-viewer-react";

// ui::PangolinWindow::Init() (src/ui/pangolin_window.cc) starts a gRPC server on :9876.
// Override VITE_RERUN_URL (in a .env.local) if the SLAM process runs on a different host.
const RERUN_URL = import.meta.env.VITE_RERUN_URL ?? "rerun+http://localhost:9876/proxy";

// Two-pane layout (world-frame overview + world/follow_anchor-relative chase cam), generated via
// scripts/gen_blueprint.py -- regenerate after changing that script or the entity paths it references.
// Must be an absolute URL: the viewer resolves this itself rather than through the page's fetch(),
// so a root-relative path doesn't get resolved against the page origin the way a normal asset would.
const BLUEPRINT_URL = new URL("/blueprint.rbl", window.location.href).toString();

export default function App() {
  return (
    <WebViewer
      width="100%"
      height="100%"
      rrd={[RERUN_URL, BLUEPRINT_URL]}
      hide_welcome_screen
      theme="dark"
    />
  );
}
