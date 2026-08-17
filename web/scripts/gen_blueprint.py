#!/usr/bin/env python3
"""Regenerate web/public/blueprint.rbl.

rerun_cpp (unlike the Python SDK) has no high-level blueprint builder, so the
blueprint is authored here with `rerun-sdk` and shipped as a static asset the
frontend loads alongside the live data stream (see web/src/App.tsx).

Requires: pip install rerun-sdk==0.36.0  (must match LIGHTNING_RERUN_VERSION
in cmake/packages.cmake and the @rerun-io/web-viewer-react version in
web/package.json -- a blueprint from a mismatched SDK version may not load.)
"""

import pathlib

import rerun as rr
import rerun.blueprint as rrb

OUT_PATH = pathlib.Path(__file__).parent.parent / "public" / "blueprint.rbl"

blueprint = rrb.Blueprint(
    rrb.Horizontal(
        # World-frame overview: static map + full trajectories.
        rrb.Spatial3DView(origin="/", name="World overview"),
        # Chase cam: everything rendered relative to world/follow_anchor, a translate-only
        # (Z=0, identity rotation) entity logged in PangolinWindow::LogFrontendPose -- mirrors
        # the old Pangolin UI's OpenGlRenderState::Follow() behavior (translate, don't rotate
        # with heading).
        rrb.Spatial3DView(origin="world/follow_anchor", name="Chase cam"),
        column_shares=[1, 1],
    ),
    auto_layout=False,
    auto_views=False,
)

blueprint.save("lightning_lm", str(OUT_PATH))
print(f"wrote {OUT_PATH}")
