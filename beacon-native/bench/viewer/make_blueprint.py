"""Writes beacon.rbl, the Rerun blueprint (layout + default visibility) for the Beacon viewer.

Matches the team's reference videos (RViz): display list on the left with on/off toggles,
camera image bottom-left, 3-D orbit view on the right with the grid. Default visibility per
the plan's Task C6 table.

Usage (pinned rerun-sdk 0.41.0, installed by scripts/setup_viewer.sh):
    beacon-native/third_party/venv/bin/python beacon-native/bench/viewer/make_blueprint.py OUT.rbl
"""

import sys

import rerun as rr
import rerun.blueprint as rrb

APP_ID = "beacon"

# Entities and their default visibility (plan Task C6 table).
ENTITIES_3D = {
    "beacon/world/grid": True,
    "beacon/world/tf": False,
    "beacon/points": True,
    "beacon/backend_pose_imu_viz": True,
    "beacon/loop_closures": True,
    "beacon/pose_imu": False,
    "beacon/kfs": True,
    "beacon/frontend_map": False,
    "beacon/pose_graph": True,
    "beacon/lc_database_map": False,
    "beacon/backend_points": True,
}


def blueprint() -> rrb.Blueprint:
    overrides = {
        path: rrb.EntityBehavior(visible=False) for path, on in ENTITIES_3D.items() if not on
    }
    view3d = rrb.Spatial3DView(
        name="3-D",
        origin="/",
        contents=[f"+ {p}/**" for p in ENTITIES_3D] + ["+ beacon/world/camera/**"],
        overrides=overrides,
        background=[166, 166, 166],  # RViz-like grey
        line_grid=False,  # the grid is the toggleable entity beacon/world/grid
    )
    image = rrb.Spatial2DView(
        name="beacon/image/cam0",
        origin="beacon/image/cam0",
        contents=["+ beacon/image/cam0/**"],
    )
    status = rrb.TextDocumentView(name="Status", origin="beacon/status")
    left = rrb.Vertical(status, image, row_shares=[1, 3])
    return rrb.Blueprint(
        rrb.Horizontal(left, view3d, column_shares=[1, 2.4]),
        rrb.BlueprintPanel(state="expanded"),
        rrb.SelectionPanel(state="collapsed"),
        rrb.TimePanel(state="collapsed"),
        auto_views=False,
    )


def main() -> int:
    out = sys.argv[1] if len(sys.argv) > 1 else "beacon.rbl"
    rr.init(APP_ID, spawn=False)
    blueprint().save(APP_ID, out)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
