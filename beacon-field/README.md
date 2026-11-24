# Beacon field models

One JSON file per season describes the field geometry Beacon can anchor to: floor and tile grid,
perimeter walls, and the hive. The format is plan interface **I1**
(`docs/design/2026-11-17-beacon-orbslam-plan.md`). Next season you add a new file; no
code changes.

| File | Season | Source |
| --- | --- | --- |
| `biobuzz-2026.json` | BIOBUZZ 2026-27 | FIRST's official field CAD (STEP v26-27.2), checked against the Competition Manual |
| `derive/extract_biobuzz.py` | | Script that derived the JSON from the CAD (offline only, see below) |

Load and check a model from Python:

```
tools/.venv/bin/python -c "import sys; sys.path.insert(0,'tools/sim'); import fieldmodel; \
f = fieldmodel.load_field('beacon-field/biobuzz-2026.json'); print(f.validate() or 'OK')"
```

Tests: `tools/.venv/bin/python -m pytest tools/tests/test_fieldmodel.py -v`

## Where the BIOBUZZ numbers come from

- **Geometry:** *Field CAD (STEP, .ZIP)* v26-27.2, posted 2026-09-15 on FIRST's Playing Field
  Resources page (https://ftc-resources.firstinspires.org/ftc/archive/2027/field/field-cad-step).
  It is an Onshape export dated 2026-09-09. The zip's SHA-256 when downloaded on 2026-11-17 was
  `5e768b731f1ec8dcd14debba53225c43718877923c351ce08504305f68f7fe00`. Competition Manual 9.1 says
  the 3D CAD is the official representation of the field.
- **Pivot height and tilt:** the pivot height (43.95 in) and tilt (30°) come from Competition
  Manual Section 9 (TU06, 2026-10-22), Figures 9-8 and 9-10. The CAD agrees with both: placed with
  them, the two cells of each hive sit symmetrically about the pivot, to within 0.001 in.
- **Tolerance (`toleranceM` = 0.0254 m):** Competition Manual 9.1 gives CAD measurements a
  general tolerance of ±1 in. The Field Acceptance Checklist (26-27.3) gives tighter checks for a
  few items:
  - the hive frame feet: ±0.50 in;
  - the height of the upright cell's opening: 53.5 ± 0.50 in.
- **Cross-checks** (asserted in `tools/tests/test_fieldmodel.py`): the model reproduces the
  manual's values, each within its tolerance:
  - bottom of the hive: 30.6 in;
  - opening bottom and top: 53.5 in and 65.6 in;
  - frame: 49.46 in by 38.95 in;
  - hive centre to centre: 25.5 in.

### Differences from nominal values worth knowing

- The CAD puts the walls' inner faces at **±1.7929 m** (141.2 in square). The plan's example
  value, 1.8288 m, is the manual's "approximately 144 in".
- The CAD's tile seam pitch is **0.59694 m** (23.50 in), not the nominal 24 in. One seam crosses
  the centre of the field.
- The hive is **not static**. Each alliance's hive pivots and sits in one of two stable tilts
  (±30°). It flips during a match when enough game elements land in its upward-facing cell. The
  file gives the start-of-match state, from Event Field Setup Guide 11.1: the red hive's
  audience-side cell is up, and the blue hive's scoring-side cell is up. `pivots` says how to rotate
  each hive. The frame does not move.

## Frame and units

These are the FTC field coordinates from ftc-docs, *Field Coordinate System*:

- **Origin:** the centre of the field, on top of the tiles.
- **Units:** metres and radians.
- **Red wall:** the wall at the red alliance area. In BIOBUZZ that is the audience's left
  (Competition Manual 9.5). Seen from the red wall, +x points right and +y points away from it.
- **Axes for BIOBUZZ:** +x toward the audience, +y toward the blue alliance, +z up. Rotations
  are counter-clockwise.

The SDK's `AprilTagGameDatabase` does not yet hold real BIOBUZZ positions (see DECISIONS.md), so
it cannot serve as a cross-check.

## Keys (I1 plus optional additions)

Readers that only know I1 can ignore the optional keys.

| Key | Meaning |
| --- | --- |
| `schema`, `season`, `source`, `toleranceM` | I1. `source` names the drawing, revision and URL |
| `toleranceSource`, `fieldFrame` | optional: where the tolerance and frame come from |
| `floor.halfSizeM` | distance from the centre to each wall's inner face |
| `floor.tilePitchM`, `floor.gridOriginM` | seam spacing; a seam passes through `gridOriginM` |
| `floor.tiles` | optional: `[6, 6]` |
| `floor.tape` | optional: gaffer-tape rectangles on the tiles (`color`, `min`, `max` in x, y) |
| `walls[]` | I1: `name`, `p0`, `p1`. The walls run counter-clockwise, so the field is on the left of `p0` to `p1` |
| `walls[].heightM` | height of the top edge of the top rail above the tile surface |
| `walls[].railHeightM` | vertical size of the top rail (black aluminium). Its bottom edge is at `heightM - railHeightM` |
| `walls[].bottomRailTopM` | optional: top edge of the bottom rail. The wall-floor line is at z = 0 |
| `walls[].panelInsetM` | optional: how far the clear panels sit behind the rails' inner face |
| `walls[].posts` | vertical lines on the wall: its start corner and the joints between its panels. The am-0481 perimeter has no vertical posts. A joint is a 3 mm gap between two clear panels, with fasteners at both rails |
| `hive.nominalPose` | `[0, 0, 0]`: the hive structure is centred on the field |
| `hive.faces[]` | I1: `name`, `vertices` (planar, in order), `textured` |
| `hive.faces[].material` | optional: `clear` (see-through skins: never use their texture), `rib_red` / `rib_blue` (solid coloured plastic), `aluminum`, `sticker` (AprilTag cluster print, used only as texture: no tag decoding), `printed` (BIOBUZZ panel graphic) |
| `hive.faces[].part` | optional: `frame` (static), `hive_red`, `hive_blue` (each tips about its pivot) |
| `hive.edges[]`, `hive.edgeLabels[]` | I1 segments, plus a label per segment (for example `red_audience_opening` or `frame_leg0`) |
| `hive.pivots[]` | optional: `part`, `point`, `axis` (+y), `startTiltRad`, `tiltRad` (the two stable tilts). Positive tilt means the audience (+x) end is up. To tip a hive, move each point p of that part to `point + Ry(-(tilt - startTiltRad)) (p - point)` (see `Field.hive_at`) |
| `flowers[]` | optional: axis-aligned boxes (`min`, `max`) around the four FLOWERs on the walls. They block the view of the wall but are not modelled in detail |

**Faces with a hole.** Each coloured rib is a pentagonal ring. Its flat front and back faces are
written as one polygon: the outer ring, a slit to the inner ring, the inner ring backwards, then
back along the slit. Even-odd fill and ear-clipping triangulation handle this shape correctly.
Code that fills polygons any other way should skip `rib_*` faces or triangulate them with care.

**Textured faces** are only the opaque prints: four AprilTag stickers and two BIOBUZZ panel
stickers. The cell skins are clear plastic and the walls are clear polycarbonate, so any texture
seen through them belongs to the venue. Beacon must not anchor points to them.

## Regenerating the file

Converting the CAD needs OpenCASCADE (through `cascadio`), which is not a Beacon dependency. Use a
throwaway environment. The steps are in the docstring of `derive/extract_biobuzz.py`, which also
prints its consistency checks.
