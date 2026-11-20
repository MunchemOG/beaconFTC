# Beacon synthetic bench: simulator and evaluation

These tools produce synthetic datasets in the plan's I2 layout from the BIOBUZZ field model
(`beacon-field/biobuzz-2026.json`), then score pose estimates against the truth. They use
`tools/.venv`. Packages: `tools/requirements.txt` plus `tools/sim/requirements.txt`.

```
tools/.venv/bin/python tools/sim/make_dataset.py --traj match --seed 1 --out /tmp/beacon-runs/match1
tools/.venv/bin/python tools/eval/eval_run.py --run /tmp/beacon-runs/match1 --est fused.csv --gate stage1
tools/.venv/bin/python -m pytest tools/tests/test_sim.py tools/tests/test_eval.py -q
```

Run `--help` on either script for every option. Options include camera height, pitch, mount and
FOV; distortion (`--k1`, `--k2`); rolling shutter (`--rs-readout-ms`); motion blur
(`--exposure-ms`, `--blur-samples`); noise; JPEG (`--jpeg-quality`); start-pose error; hive
tipping; and the venue.

## Odometry profiles (`--odom-profile`)

| Profile | Scale error per axis | Heading random walk | Collisions | Slip per collision | Heading jump | Start error |
| --- | --- | --- | --- | --- | --- | --- |
| `nominal` (default) | ±0.5 % | σ 0.5° at 60 s | `match`: about 1 per 50 s | 1-3 cm | none | 0 |
| `hard` | ±1.5 % | σ 2° at 60 s | 4-8 per 150 s | 2-5 cm | up to ±2° | 2 cm, 1° |

`--start-error-cm` and `--start-error-deg` override the profile's start error.

**Run accuracy gates on `hard` runs.** Pure odometry passes Stage 1 on 3 of 5 `nominal` match
seeds, so the nominal gate barely tests vision. The `hard` start error (2 cm, 1°) is well under the
5 cm limit; a run fails only once odometry drifts past 5 cm. On match seeds 1-5 that happens at
t = 5.0-44.4 s, and the error reaches 0.16-0.33 m. To get that
pure-Pinpoint baseline, `tools/eval/odom_only.py --run RUN --out odom_only.csv` aligns the
odometry once through `start.csv` and never corrects it. To change the start error of existing
runs without re-rendering, use `tools/sim/rewrite_start.py`.

Ready-made `hard` runs, generated 2026-11-17 and gitignored, are in `tools/sim/runs/`:
- `hard-match-s1` to `hard-match-s5`: `match`, 30 fps;
- `hard-session-s1`: 20 min, 10 fps.

`tools/sim/runs/nominal-match-s1` … `s8` are the same eight matches (`hard-match-s1` … `s5` and
`heldout-match-s6` … `s8`) with `nominal` odometry. Each one's `frames/` is a symlink to the hard
run's frames. They were made with `tools/sim/make_odom_variant.py --from RUN --odom-profile
nominal --out DIR`, which uses `make_dataset.py --no-render` and checks that every frame-related
file is byte-identical to the source run.

## Trajectories (`--traj`)

| Kind | Default length | What it exercises |
| --- | --- | --- |
| `match` | 150 s | 3 s still (INIT), 30 s auto at up to 1.5 m/s, 8 s still, teleop at up to 2.7 m/s and 7 rad/s, a few collisions (odometry slip) |
| `session` | 1200 s | teleop-style driving with pauses: drift over a practice session |
| `spin` | 60 s | spins at up to 7 rad/s with short drives between them |
| `blind` | 45 s | 10 s parked facing a corner, where a partner robot hides the walls (`meta.json` `blindWindowS`) |
| `clutter` | 150 s | match-style driving, plus 3 other robots driving around and 30 game pieces on the floor; contacts are collisions |

The hives tip at random during `match`, `session` and `clutter` (`--hive-tips`). The true tilts
are in `hive.csv`.

## Files written (I2, plus two simulator-only extras)

| File | Contents |
| --- | --- |
| `camera.json` | intrinsics, `dist` [k1,k2,p1,p2,k3], `mount` (T_robot_cam, BeaconConfig convention), `rollingShutterReadoutS` |
| `field.json` | byte-for-byte copy of `--field` |
| `frames.csv` + `frames/<idx:06d>.png` | 8-bit grey frames. `t_ns` is the exposure time of the **centre row** at the **middle of the exposure**. Row r is exposed at `t + (r - (H-1)/2) * readout / H` |
| `odom.csv` | Pinpoint-like odometry in the Pinpoint's own world frame, which starts at (0, 0, 0). 200 Hz. ±0.5 % scale error per axis, heading random walk with σ = 0.5° at 60 s, 1-3 cm slips at collisions |
| `start.csv` | the start pose as the team typed it: truth plus `--start-error-cm` / `--start-error-deg` |
| `truth.csv` | true robot pose in the field frame, at the same times as `odom.csv` |
| `hive.csv` | simulator only: true hive tilts (rad), with the same sign as the field model's `pivots` |
| `meta.json` | simulator only: all arguments, odometry scale errors, slips, collision times, hive flip times, max speed and turn rate, render time |

All CSV headings are wrapped to (-π, π]. `t_ns` starts at 10 s, on an arbitrary
`elapsedRealtimeNanos` time base.

## How the images are made

The images are ray-cast: each pixel's ray is intersected exactly with every textured planar
polygon. Projection and rendering share one camera model (`camera.py`), so a 3-D point lands
exactly where `Camera.project` puts it. `test_rendered_corners_land_on_projected_points` checks
this through OpenCV's corner finder: no bias above 0.03 px, RMS scatter under 0.1 px.

**The scene:**
- foam tiles with zig-zag seams on the model's grid, and the tape zones;
- black wall rails, and clear polycarbonate panels with the venue visible through them;
- the hive's clear skins, coloured ribs, aluminium frame, AprilTag-cluster stickers (texture
  only) and the BIOBUZZ panel;
- FLOWERs, drawn as see-through boxes;
- the venue: floor, bleachers with people, banners, a ceiling with lights, and people beside the
  field;
- clutter: other robots and game pieces.

**Not modelled:** lighting changes, specular reflections on the clear panels, lens vignetting and
exposure control.
