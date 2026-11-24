# 06 — Testing protocol and acceptance numbers

Run the full table on every tier and after any change to `astro.json` or calibration. Log every
run as a CSV row: `date, tier, test, result, notes, phone_temp_c, loop_hz`. Publish measured
numbers; never the targets.

| # | Test | Procedure | Pass |
| --- | --- | --- | --- |
| 1 | Pose rate | 60 s idle + 60 s driving, count `latest()` updates with new `tNs` | ≥ 30 Hz sustained |
| 2 | Latency | Flash a LED visible to the camera; compare the frame timestamp of first brightness change to a hub timestamp | ≤ 60 ms end-to-end |
| 3 | Scale X / Y | Slide 1.000 m along tape, 10 reps each axis | mean within 1 %, σ < 1 % |
| 4 | Sign & axes | Push forward: x↑. Push left: y↑. Turn CCW: heading↑ | all three |
| 5 | Spin leak | 5 × 360° in place | x, y within 1 cm (lever arm) |
| 6 | Straight drift | 3 m forward under power, tape at endpoint | ≤ 1 % |
| 7 | Closed square | 2 × 2 m by hand, return to mark | ≤ 5 cm, ≤ 2° |
| 8 | Speed | Repeat 6 at 25/50/75/100 % power | tracking never lost; drift ≤ 1 % at all |
| 9 | Fast rotation | 3 full spins at max rotational speed | status stays `tracking`; heading error ≤ 3° vs IMU |
| 10 | Lighting | Repeat 7 with arena-style lights on, then dim | same drift |
| 11 | Dynamic scene | Repeat 7 with 2 people walking around the robot | ≤ 2× the static drift |
| 12 | Tag correction | Approach a wall tag from 2 m on VIO only, tags enabled | correction jump ≤ 3 cm |
| 13 | Full auto × 10 | Your real autonomous | endpoint spread ≤ 3 cm (tags), ≤ 5 cm (no tags) |
| 14 | Endurance | 10-minute continuous drive | no divergence, no thermal crash, rate ≥ 30 Hz at the end |
| 15 | Side-by-side | Any of 6/7/13 with a Pinpoint logging in parallel | table in README |

Tooling: `tools/plot_log.py <log.csv>` (top-down path, status colouring, tag-fix markers) and
`tools/compare_logs.py <astro.csv> <pinpoint.csv>` (time-aligned error plot).

Known failure cases to document with observed behaviour: featureless wall close-ups, bright
specular floor glare, another robot filling the frame, phone case blocking the lens corner.

## Beating 3–5 cm: the accuracy and speed strategy

Raw monocular VIO alone will not reliably land under 3 cm on a robot that drives fast and turns
hard; published EuRoC errors for good systems are several centimetres on small rooms. The way to
get below the original's 3–5 cm is to stack corrections, each measured by the table above:

1. **Calibration first (05):** intrinsics ≤ 0.3 px, time offset and extrinsics frozen, lever arm ≤ 1 cm.
2. **Zero-velocity updates** (`zupt_*`) whenever the robot is stationary or the IMU says so.
3. **Absolute correction from field AprilTags** (the `TagCorrector` in 02): any tag fix resets
   accumulated drift; test 12 and 13 measure it. Put 2–3 tags where autonomous paths can see them.
4. **Fuse hub IMU heading** only if the phone's yaw drifts more than the hub's in test 9
   (decide from data, not by assumption).
5. **Keep it fast**: tune `num_pts`, `max_clones`, resolution and frame rate until
   test 1 holds ≥ 30 Hz sustained and test 14 holds after 10 minutes of heat. A pose that arrives
   late is worse than a slightly less accurate pose that arrives on time.

New tightened targets once the above is in: test 7 ≤ 3 cm, test 13 ≤ 2 cm with tags and
≤ 4 cm without, test 2 ≤ 40 ms. Treat these as goals; publish whatever the measured numbers are.
