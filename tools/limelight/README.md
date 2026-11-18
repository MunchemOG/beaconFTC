# Limelight 3A on a Mac: view, record, calibrate

Use these tools with the Limelight plugged straight into a Mac by USB-C, no Control Hub needed.
They let you:

- check the camera stream (`ll_view.py`),
- record footage into a Beacon dataset folder (`ll_record.py`),
- measure the lens from checkerboard footage (`ll_calibrate.py`),
- send the camera image without any drawing on it (`passthrough_snapscript.py`, which runs on
  the Limelight).

**Team-confirmed (2026-11-17):** the Limelight 3A's MJPEG stream works on the team's Mac through
`limelight.local`. The team has not yet said which port or URL they used. Use port 5800 first.
If that fails, use the stream URL from the web page (see step 3).

The steps below run from the repo root (`beaconFTC/`) in Terminal.

---

## One-time setup

1. Install Homebrew's Python 3.11 if you don't have it: `brew install python@3.11`.
2. Create the tools environment. Skip this if `tools/.venv` already exists.
   ```
   /opt/homebrew/bin/python3.11 -m venv tools/.venv
   ```
3. Install the packages:
   ```
   tools/.venv/bin/python -m pip install -r tools/requirements.txt -r tools/limelight/requirements.txt
   ```
4. Check the setup. This runs against a fake Limelight, so no device is needed:
   ```
   tools/.venv/bin/python -m pytest tools/tests/test_limelight_tools.py -q
   ```
   Expect `17 passed`.

---

## Step 1: plug in

Use a USB-C cable that carries data. Many charge-only cables do not, and nothing will connect.
Plug the Limelight into the Mac, then wait **15-20 seconds** for the green status light. The USB
port powers the Limelight (it draws 4 W at most), so it needs no other power.

## Step 2: open the Limelight web page

In Safari or Chrome, open **http://limelight.local:5801**. You should see the Limelight
interface with a live camera image. If the page does not open, see
[If it does not connect](#if-it-does-not-connect).

## Step 3: check the stream from the Mac

```
tools/.venv/bin/python tools/limelight/ll_view.py
```

A window opens with the camera image. A text line across the top shows:
- frame size (for example `640x480`),
- frames per second (fps) measured on the Mac,
- average JPEG size in KB,
- average time between frames, plus its jitter (the spread) and the longest gap.

All numbers cover the last 2 seconds. Press **q** to quit and **s** to save a PNG of the current
frame. Write down the fps and jitter: the team needs these numbers.

The default URL is `http://limelight.local:5800/`. If that fails, find the stream's real address:
on the web page from step 2, right-click the camera image and choose **Copy Image Address**.
Then pass that address:
```
tools/.venv/bin/python tools/limelight/ll_view.py --url "http://limelight.local:5800/"
```
Paste your copied address in place of the one in quotes. Use the URL that works in every command
below too. Tell the team which URL worked, so this README can name it.

`--no-window` prints the numbers once a second instead of opening a window.

## Step 4: set up the pipeline for Beacon (in the web page)

Beacon needs the plain camera image, with nothing drawn on it:

1. In the web page, choose an unused pipeline slot and set its **pipeline type to Python**
   (SnapScript). A code editor appears.
2. Delete the example code. Paste the whole of `tools/limelight/passthrough_snapscript.py` in its
   place, then save. The script returns each frame unchanged: no contour, no boxes, no text.
3. In the pipeline's input settings:
   - **Resolution:** 640x480 at the highest frame rate offered (the 3A's datasheet says up to
     90 fps).
   - **Exposure:** as short as you can while the image stays readable. Short exposure means less
     blur when the robot moves fast. Raise **sensor gain** if the image gets too dark.
4. Check the stream again with `ll_view.py`. If a crosshair or any text still shows on the
   image, look for a crosshair or overlay setting in the pipeline's output settings and turn it
   off. Report what you found: Beacon needs an image with no drawing on it.

We have not checked these button and tab names on a real Limelight. They come from the Limelight
documentation. If yours differ, write down what you did.

## Step 5: record footage

Keep recordings outside the repo (they are large). One folder per recording:

```
tools/.venv/bin/python tools/limelight/ll_record.py --out ~/beacon-runs/room1
```

Press **Ctrl-C** to stop. You can also stop it automatically:
- `--duration 60` stops after 60 seconds,
- `--max-frames 500` stops after 500 frames.

When it stops it prints a summary like this:

```
Recorded 1800 frames in 60.0 s (30.0 fps), 640x480, mean JPEG 30.1 KB, dropped 0, undecodable 0 -> ...
```

The recording folder holds:

| File | Contents |
| --- | --- |
| `frames.csv` | `idx,t_ns`, one row per frame |
| `frames/000000.png`, ... | each frame as an 8-bit gray PNG |
| `camera.json` | lens numbers (see below) |

`t_ns` is when the frame **arrived at the Mac**, in nanoseconds, on the Mac's monotonic clock.
It is not the moment the camera took the picture. A recording has no `odom.csv`, because a Mac
has no Pinpoint.

`camera.json` holds **uncalibrated** values from the datasheet (horizontal field of view 54.5°,
no lens distortion), so you can replay the recording at once. For accurate work, calibrate once
(step 6). Then record with `--camera ~/beacon-runs/board1/camera.json`, and the calibrated file
is copied in instead. `--field beacon-field/<season>.json` also copies in the field model, as
`field.json`.

## Step 6: calibrate the lens (once per Limelight, per resolution)

1. Print a checkerboard of **10 x 7 squares** (9 x 6 inner corners), such as OpenCV's standard
   `pattern.png`. Print it at 100 % scale and tape it **flat** to a stiff board. Measure one
   square with a ruler, in mm.
2. Record while you move the board slowly. Cover the middle and every corner of the image, at
   several distances, tilted up to about 45° in different directions. Keep the whole board in view
   and sharp. One to two minutes is plenty:
   ```
   tools/.venv/bin/python tools/limelight/ll_record.py --out ~/beacon-runs/board1 --duration 90
   ```
3. Calibrate. Replace `25` with the square size you measured:
   ```
   tools/.venv/bin/python tools/limelight/ll_calibrate.py --run ~/beacon-runs/board1 --board 9x6 --square-mm 25
   ```
   It prints the RMS reprojection error, fx/fy/cx/cy, the field of view and the distortion. It
   writes `camera.json` and `calibration_report.json` into the recording folder. A good result
   has an RMS below about 0.5 px. Above 1 px, record again: board flat, well lit, moved slowly.

`--board` counts **inner** corners, as columns x rows. Calibrate again whenever you change the
resolution. `camera.json` also holds two values you fill in later: `mount`, the camera's
position and angle on the robot, and `rollingShutterReadoutS`. Calibration keeps whatever is
already in an existing `camera.json`.

---

## Run Beacon live on the Mac

The Beacon core is built by the native engineer in `beacon-native/`. It has an **explore mode**
with a 3-D viewer (Rerun), which works like the team's reference SLAM videos: tracked features
on the camera image, a growing point cloud, keyframes and loop closures.

- Live, from the Limelight (the viewer opens by default; `--no-view` turns it off):
  ```
  beacon-native/build/beacon_live --url http://limelight.local:5800 --mode explore \
      --camera ~/beacon-runs/board1/camera.json
  ```
- Replaying a recording from step 5:
  ```
  beacon-native/build/beacon_replay --run ~/beacon-runs/room1 --mode explore --view
  ```

**Status (2026-11-17): new and not yet tried with a Limelight.** The native engineer has just
built both programs, and their options may still change: `beacon-native/README.md` has the
current build command and options. If `beacon-native/build/beacon_live` is missing on your Mac,
build it first, or record with `ll_record.py` now and replay later. `beacon_replay` reads exactly
what `ll_record.py` writes: `frames.csv`, `frames/` and `camera.json`. Explore mode needs no field
model.

---

## If it does not connect

The tools never crash with a long error dump. They print one line starting with `Error:` and
stop with exit status 2. Match your message:

- **`Could not find 'limelight.local' on the network`**
  1. Is the green light on? Wait 20 s after plugging in.
  2. Try another USB-C cable. It must carry data.
  3. Open **System Settings > Network**. When the Limelight is connected, a new network
     interface should appear (often named like a USB Ethernet adapter). If none appears, the Mac
     does not see the Limelight as a network device. Try another cable or port, and avoid USB
     hubs.
  4. Try the address instead of the name: `--url http://172.29.0.1:5800`, and
     http://172.29.0.1:5801 in the browser. (172.29.0.1 is the address Limelight's docs give for
     a USB-connected 3A with USB ID 0. We have not tried it on a Mac.)
  5. Run Limelight's **Hardware Manager** (docs.limelightvision.io, Downloads) and scan for
     Limelights. It shows the address the camera actually has.
- **`Could not connect to ...: connection refused`**: the Limelight answers, but not on that
  port. The stream is on port **5800** and the web page on **5801**. If the Limelight has only
  just started, wait 20 s.
- **`Could not connect to ...: no answer within 3 s`**: the address is wrong, or the Limelight is
  still starting up. Check steps 1-5 above.
- **`returned a web page, not a camera stream`**: you used the web page port (5801). Use 5800,
  or the address from **Copy Image Address** (step 3).
- **`No frame from ... for 3.0 s`**: the stream stopped. The Limelight may have been unplugged
  or restarted. `ll_record.py` keeps every frame it already wrote and exits with status 1.

Exit status of every tool: `0` OK; `1` the stream was lost part-way (recording kept); `2` could
not start (unreachable, wrong URL, bad arguments, output folder already used, too few
checkerboard views).

## Rules note

These tools are for the bench. In a match, no video may leave the robot (R704 D). The tools are
never part of the robot code.
