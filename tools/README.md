# Beacon log tools

Three small command-line tools for looking at Beacon logs after a practice run.
They need Python 3.11 and the packages in `requirements.txt`.

## One-time setup (macOS)

From the repo root:

```
/opt/homebrew/bin/python3.11 -m venv tools/.venv
tools/.venv/bin/python -m pip install -r tools/requirements.txt
```

Run the tests to check the setup:

```
tools/.venv/bin/python -m pytest tools/tests -q
```

Every example below uses `tools/.venv/bin/python`. Run them from the repo root.

## plot_log.py: draw a pose log

```
tools/.venv/bin/python tools/plot_log.py /sdcard/FIRST/beacon/logs/run1.csv
tools/.venv/bin/python tools/plot_log.py run1.csv --tags run1_tags.csv --out run1_plot.png
```

Draws the top-down path (x against y, equal scale), coloured by status, with the start (open
circle) and end (square) marked. `--tags` overlays tag fixes from a file with header
`t_ns,x,y,heading`. The PNG defaults to the log name with `.png`. It prints the number of rows,
the duration, the path length, and the share of rows in each status.

## compare_logs.py: Beacon against a reference odometry log

```
tools/.venv/bin/python tools/compare_logs.py run1.csv pinpoint_run1.csv
tools/.venv/bin/python tools/compare_logs.py run1.csv pinpoint_run1.csv --out errors.png
```

The reference file has header `t_ns,x,y,heading` (metres, radians, same clock as the Beacon log).
The reference is interpolated onto the Beacon timestamps where the two logs overlap. The tool
plots position error and heading error against time, and prints the mean, RMS and maximum
position error (m) and the maximum heading error (deg). Only x and y are compared. The two logs
must already share a field frame and origin; the tool does not align them.
The default plot is the Beacon name with `_compare.png`.

## check_calibration.py: sanity-check calibration YAML

```
tools/.venv/bin/python tools/check_calibration.py calibration/phones/galaxy_s22/*.yaml
```

Works out from the keys whether each file is a camera chain (`cam0`) or an IMU file (`imu0`).
Prints one PASS or FAIL line per check. Exit status:

- `0`: every check passed.
- `1`: at least one check failed.
- `2`: a file is missing, empty, not YAML, or is neither kind of file.

## Input errors

Bad input prints one line starting with `Error:` on stderr and exits with status 2. It does not
print a stack trace. Plot and compare tools need a header that matches exactly and at least two
rows with strictly increasing `t_ns`.
