#!/usr/bin/env bash
# Run OpenVINS (mono + IMU) on one EuRoC sequence, evaluate it, and show it in Rerun.
#
#   ./run.sh <sequence> [--config DIR] [--out DIR] [--realtime] [--no-view] [--save-rrd]
#
#   <sequence>    e.g. MH_01_easy, V1_01_easy (downloaded on first use)
#   --config DIR  directory holding estimator_config.yaml (default: config/euroc)
#   --out DIR     run directory (default: runs/<sequence>_<yyyymmdd_hhmmss>)
#   --realtime    play at sensor speed instead of as fast as possible
#   --no-view     skip Rerun
#   --save-rrd    write <run>/view.rrd instead of opening the viewer (for headless machines)
#
# Builds everything first if needed (setup.sh). Output: trajectory.csv (project log format),
# features.csv, points.csv, timing.csv, ate.json, speed.json, ate_topdown.png.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

SEQ="${1:-}"
[[ -n "$SEQ" && "$SEQ" != -* ]] || { sed -n '2,15p' "$0" >&2; exit 2; }
shift
CONFIG_DIR="$HERE/config/euroc"
OUT=""
VIEW=open
DRIVER_ARGS=()
while (($#)); do
  case "$1" in
    --config) CONFIG_DIR="$(cd "$2" && pwd)"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --realtime) DRIVER_ARGS+=(--realtime); shift ;;
    --no-view) VIEW=none; shift ;;
    --save-rrd) VIEW=save; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done

if [[ ! -x build/driver/euroc_driver || ! -x .venv/bin/python ]]; then
  ./setup.sh
fi
if [[ ! -f "data/euroc/$SEQ/mav0/cam0/data.csv" ]]; then
  ./data/get_euroc.sh "$SEQ"
fi

OUT="${OUT:-runs/${SEQ}_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"
cp "$CONFIG_DIR"/*.yaml "$OUT/" 2>/dev/null || true   # keep the exact config with the result

echo "==> OpenVINS on $SEQ  (config $CONFIG_DIR, out $OUT)"
./build/driver/euroc_driver "$CONFIG_DIR/estimator_config.yaml" "data/euroc/$SEQ" "$OUT" "${DRIVER_ARGS[@]+"${DRIVER_ARGS[@]}"}"

echo "==> ATE"
.venv/bin/python eval/ate.py "$OUT/trajectory.csv" "data/euroc/$SEQ" --json "$OUT/ate.json"
echo "==> speed"
.venv/bin/python eval/speed.py "$OUT" --json "$OUT/speed.json"

case "$VIEW" in
  open) echo "==> Rerun viewer"; .venv/bin/python viz/view.py "$OUT" ;;
  save) echo "==> Rerun recording"; .venv/bin/python viz/view.py "$OUT" --save "$OUT/view.rrd" ;;
  none) ;;
esac
echo "run directory: $OUT"
