#!/usr/bin/env bash
# Download one EuRoC MAV sequence in ASL format into desktop-harness/data/euroc/<sequence>/mav0.
#
# Usage: data/get_euroc.sh MH_01_easy        (or V1_01_easy, MH_03_medium, V1_02_medium, ...)
#
# Where the data lives (verified 2026-11-16): the old robotics.ethz.ch/~asl-datasets host no longer
# answers. The ASL page (projects.asl.ethz.ch/datasets/euroc-mav/) points to the ETH Research
# Collection, DOI 10.3929/ethz-b-000690084 (handle 20.500.11850/690084). That record holds one zip
# per environment: machine_hall.zip (12.7 GB), vicon_room1.zip (6.0 GB), vicon_room2.zip (6.0 GB).
# Each contains <env>/<seq>/<seq>.zip (ASL format) and <seq>.bag. The server supports HTTP Range,
# so zip_member_fetch.py pulls only the ~1.5 GB inner ASL zip instead of the whole environment.
set -euo pipefail

SEQ="${1:-}"
if [[ -z "$SEQ" ]]; then
  echo "usage: $0 <sequence>   e.g. MH_01_easy, V1_01_easy" >&2
  exit 2
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$HERE/euroc/$SEQ"
PY=/opt/homebrew/bin/python3.11
[[ -x "$PY" ]] || PY=python3

case "$SEQ" in
  MH_*) ENV=machine_hall;  FALLBACK_UUID=7b2419c1-62b5-4714-b7f8-485e5fe3e5fe ;;
  V1_*) ENV=vicon_room1;   FALLBACK_UUID=02ecda9a-298f-498b-970c-b7c44334d880 ;;
  V2_*) ENV=vicon_room2;   FALLBACK_UUID=ea12bc01-3677-4b4c-853d-87c7870b8c44 ;;
  *) echo "unknown sequence '$SEQ' (expected MH_xx_*, V1_xx_*, V2_xx_*)" >&2; exit 2 ;;
esac

if [[ -f "$DEST/mav0/cam0/data.csv" && -f "$DEST/mav0/imu0/data.csv" && -f "$DEST/mav0/state_groundtruth_estimate0/data.csv" ]]; then
  echo "$SEQ already present at $DEST"
  exit 0
fi

API=https://www.research-collection.ethz.ch/server/api
ITEM_UUID=bcaf173e-5dac-484b-bc37-faf97a594f1f   # item behind handle 20.500.11850/690084

# Look the outer zip up by name so a re-upload (new bitstream UUID) does not break us.
UUID="$(curl -sf -A 'beacon-desktop-harness/1.0' "$API/core/items/$ITEM_UUID/bundles?embed=bitstreams&size=50" \
  | "$PY" -I -c '
import json, sys
name = sys.argv[1] + ".zip"
d = json.load(sys.stdin)
for b in d["_embedded"]["bundles"]:
    if b["name"] != "ORIGINAL":
        continue
    for bs in b["_embedded"]["bitstreams"]["_embedded"]["bitstreams"]:
        if bs["name"] == name:
            print(bs["uuid"])
' "$ENV" || true)"
if [[ -z "$UUID" ]]; then
  echo "could not query the Research Collection API (rate limited?); using pinned bitstream UUID" >&2
  UUID="$FALLBACK_UUID"
fi
URL="$API/core/bitstreams/$UUID/content"
echo "outer zip: $ENV.zip  ($URL)"

mkdir -p "$DEST"
INNER="$DEST/$SEQ.zip"
if [[ ! -f "$INNER" ]]; then
  "$PY" -I "$HERE/zip_member_fetch.py" "$URL" "$ENV/$SEQ/$SEQ.zip" "$INNER"
fi

echo "unzipping $INNER"
unzip -q -o "$INNER" -d "$DEST"
# Some archives carry macOS metadata folders; drop them.
rm -rf "$DEST/__MACOSX"
if [[ ! -f "$DEST/mav0/cam0/data.csv" ]]; then
  echo "unexpected layout: $DEST/mav0/cam0/data.csv missing" >&2
  ls -la "$DEST" >&2
  exit 1
fi
rm -f "$INNER"   # keep only the extracted ASL tree (saves ~1.5 GB per sequence)
echo "cam0 frames: $(($(wc -l < "$DEST/mav0/cam0/data.csv") - 1))"
echo "imu0 samples: $(($(wc -l < "$DEST/mav0/imu0/data.csv") - 1))"
echo "done: $DEST/mav0"
