#!/bin/bash
# C4/C5 gate through the real fusion path: core fixes -> Java ReplayFusion (BeaconLocalizer) ->
# tools/eval/eval_run.py. Holds orchestration/gradle.lock while Gradle runs (shared checkout).
# Usage (repo root): beacon-native/scripts/fused_gate.sh RUN_DIR OUT_DIR [stage1|stage2] [params.json]
set -uo pipefail
RUN="$1"; OUT="$2"; GATE="${3:-stage1}"; PARAMS="${4:-}"
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
mkdir -p "$OUT"
name=$(basename "$RUN")
args=(--run "$RUN" --out "$OUT/$name-fixes.csv")
[ -n "$PARAMS" ] && args+=(--params "$PARAMS")
"$ROOT/beacon-native/build/beacon_replay" "${args[@]}" > "$OUT/$name-replay.log" 2>&1 || exit 1
until mkdir "$ROOT/orchestration/gradle.lock" 2>/dev/null; do sleep 5; done
(cd "$ROOT" && ./gradlew :TeamCode:testDebugUnitTest --offline -q --tests '*ReplayFusionTest*' \
   -Pbeacon.replay.run="$RUN" -Pbeacon.replay.fixes="$OUT/$name-fixes.csv" -Pbeacon.replay.out="$OUT/$name-fused.csv") \
   > "$OUT/$name-gradle.log" 2>&1
rc=$?
rmdir "$ROOT/orchestration/gradle.lock"
[ $rc -ne 0 ] && { echo "gradle failed, see $OUT/$name-gradle.log"; exit 1; }
"$ROOT/tools/.venv/bin/python" "$ROOT/tools/eval/eval_run.py" --run "$RUN" --est "$OUT/$name-fused.csv" --gate "$GATE"
