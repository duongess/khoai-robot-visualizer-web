#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FRAMEWORK_DIR="$(cd "$ROOT_DIR/../khoai-robot-control-framework" && pwd)"
ADDRESS="127.0.0.1:18080"
LEARNER_LOG="$(mktemp /tmp/swarmdex-learner.XXXXXX)"
VISUALIZER_LOG="$(mktemp /tmp/swarmdex-visualizer.XXXXXX)"

cleanup() {
  kill -- "-${VISUALIZER_PID:-}" "-${LEARNER_PID:-}" 2>/dev/null || true
  wait "${VISUALIZER_PID:-}" 2>/dev/null || true
  wait "${LEARNER_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT

wait_for_url() {
  local url="$1"
  for _ in $(seq 1 40); do
    if curl -fsS "$url" >/dev/null 2>&1; then return 0; fi
    sleep 0.25
  done
  return 1
}

cd "$ROOT_DIR"
make build
setsid bash -c "cd \"$FRAMEWORK_DIR\" && exec poetry run python -m ai" >"$LEARNER_LOG" 2>&1 &
LEARNER_PID=$!
setsid bash -c "cd \"$ROOT_DIR\" && FORCE_CONTROL_ADDR=\"$ADDRESS\" exec ./bin/force-control-demo" >"$VISUALIZER_LOG" 2>&1 &
VISUALIZER_PID=$!

if ! wait_for_url "http://$ADDRESS/api/health"; then
  cat "$LEARNER_LOG" "$VISUALIZER_LOG"
  exit 1
fi

for _ in $(seq 1 40); do
  STATUS="$(curl -fsS "http://$ADDRESS/api/status")"
  if grep -q '"training_batches":[1-9]' <<<"$STATUS"; then break; fi
  sleep 0.25
done
grep -q '"total_steps":[1-9]' <<<"$STATUS"
grep -q '"training_batches":[1-9]' <<<"$STATUS"

node --input-type=module -e "await new Promise((resolve, reject) => { const socket = new WebSocket('ws://$ADDRESS/ws'); const timer = setTimeout(() => reject(new Error('telemetry timeout')), 3000); socket.addEventListener('message', event => { const snapshot = JSON.parse(event.data); const worker = snapshot.worker; if (snapshot.type !== 'simulation_snapshot' || !worker.workspace) reject(new Error('unexpected telemetry')); const { minX, maxX, minY, maxY } = worker.workspace; const { carriage_x: x, gripper_y: y } = worker.gantry; if (minX !== 0 || maxX !== 6 || minY !== 0 || maxY !== 3.2 || x < minX || x > maxX || y < minY || y > maxY) reject(new Error('out-of-bounds telemetry')); clearTimeout(timer); socket.close(); resolve(); }); socket.addEventListener('error', () => reject(new Error('WebSocket error'))); });"
printf '%s\n' "End-to-end smoke test passed."
