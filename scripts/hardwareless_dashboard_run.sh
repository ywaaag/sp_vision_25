#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

if [[ "$#" -eq 0 ]]; then
  set -- ./build/auto_aim_dashboard_hardwareless_test \
    --dashboard \
    --mqtt-host tcp://127.0.0.1:1883 \
    --video-source assets/demo/demo.avi \
    --video-loop \
    --max-frames 300 \
    configs/sentry_pose.yaml
fi

docker compose -f docker-compose.dev.yml run --rm sp-vision-dev "$@"
