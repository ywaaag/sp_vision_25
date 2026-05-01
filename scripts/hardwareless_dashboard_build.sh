#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

docker compose -f docker-compose.dev.yml build
docker compose -f docker-compose.dev.yml run --rm sp-vision-dev \
  bash -lc 'cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build --target auto_aim_dashboard_hardwareless_test -j"$(nproc)"'
