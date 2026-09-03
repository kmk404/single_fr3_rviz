#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

docker compose --env-file .env -f docker/compose.yaml run --rm single_fr3 \
  bash -lc /workspace/ops/smoke_fake.sh

