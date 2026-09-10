#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

docker compose --env-file .env -f docker/compose.yaml run --rm single_fr3 \
  bash -lc '
set -euo pipefail
source /etc/os-release
[[ "$ID" == "ubuntu" && "$VERSION_ID" == "24.04" ]]
[[ "$(uname -m)" == "x86_64" ]]
[[ -r "$OMEGA7_SDK_LIBRARY" ]]
if ldd "$OMEGA7_SDK_LIBRARY" | grep -q "not found"; then
  ldd "$OMEGA7_SDK_LIBRARY"
  exit 1
fi
python3 - <<"PY"
import ctypes
import os

library = ctypes.CDLL(os.environ["OMEGA7_SDK_LIBRARY"])
for symbol in (
    "dhdOpen",
    "dhdClose",
    "dhdEnableForce",
    "dhdEmulateButton",
    "dhdGetButton",
    "dhdGetPositionAndOrientationFrame",
):
    getattr(library, symbol)
print("OMEGA7_COMPAT=PASS ubuntu=24.04 arch=x86_64 sdk=3.17.7")
PY
'
