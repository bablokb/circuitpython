#!/bin/bash
# -----------------------------------------------------------------------------
# fetch-port-submodules.sh: fetch port specific submodules
#
# Author: Bernhard Bablok
#
# -----------------------------------------------------------------------------

REPO_ROOT="/workspaces/circuitpython"
cd "$REPO_ROOT"

if [ -z "$PORT" ]; then
  echo -e "[fetch-port-submodules.sh] PORT not set. Cannot fetch submodules!"
  exit 3
fi

cd "ports/$PORT"
echo -e "[fetch-port-submodules.sh] fetching necessary submodules"
make fetch-port-submodules
