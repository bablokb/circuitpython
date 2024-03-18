#!/bin/bash
# -----------------------------------------------------------------------------
# install_build_env.sh: install build-environment for CircuitPython
#
# Normally, this should run directly as postCreateCommand during container
# creation. Due to an unresolved bug on how Github-codespaces creates a clone,
# this script is run from $HOME/.bashrc instead.
#
# The script delegates parts to other scripts for reuse across toolchains.
# This has the added benefit that they can be called independently later again
# if necessary.
#
# The scripts expect the environment-variables TOOLCHAIN and PORT to be set
# to valid values. This is normally done from within
#   .devcontainer/<port>/devcontainer.json
#
# Author: Bernhard Bablok
#
# -----------------------------------------------------------------------------

REPO_ROOT="/workspaces/circuitpython"

echo -e "[install_build_env.sh] starting install"

# only run when
#   - connected to a terminal
#   - we did not already run

if [ ! -t 1 ]; then
  # not connected to a terminal
  exit 0
elif [ -f /workspaces/install_build_env.log ]; then
  # setup already done
  echo "CircuitPython build-environment ready for $TOOLCHAIN/$PORT"
  echo "To start a build run:"
  echo "  cd ports/$PORT"
  echo "  time make -j $(nproc) BOARD=pimoroni_badger2040w TRANSLATION=de_DE"
  exit 0
fi

# delegate install steps to other scripts
(
"$REPO_ROOT/.devcontainer/$TOOLCHAIN-toolchain.sh" || exit 3
"$REPO_ROOT/.devcontainer/common_tools.sh" || exit 3
"$REPO_ROOT/.devcontainer/make-mpy-cross.sh" || exit 3
"$REPO_ROOT/.devcontainer/fetch-port-submodules.sh"
) | tee /workspaces/install_build_env.log
