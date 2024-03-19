#!/bin/bash
# -----------------------------------------------------------------------------
# post_create.sh: postCreateCommand-command writing to $HOME/.bashrc
#
# Author: Bernhard Bablok
#
# -----------------------------------------------------------------------------

echo -e "[post_create.sh] starting postCreateCommand $0\n"
echo -e "[post_create.sh] PWD=$PWD\n"

cat >> $HOME/.bashrc << "EOF"

if [ -f /workspaces/install_build_env.log ]; then
  # setup already done
  echo "CircuitPython build-environment ready for $TOOLCHAIN/$PORT"
  echo "To start a build run:"
  echo "  cd ports/$PORT"
  echo "  time make -j $(nproc) BOARD=pimoroni_badger2040w TRANSLATION=de_DE"
elif [ -f /workspaces/install_build_env.log.active ]; then
  echo "initial setup of build environment in progress, please wait"
  echo "use 'tail -f /workspaces/install_build_env.log.active' to monitor progress"
else
  echo "starting initial setup of build environment, please wait"
  nohup /workspaces/circuitpython/.devcontainer/install_build_env.sh >> $HOME/nohup.out &
  echo "use 'tail -f /workspaces/install_build_env.log.active' to monitor progress"
fi

EOF
touch /workspaces/post_create.finished

# --- that's it!   ------------------------------------------------------------

echo -e "[post_create.sh] setup complete\n"
