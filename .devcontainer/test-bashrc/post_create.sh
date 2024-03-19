#!/bin/bash
# -----------------------------------------------------------------------------
# post_create.sh: postCreateCommand-command writing to $HOME/.bashrc
#
# Author: Bernhard Bablok
#
# -----------------------------------------------------------------------------

echo -e "[post_create.sh] starting postCreateCommand $0\n"
echo -e "[post_create.sh] PWD=$PWD\n"

echo -e \
  "\n/workspaces/circuitpython/.devcontainer/install_build_env.sh" >> $HOME/.bashrc
touch /workspaces/post_create.finished

# --- that's it!   ------------------------------------------------------------

echo -e "[post_create.sh] setup complete\n"
