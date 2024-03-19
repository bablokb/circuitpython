#!/bin/bash
# -----------------------------------------------------------------------------
# post_create.sh: postCreateCommand-command writing to $HOME/.bashrc
#
# Author: Bernhard Bablok
#
# -----------------------------------------------------------------------------

echo -e "[post_create.sh] starting postCreateCommand $0\n"
echo -e "[post_create.sh] PWD=$PWD\n"

#echo -e \
#  "\n/workspaces/circuitpython/.devcontainer/install_build_env.sh" >> $HOME/.bashrc

nohup /workspaces/circuitpython/.devcontainer/nh_test.sh &

# wait a few seconds to make sure any initial terminals are started
sleep 10
touch /workspaces/post_create.finished

# --- that's it!   ------------------------------------------------------------

echo -e "[post_create.sh] setup complete\n"
