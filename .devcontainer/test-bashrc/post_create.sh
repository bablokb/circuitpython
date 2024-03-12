#!/bin/bash
# -----------------------------------------------------------------------------
# post_create.sh: postCreateCommand-command writing to $HOME/.bashrc
#
# Author: Bernhard Bablok
#
# -----------------------------------------------------------------------------

echo -e "[post_create.sh] starting postCreateCommand $0\n"
echo -e "[post_create.sh] PWD=$PWD\n"

cat >> $HOME/.bashrc <<"EOF"

if [ ! -f /workspaces/toolchain_setup.log ]; then
  (echo "TOOLCHAIN=$TOOLCHAIN"
   echo "PORT=$PORT"
   tools/describe
   git --no-pager show --no-color --pretty=oneline --decorate
  ) | tee /workspaces/toolchain_setup.log
fi
EOF

# --- that's it!   ------------------------------------------------------------

echo -e "[post_create.sh] setup complete\n"
