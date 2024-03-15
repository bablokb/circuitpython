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

if [ -t 1 -a ! -f /workspaces/toolchain_setup.log ]; then
  (echo "TOOLCHAIN=$TOOLCHAIN"
   echo "PORT=$PORT"
   tools/describe
   git --no-pager show --summary --no-color --pretty=short --decorate
  ) | tee /workspaces/toolchain_setup.log
fi
EOF

# --- that's it!   ------------------------------------------------------------

echo -e "[post_create.sh] setup complete\n"
