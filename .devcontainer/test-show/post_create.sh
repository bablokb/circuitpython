#!/bin/bash
# -----------------------------------------------------------------------------
# post_create.sh: postCreateCommand-command with git command
#
# Author: Bernhard Bablok
#
# -----------------------------------------------------------------------------

echo -e "[post_create.sh] starting postCreateCommand\n"
echo -e "[post_create.sh] PWD=$PWD\n"

git --no-pager status
git --no-pager show --summary

# --- that's it!   ------------------------------------------------------------

echo -e "[post_create.sh] setup complete\n"
