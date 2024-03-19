#!/bin/bash
# -----------------------------------------------------------------------------
# nh_test.sh: test-script (started with nohup)
#
# Author: Bernhard Bablok
#
# -----------------------------------------------------------------------------

(
while ! test -f /workspaces/post_create.finished; do
  echo -e "[nh_test.sh] waiting for /workspaces/post_create.finished ..."
  sleep 1
done

echo -e "[nh_test.sh] =========================================="
echo -e "tools/describe: $(/workspaces/circuitpython/tools/describe)"
echo -e "[nh_test.sh] =========================================="
) > /workspaces/nh_test.log
