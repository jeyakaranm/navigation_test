#!/usr/bin/env bash
#
# Convenience launcher for the docking demo WITH disturbances enabled
# (sensor noise + moving obstacle on the namespaced /scan_dock, /odom_dock).
#
# This is a thin wrapper around run_demo.sh that forces "disturbed" mode and
# forwards any extra flags (--headless, --rviz, --eval N).
#
# Usage:
#   scripts/run_disturbed.sh                 # GUI, disturbances on
#   scripts/run_disturbed.sh --rviz          # GUI + RViz
#   scripts/run_disturbed.sh --headless      # headless (CI / no display)
#   scripts/run_disturbed.sh --eval 10       # 10-trial statistics run
#
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec "${SCRIPT_DIR}/run_demo.sh" disturbed "$@"
