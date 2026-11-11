#!/bin/bash
# Switch the rover from Project #1's services to Project #2's.
#
# Stops and disables Project #1's always-on services:
#   rover-bridge and rover-control
# then enables and starts Project #2's:
#   stereo-rover-bridge and stereo-rover-control
# Only one project's services may run at a time: they share the motor board's
# serial port, the lidar, the cameras, the pendant topics and the Foxglove port
# 8765. No unit file is changed. Project #2's units must be installed first, with
#   ~/stereo_ws/tools/systemd/install.sh
#
# Refuses to switch while either project's stack (bringup, Nav2 or exploration)
# is running, so no run is cut off: stop it from the pendant first.
# Run as your own user, without sudo.
set -uo pipefail

TO_NAME="Project #2"
WORKSPACE="$HOME/stereo_ws"
INSTALL_HINT="~/stereo_ws/tools/systemd/install.sh"
STOP_UNITS="rover-bridge.service rover-control.service"
START_UNITS="stereo-rover-bridge.service stereo-rover-control.service"
STACK_UNITS="rover-explore.service rover-nav2.service rover-bringup.service"
STACK_UNITS="$STACK_UNITS stereo-rover-explore.service stereo-rover-nav2.service stereo-rover-bringup.service"
UNIT_DIR="$HOME/.config/systemd/user"

fail() {
    echo "Switch to $TO_NAME FAILED: $1" >&2
    exit 1
}

[ "$(id -u)" -ne 0 ] || fail "run it as your own user, not with sudo: these are user services."
[ -f "$WORKSPACE/install/setup.bash" ] || fail "$WORKSPACE is not built (no install/setup.bash)."
for unit in $START_UNITS; do
    [ -f "$UNIT_DIR/$unit" ] || fail "$unit is not installed. Run $INSTALL_HINT first."
done
systemctl --user daemon-reload || fail "cannot reach the user service manager (systemctl --user)."

busy=""
for unit in $STACK_UNITS; do
    state="$(systemctl --user is-active "$unit" 2> /dev/null)"
    case "$state" in
        inactive|failed) ;;
        *) busy="$busy $unit (${state:-no answer})" ;;
    esac
done
[ -z "$busy" ] || fail "the stack is running:$busy. Stop it from the pendant first."

for unit in $STOP_UNITS; do
    if [ -f "$UNIT_DIR/$unit" ]; then
        echo "Stopping and disabling $unit"
        systemctl --user disable --now "$unit" || fail "could not stop and disable $unit."
    fi
done
before=""
for unit in $START_UNITS; do
    echo "Enabling and starting $unit"
    systemctl --user enable --now "$unit" || fail "could not enable and start $unit."
    n="$(systemctl --user show -p NRestarts --value "$unit" 2> /dev/null)"
    before="$before ${n:-unknown}"
done

sleep 5
ok=1
for unit in $STOP_UNITS; do
    active="$(systemctl --user is-active "$unit" 2> /dev/null)"
    enabled="$(systemctl --user is-enabled "$unit" 2> /dev/null)"
    echo "$unit: ${active:-no answer}, ${enabled:-not installed}"
    case "$active" in inactive|failed) ;; *) ok=0 ;; esac
    [ "$enabled" != enabled ] || ok=0
done
set -- $before
for unit in $START_UNITS; do
    active="$(systemctl --user is-active "$unit" 2> /dev/null)"
    enabled="$(systemctl --user is-enabled "$unit" 2> /dev/null)"
    restarts="$(systemctl --user show -p NRestarts --value "$unit" 2> /dev/null)"
    echo "$unit: ${active:-no answer}, ${enabled:-no answer}, restarted ${restarts:-?} times (${1:-?} at start)"
    { [ "$active" = active ] && [ "$enabled" = enabled ] && [ -n "$restarts" ] && [ "$restarts" = "${1:-}" ]; } || ok=0
    shift
done
[ "$ok" -eq 1 ] || fail "the services did not end up as expected (see above)."
echo "$TO_NAME is live."
