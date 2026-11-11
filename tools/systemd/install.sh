#!/bin/bash
# Install the Project #2 systemd user units. Enables and starts nothing:
# switch_to_project2.sh moves the rover from Project #1's services to these,
# and switch_to_project1.sh moves it back.
set -euo pipefail

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST_DIR="$HOME/.config/systemd/user"

mkdir -p "$DEST_DIR"
for unit in stereo-rover-bridge.service stereo-rover-control.service stereo-rover-bringup.service stereo-rover-nav2.service stereo-rover-explore.service; do
    install -m 644 "$SRC_DIR/$unit" "$DEST_DIR/$unit"
    echo "Installed $DEST_DIR/$unit"
done

systemctl --user daemon-reload

linger="$(loginctl show-user "$(whoami)" -p Linger)"
echo "$linger"
if [ "$linger" != "Linger=yes" ]; then
    echo "WARNING: lingering is not enabled, so user services stop at logout and do not start at boot." >&2
    echo "WARNING: enable it with: sudo loginctl enable-linger $(whoami)" >&2
fi
