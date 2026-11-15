#!/usr/bin/env bash
# User-local ROS transport. Does not install packages system-wide or edit desktop launchers.
set -euo pipefail
transport_root="${N01_TRANSPORT_ROOT:-$HOME/.local/share/n01_navigation/cyclonedds}"
mkdir -p "$transport_root/packages" "$transport_root/root"
cd "$transport_root/packages"
apt-get download ros-humble-rmw-cyclonedds-cpp ros-humble-cyclonedds \
  ros-humble-iceoryx-binding-c ros-humble-iceoryx-posh ros-humble-iceoryx-hoofs
for package in ./*.deb; do dpkg-deb -x "$package" "$transport_root/root"; done
sha256sum ./*.deb > "$transport_root/packages.sha256"
echo "Project CycloneDDS installed under $transport_root; desktop drivers unchanged."
