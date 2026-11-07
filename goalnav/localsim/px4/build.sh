#!/usr/bin/env bash
# Fetch and build PX4 1.16 SITL (px4_sitl_default) outside the repository.
#
#   nix-shell localsim/px4/shell.nix --run localsim/px4/build.sh
#
# PX4_DIR overrides the checkout (default ~/.cache/snowcrashers/PX4-Autopilot).
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/.cache/snowcrashers/PX4-Autopilot}"
PX4_TAG="${PX4_TAG:-v1.16.2}"
SUBMODULES=(
  src/modules/mavlink/mavlink
  src/drivers/gps/devices
  src/lib/events/libevents
  src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client
  src/lib/heatshrink/heatshrink
  src/lib/crypto/monocypher
  src/lib/cdrstream/cyclonedds
  src/lib/cdrstream/rosidl
  src/modules/zenoh/zenoh-pico
  Tools/simulation/gz
)

if [ ! -d "$PX4_DIR/.git" ]; then
  git clone --depth 1 --branch "$PX4_TAG" https://github.com/PX4/PX4-Autopilot "$PX4_DIR"
fi
cd "$PX4_DIR"
git submodule update --init --depth 1 --jobs 8 "${SUBMODULES[@]}"
# mavlink carries pymavlink (the message generator) as its own submodule.
git -C src/modules/mavlink/mavlink submodule update --init --depth 1

# PX4's CMake would otherwise try to update every submodule itself.
export GIT_SUBMODULES_ARE_EVIL=1
export CCACHE_DISABLE=1
make px4_sitl_default -j"$(nproc)"
echo "Built $PX4_DIR/build/px4_sitl_default/bin/px4"
