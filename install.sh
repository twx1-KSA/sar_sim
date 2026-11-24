#!/usr/bin/env bash
# Link this repository's Gazebo world and models into a PX4-Autopilot checkout, so PX4 SITL
# and Gazebo find them (PX4 looks for worlds/models inside Tools/simulation/gz).
# Symlinks, not copies: editing files in this repository takes effect immediately.
#
#   ./install.sh                         # PX4 at ~/PX4-Autopilot
#   PX4_DIR=/path/to/PX4-Autopilot ./install.sh
#   ./install.sh --extras                # also the x500_360_cam experiment (airframe 4022)

set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
GZ="$PX4_DIR/Tools/simulation/gz"

[ -d "$GZ/models" ] || { echo "ERROR: $GZ/models not found. Clone PX4 with submodules first (see README)."; exit 1; }

link() {   # link <source in repo> <destination in PX4>
    local src="$1" dst="$2"
    if [ -L "$dst" ]; then
        ln -sfn "$src" "$dst"
    elif [ -e "$dst" ]; then
        echo "ERROR: $dst exists and is not a link; move it away first."; exit 1
    else
        ln -s "$src" "$dst"
    fi
    echo "  linked $dst"
}

echo "Linking SAR world and models into $GZ"
for m in "$REPO"/gz/models/*/; do
    m="${m%/}"; link "$m" "$GZ/models/$(basename "$m")"
done
link "$REPO/gz/worlds/sar_mountain.sdf" "$GZ/worlds/sar_mountain.sdf"

if [ "${1:-}" = "--extras" ]; then
    echo "Installing extras (x500_360_cam model + airframe 4022)"
    link "$REPO/extras/gz/models/x500_360_cam" "$GZ/models/x500_360_cam"
    AF="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
    cp "$REPO/extras/airframes/4022_gz_x500_360_cam" "$AF/"
    grep -q '4022_gz_x500_360_cam' "$AF/CMakeLists.txt" || \
        sed -i 's/^\(\s*\)4021_gz_x500_flow$/&\n\14022_gz_x500_360_cam/' "$AF/CMakeLists.txt"
    echo "  airframe 4022 added; rebuild PX4 (make px4_sitl) to use it"
fi

chmod +x "$REPO"/*.sh
echo "Done."
