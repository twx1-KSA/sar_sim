#!/usr/bin/env bash
# Start the SAR simulation: Gazebo (headless) + one PX4 SITL instance per drone.
#
#   ~/sar_sim/start_drones.sh            # all four drones: 0 1 2 3
#   ~/sar_sim/start_drones.sh 0 1        # only the two search drones
#   GUI=1 ~/sar_sim/start_drones.sh      # also open the Gazebo window
#
# Drone roles and pads (world x east, y north, z up):
#   0  search drone, west half    terrain pad      -55,-20
#   1  search drone, east half    terrain pad       62, 22
#   2  backup drone               blue platform      3,-13
#   3  confirmation drone         orange platform    4, 12
# Logs: ~/sar_sim/logs/px4_<N>.log.  Stop everything with:  ~/sar_sim/stop_all.sh

set -e
DRONES=("$@"); [ ${#DRONES[@]} -eq 0 ] && DRONES=(0 1 2 3)
declare -A POSE=( [0]="-55,-20,32" [1]="62,22,38.4" [2]="3,-13,24.7" [3]="4,12,29.7" )
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
LOG_DIR=$HOME/sar_sim/logs
mkdir -p "$LOG_DIR"
cd "$PX4_DIR"

# Software rendering is much faster than the VMware/VirtualBox 3D driver for camera sensors,
# but much slower than a real GPU. Default: on inside a VM, off on real hardware.
# Override with SOFTWARE_GL=1 or SOFTWARE_GL=0.
if [ -z "${SOFTWARE_GL:-}" ]; then
    [ "$(systemd-detect-virt 2>/dev/null || echo none)" != "none" ] && SOFTWARE_GL=1 || SOFTWARE_GL=0
fi
[ "$SOFTWARE_GL" = "1" ] && export LIBGL_ALWAYS_SOFTWARE=1 && echo "using software rendering (VM detected)"
[ "${GUI:-0}" = "1" ] || export HEADLESS=1

first=1
for n in "${DRONES[@]}"; do
    [ -n "${POSE[$n]}" ] || { echo "unknown drone $n"; exit 1; }
    world_env=()
    [ $first = 1 ] && world_env=(PX4_GZ_WORLD=sar_mountain)   # the first instance starts Gazebo
    echo "starting drone $n at ${POSE[$n]} ..."
    env "${world_env[@]}" PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_dual_cam \
        PX4_GZ_MODEL_POSE="${POSE[$n]}" setsid ./build/px4_sitl_default/bin/px4 -i "$n" -d \
        > "$LOG_DIR/px4_$n.log" 2>&1 &
    for _ in $(seq 60); do grep -q 'Startup script returned' "$LOG_DIR/px4_$n.log" && break; sleep 2; done
    grep -q 'Startup script returned' "$LOG_DIR/px4_$n.log" || { echo "drone $n did not start, see $LOG_DIR/px4_$n.log"; exit 1; }
    first=0
done
sleep 5
echo "drones on their pads (x y z / roll pitch yaw):"
for n in "${DRONES[@]}"; do
    echo "  drone $n: $(gz model -m x500_dual_cam_$n -p 2>/dev/null | sed -n '/Pose/{n;N;p}' | tr -s ' \n' ' ')"
done
echo
echo "next:  source /opt/ros/humble/setup.bash"
echo "       ros2 launch ~/sar_sim/ros2/sar_bringup.launch.py drones:=$(IFS=,; echo "${DRONES[*]}")"
