#!/usr/bin/env bash
# Stop everything the SAR simulation started: missions, coordinator, detectors, bridge, PX4, Gazebo.
# Matches on exact program paths/names so it never kills unrelated processes.
for pat in 'sar_mission.py' 'sar_coordinator.py' 'thermal_detector.py' 'sar_bringup.launch.py' \
           'ros_gz_bridge/parameter_bridge' 'px4_sitl_default/bin/px4 -i' 'gz sim --verbose'; do
    for pid in $(ps -eo pid=,args= | awk -v p="$pat" 'index($0, p) && !index($0, "stop_all.sh") && !index($0, "awk") {print $1}'); do
        kill "$pid" 2>/dev/null && echo "stopped $pid ($pat)"
    done
done
sleep 5
# Gazebo sometimes ignores SIGTERM; force-stop anything still running.
for pid in $(ps -eo pid=,args= | awk '(index($0, "px4_sitl_default/bin/px4 -i") || index($0, "gz sim --verbose")) && !index($0, "awk") {print $1}'); do
    kill -9 "$pid" 2>/dev/null && echo "force-stopped $pid"
done
sleep 1
ps -eo args= | grep -E 'px4_sitl_default/bin/px4|gz sim --verbose' | grep -v grep >/dev/null \
    && echo "WARNING: some simulation processes are still running" || echo "all stopped"
