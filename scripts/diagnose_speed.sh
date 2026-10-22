#!/usr/bin/env bash
# Measures the simulation speed (real time factor) in a few configurations, to find out
# what slows the simulation down on a given machine. About 5 minutes, no input needed.
#
#   ~/fob3_ws/src/ros2-perimeter-interception/scripts/diagnose_speed.sh
#
# The result is printed at the end and saved to ~/fob3_speed.txt.

set -u
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS=${WS:-$(cd "$SCRIPT_DIR/../../.." && pwd)}
OUT=${OUT:-$HOME/fob3_speed.txt}
WAIT=${WAIT:-50}
LOG=$(mktemp /tmp/fob3_speed_XXXX.log)

cleanup() {
    for p in "gz sim" "fast-discovery" "rviz2" "_ros2_daemon"; do pkill -9 -f "$p" 2>/dev/null; done
    pkill -9 -f "$WS/install/" 2>/dev/null
    # Wait until the previous Gazebo server is really gone (the launch refuses to start
    # next to a running one).
    for _ in $(seq 1 20); do pgrep -f "gz sim" > /dev/null || break; sleep 0.5; done
    sleep 1
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null
}

unset FASTDDS_BUILTIN_TRANSPORTS ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
export GZ_IP=127.0.0.1
set +u
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

RENDER=()
if grep -qi microsoft /proc/version 2>/dev/null && [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    RENDER=(headless_rendering:=false)
fi

stats() {
    timeout 5 gz topic -e -t /stats -n 1 2>/dev/null | python3 -c '
import re, sys
t = sys.stdin.read()
def get(name):
    m = re.search(name + r" \{\s*sec: (\d+)(?:\s*nsec: (\d+))?", t)
    return int(m.group(1)) + int(m.group(2) or 0) * 1e-9 if m else None
print(get("sim_time"), get("real_time"))'
}

rtf() {
    read -r s1 r1 <<< "$(stats)"; sleep 5; read -r s2 r2 <<< "$(stats)"
    python3 -c "print(f'{($s2 - $s1) / ($r2 - $r1):.2f}')" 2>/dev/null || echo "?"
}

top_cpu() {
    # Busiest processes, grouped by program name.
    ps -eo pcpu=,comm= | awk '{c[$2]+=$1} END {for (k in c) if (c[k] > 3) printf "%s %.0f%%\n", k, c[k]}' \
        | sort -k2 -t' ' -n -r | head -6 | tr '\n' ',' | sed 's/,$//; s/,/, /g'
}

run() {   # name, launch file, args...
    local name=$1 file=$2; shift 2
    cleanup
    echo "   - $name ..."
    setsid ros2 launch perimeter_defense_bringup "$file" "$@" > "$LOG" 2>&1 &
    local pid=$!
    sleep "$WAIT"
    local r; r=$(rtf)
    local cpu; cpu=$(top_cpu)
    kill -INT -- -"$pid" 2>/dev/null; sleep 8; kill -9 -- -"$pid" 2>/dev/null
    printf '%-38s %6s   %s\n' "$name" "$r" "$cpu" >> "$OUT.tmp"
}

echo "Measuring the simulation speed (about 5 minutes)..."
{
    echo "machine: $(nproc) CPU threads, $(free -g | awk '/Mem/ {print $2}') GB RAM, $(uname -r)"
    echo "rendering: ${RENDER[*]:-offscreen (headless)}"
    printf '%-38s %6s   %s\n' "configuration" "speed" "busiest processes (CPU, 100% = one thread)"
} > "$OUT.tmp"
run "world only (no robots)"            world.launch.py headless:=true quality:=low "${RENDER[@]}"
run "1 robot"                           m2_track.launch.py robots:=lup2 transports:=none rviz:=false dashboard:=false \
                                        headless:=true quality:=low discovery_server:=true start_delay:=900 "${RENDER[@]}"
run "3 robots + transport"              m2_track.launch.py robots:=lup1,lup2,lup3 rviz:=false dashboard:=false \
                                        headless:=true quality:=low discovery_server:=true start_delay:=900 "${RENDER[@]}"
run "3 robots + transport, lidar 5 Hz"  m2_track.launch.py robots:=lup1,lup2,lup3 rviz:=false dashboard:=false \
                                        headless:=true quality:=low discovery_server:=true start_delay:=900 lidar_rate:=5 \
                                        "${RENDER[@]}"
cleanup
mv "$OUT.tmp" "$OUT"
rm -f "$LOG"
echo
echo "================================================================================"
cat "$OUT"
echo "(saved in $OUT; speed 1.0 = real time)"
echo "================================================================================"
