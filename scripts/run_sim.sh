#!/usr/bin/env bash
# Clean start of the simulation, for WSL2 and several robots.
#
#   ~/fob3_ws/src/ros2-perimeter-interception/scripts/run_sim.sh            # three robots
#   ROBOTS=lup2 .../run_sim.sh                                              # one robot
#   .../run_sim.sh mode:=naive rviz:=false                                  # extra launch arguments
#
# 1. stops whatever a previous run left behind (Gazebo, discovery servers,
#    RViz, ROS daemon) and removes stale Fast DDS shared memory files;
# 2. clears DDS settings exported by hand in the shell;
# 3. starts the launch with a Fast DDS discovery server on a free port, so
#    the ~50 processes of three robots all find each other;
# 4. after CHECK_AFTER seconds prints whether every robot localized and a startup
#    report, also saved in ~/fob3_startup.txt.
# The terminal stays quiet: the ROS output goes to ~/fob3_last.log only
# (VERBOSE=1 shows it in the terminal too). Stop with Ctrl+C.

set -u
ROBOTS=${ROBOTS:-lup1,lup2,lup3}
CHECK_AFTER=${CHECK_AFTER:-90}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS=${WS:-$(cd "$SCRIPT_DIR/../../.." && pwd)}
LOG=${LOG:-$HOME/fob3_last.log}
REPORT=${REPORT:-$HOME/fob3_startup.txt}
VERBOSE=${VERBOSE:-0}

echo "[1/4] stopping leftovers of previous runs"
for p in "gz sim" "fast-discovery" "fastdds discovery" "rviz2" "_ros2_daemon"; do
    pkill -9 -f "$p" 2>/dev/null
done
sleep 1
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null

echo "[2/4] clearing DDS settings of this shell"
unset FASTDDS_BUILTIN_TRANSPORTS ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT RMW_FASTRTPS_USE_QOS_FROM_XML
export GZ_IP=127.0.0.1      # Gazebo transport on loopback (same as the launch)
if grep -qE "ROS_DISCOVERY_SERVER|ROS_SUPER_CLIENT|FASTDDS_BUILTIN_TRANSPORTS" "$HOME/.bashrc" 2>/dev/null; then
    echo "      note: ~/.bashrc still sets DDS variables; remove them with:"
    echo "      sed -i '/ROS_DISCOVERY_SERVER/d; /ROS_SUPER_CLIENT/d; /FASTDDS_BUILTIN_TRANSPORTS/d' ~/.bashrc"
fi

PORT=11811
while ss -lun 2>/dev/null | grep -q ":$PORT "; do
    PORT=$((PORT + 1))
done
echo "[3/4] discovery server on UDP port $PORT, robots: $ROBOTS"

if [ ! -f "$WS/install/setup.bash" ]; then
    echo "ERROR: $WS/install/setup.bash not found; build first (cd $WS && colcon build)" >&2
    exit 1
fi
if ! grep -q "discovery_server" "$WS/install/perimeter_defense_bringup/share/perimeter_defense_bringup/launch/m2_track.launch.py"; then
    echo "ERROR: the installed launch files are old; rebuild first:" >&2
    echo "       cd $WS && colcon build && source install/setup.bash" >&2
    exit 1
fi
set +u      # the ROS setup scripts use unset variables
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

n_robots=$(echo "$ROBOTS" | tr ',' '\n' | grep -c .)
T0=$(date +%s.%N)
(
    sleep "$CHECK_AFTER"
    active=$(grep -c "lifecycle_manager_localization.*Managed nodes are active" "$LOG")
    {
    echo
    echo "============================================================"
    if [ "$active" -ge "$n_robots" ]; then
        echo " [4/4] OK: $active robots localized (interceptors and transport)."
        echo "       RViz and http://localhost:2525 (F5) should show them."
    else
        echo " [4/4] PROBLEM: only $active of $n_robots robots localized after ${CHECK_AFTER} s."
        echo "       Send the output of:"
        echo "       grep -E 'process has died|Server is running|Managed nodes are active|Aborting' $LOG"
    fi
    # Startup timeline: seconds after the start for each robot.
    python3 - "$LOG" "$T0" <<'PY'
import re, sys
log, t0 = sys.argv[1], float(sys.argv[2])
steps = [('localized', r'\[(\w+)\.lifecycle_manager_localization\].*Managed nodes are active'),
         ('navigation', r'\[(\w+)\.lifecycle_manager_navigation\].*Managed nodes are active'),
         ('patrol', r'\[(\w+)\.patrol\].*patrol waypoint')]
first = {}
for line in open(log, errors='ignore'):
    line = re.sub(r'\x1b\[[0-9;]*m', '', line)
    m = re.search(r'\[(\d{10}\.\d+)\]', line)
    for k, p in steps:
        mm = m and re.search(p, line)
        if mm:
            first.setdefault((mm.group(1), k), float(m.group(1)) - t0)
robots = sorted({r for r, _ in first})
print('       startup (s after start):  robot  localized  navigation  first patrol goal')
for r in robots:
    cells = [f"{first[(r, k)]:6.0f}" if (r, k) in first else '     -' for k, _ in steps]
    print(f'                                 {r:5s}  {cells[0]}     {cells[1]}      {cells[2]}')
PY
    # Average speed over 5 s (the instantaneous factor of /stats jumps around).
    stats() { timeout 5 gz topic -e -t /stats -n 1 2>/dev/null | python3 -c '
import re, sys
t = sys.stdin.read()
def get(name):
    m = re.search(name + r" \{\s*sec: (\d+)(?:\s*nsec: (\d+))?", t)
    return int(m.group(1)) + int(m.group(2) or 0) * 1e-9 if m else None
print(get("sim_time"), get("real_time"))'; }
    read -r s1 r1 <<< "$(stats)"; sleep 5; read -r s2 r2 <<< "$(stats)"
    rtf=$(python3 -c "print(f'{($s2 - $s1) / ($r2 - $r1):.2f}')" 2>/dev/null)
    echo "       simulation speed (real time factor): ${rtf:-unknown} (1.0 = real time)"
    echo "       (this report is saved in $REPORT)"
    echo "============================================================"
    } | tee "$REPORT"
) &
CHECKER=$!

# Lighter defaults (overridden by arguments given on the command line): low graphics
# quality, and on WSL2 the sensors rendered by the GPU through the display instead of the
# CPU (offscreen rendering does not use the GPU there).
DEFAULTS=(quality:=low)
if grep -qi microsoft /proc/version 2>/dev/null && [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    DEFAULTS+=(headless_rendering:=false)
fi
echo "      launch options: ${DEFAULTS[*]} $*"
echo "      simulation starting; ROS output in $LOG"
echo "      report in ${CHECK_AFTER} s; dashboard: http://localhost:2525 ; stop: Ctrl+C"
if [ "$VERBOSE" = "1" ]; then
    ros2 launch perimeter_defense_bringup m2_track.launch.py robots:="$ROBOTS" \
        discovery_server:=true discovery_port:="$PORT" headless:=true "${DEFAULTS[@]}" "$@" 2>&1 | tee "$LOG"
else
    ros2 launch perimeter_defense_bringup m2_track.launch.py robots:="$ROBOTS" \
        discovery_server:=true discovery_port:="$PORT" headless:=true "${DEFAULTS[@]}" "$@" > "$LOG" 2>&1
fi
kill "$CHECKER" 2>/dev/null
echo "      simulation stopped"
