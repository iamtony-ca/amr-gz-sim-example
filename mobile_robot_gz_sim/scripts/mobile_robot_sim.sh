#!/bin/bash
#
# mobile_robot_sim.sh
#
# Sim 환경(gz sim) 용 multi-terminal launch script — 실물 로봇의
# mobile_robot_localization.sh 와 같은 구조로, 각 launch 를 별개의 터미널
# (또는 tmux window / 백그라운드 프로세스) 에서 실행.
#
# Active (sim 에서 확실히 필요한 것) 만 위쪽에 두고, 후보 / 실물 전용은
# 아래쪽에 주석 처리. 검토 후 필요한 것만 풀어서 쓰면 됨.
#
# 사용법:
#   bash /root/work_ws/src/amhs/scripts/amr_batch/mobile_robot_sim.sh
#   또는 env 로 override:
#   PARAMS_YAML=<path> HEADLESS=False USE_RVIZ=True bash mobile_robot_sim.sh
#   터미널 종류 강제 지정:
#   TERMINAL_EMU=tmux bash mobile_robot_sim.sh
#   TERMINAL_EMU=background bash mobile_robot_sim.sh
#
# Docker container / 헤드리스 환경에서:
#   - gnome-terminal / xterm 같은 GUI 터미널이 없으면 자동으로 tmux 사용
#   - tmux 도 없으면 background 모드로 fallback (로그는 /tmp/sim_<title>.log)
#   - tmux 설치 권장: apt install -y tmux
#   - tmux 후 attach: tmux attach -t sim   |   window 전환: Ctrl+b → n
#
# 정리 (모든 sim 노드 강제 종료):
#   pkill -9 -f "ros2|gz sim|nav2_|amcl|map_server|filter_mask|costmap_filter|opennav|robot_state_publisher|three_scan_merger|parameter_bridge|rviz2|velocity_modifier"
#   tmux 세션 종료: tmux kill-session -t sim
#

# =====================================================================
# 0. 환경 / 변수 셋업
# =====================================================================
WS_DIR="${MY_WS_DIR:-/root/work_ws}"
ROS_DISTRO="${MY_ROS_DIST:-jazzy}"

# FULL_SETUP="source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WS_DIR}/install/setup.bash && export ROS_DOMAIN_ID=7 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
FULL_SETUP="source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WS_DIR}/install/setup.bash"


# Sim 옵션 (env 로 override 가능)
PARAMS_YAML="${PARAMS_YAML:-${WS_DIR}/src/amhs/mobile_robot/mobile_robot_gz_sim/params/nav2_params_gz_static_filter2.yaml}"
HEADLESS="${HEADLESS:-False}"
USE_RVIZ="${USE_RVIZ:-True}"
MAP_YAML="${MAP_YAML:-${WS_DIR}/src/amhs/mobile_robot/mobile_robot_gz_sim/maps/depot.yaml}"
FILTER_MASK_YAML="${FILTER_MASK_YAML:-${WS_DIR}/src/amhs/mobile_robot/mobile_robot_gz_sim/maps/depot.yaml}"

# =====================================================================
# 0-1. Terminal 자동 감지 + run_launch helper
# =====================================================================
detect_terminal() {
    # env override
    if [ -n "${TERMINAL_EMU}" ]; then
        echo "${TERMINAL_EMU}"; return
    fi
    # GUI terminal emulators (DISPLAY 필요)
    for t in gnome-terminal xterm konsole xfce4-terminal terminator mate-terminal lxterminal tilix kitty; do
        if command -v "$t" > /dev/null 2>&1; then
            echo "$t"; return
        fi
    done
    # 헤드리스: tmux 우선
    if command -v tmux > /dev/null 2>&1; then
        echo "tmux"; return
    fi
    # 최후: background + 로그 파일
    echo "background"
}

TERMINAL_EMU=$(detect_terminal)

# run_launch <title> <command>
#   - GUI 터미널: 새 창에서 실행 (창 끝나도 bash 유지)
#   - tmux: 'sim' 세션의 새 window 로 실행
#   - background: bash -c "<cmd>" & + /tmp/sim_<title>.log 로 stdout/stderr 리다이렉트
run_launch() {
    local title="$1"
    local cmd="$2"
    case "$TERMINAL_EMU" in
        gnome-terminal)
            gnome-terminal --title="$title" -- bash -c "$cmd; exec bash"
            ;;
        # xterm)
        #     xterm -T "$title" -e "bash -c '$cmd; exec bash'" &
        #     ;;
        xterm)
            xterm -fa 'Monospace' -fs 12 -geometry 90x25 -bg black -fg white -T "$title" -e "bash -c '$cmd; exec bash'" &
            ;;
        
        konsole)
            konsole --new-tab --title "$title" -e bash -c "$cmd; exec bash" &
            ;;
        xfce4-terminal|terminator|mate-terminal|lxterminal|tilix)
            "$TERMINAL_EMU" --title="$title" -e "bash -c \"$cmd; exec bash\"" &
            ;;
        kitty)
            kitty --title "$title" bash -c "$cmd; exec bash" &
            ;;
        tmux)
            if ! tmux has-session -t sim 2>/dev/null; then
                tmux new-session -d -s sim -n "$title" "bash -c '$cmd; exec bash'"
            else
                tmux new-window -t sim -n "$title" "bash -c '$cmd; exec bash'"
            fi
            ;;
        background)
            local safe_title="${title//[^a-zA-Z0-9_]/_}"
            local log="/tmp/sim_${safe_title}.log"
            ( bash -c "$cmd" ) > "$log" 2>&1 &
            echo "  [background] $title  PID=$!  log: $log"
            ;;
        *)
            echo "  [ERROR] Unknown TERMINAL_EMU=$TERMINAL_EMU" >&2
            ;;
    esac
}

# =====================================================================
# 시작 배너
# =====================================================================
echo "---------------------------------------------------"
echo "WS_DIR           = ${WS_DIR}"
echo "PARAMS_YAML      = ${PARAMS_YAML}"
echo "MAP_YAML         = ${MAP_YAML}"
echo "FILTER_MASK_YAML = ${FILTER_MASK_YAML}"
echo "HEADLESS         = ${HEADLESS}"
echo "USE_RVIZ         = ${USE_RVIZ}"
echo "TERMINAL_EMU     = ${TERMINAL_EMU}"
case "$TERMINAL_EMU" in
    tmux)        echo "(After all launches: tmux attach -t sim ; window switch: Ctrl+b → n)" ;;
    background)  echo "(No terminal emulator detected. Logs at /tmp/sim_<title>.log — install gnome-terminal/xterm/tmux to get interactive terminals.)" ;;
esac
echo "---------------------------------------------------"


# =====================================================================
# === 1. ACTIVE — sim 에서 필요 (verified) ===
# =====================================================================


run_launch "three_scan_merger" "$FULL_SETUP; ros2 launch three_scan_merger_ros2 scan_merger.launch.py"
sleep 1
# [필수] gz sim 전체 nav 스택
#   gz world + robot spawn + ros_gz_bridge
#   + nav2_bringup localization (AMCL+map_server)
#   + amr_bringup costmap_filter + navigation (run_amr_bringup 경유)
#   + RViz (use_rviz:=True 일 때)
#   (three_scan_merger 는 아래에서 별도 launch — 실물의 mobile_robot_localization.sh 와 동일 패턴)
run_launch "gz_amr_bringup" "$FULL_SETUP; ros2 launch mobile_robot_gz_sim gz_amr_bringup.launch.py headless:=${HEADLESS} use_rviz:=${USE_RVIZ} params_file_source:=${PARAMS_YAML} map:=${MAP_YAML} filter_mask_file:=${FILTER_MASK_YAML}"
sleep 26  # gz/bridge 가 scan 토픽 publish 시작할 때까지 대기

# [필수] three_scan_merger_ros2 — 두 corner lidar(각 270°) 를 합쳐 단일 /scan 생성.
# 실물(mobile_robot_localization.sh) 과 동일 launch / 동일 인자 사용. 노드는
# 입력 LaserScan 의 header.stamp 를 그대로 출력 /scan 에 실어 publish 하므로
# (입력은 ros_gz_bridge 가 이미 sim time stamped), use_sim_time 미설정이라도
# AMCL 은 정상적으로 sim time /scan 을 받음.
# AMCL 활성화 시점(gz_amr_bringup 의 t+10s) 전에 떠 있어야 /scan 못 찾는
# warning 도배를 피함.
# run_launch "three_scan_merger" "$FULL_SETUP; ros2 launch three_scan_merger_ros2 scan_merger.launch.py"
sleep 1  # nav 스택 4/4 lifecycle active 까지 대기 (총 ~25s, t+16s 이후 활성화)

# [필수] velocity_modifier — cmd_vel_nav → cmd_vel 변환. 사용자 확인됨 (이게 없으면 sim 에서 로봇 안 움직임).
run_launch "velocity_modifier" "$FULL_SETUP; ros2 launch velocity_modifier velocity_modifier.launch.py"
sleep 2


# =====================================================================
# === 2. CANDIDATES — sim 에서 쓸 가능성 있음, 검토 후 결정 ===
# (아래는 모두 주석 처리 — 필요한 것만 # 떼고 사용)
# =====================================================================

# [후보] monitoring — 시스템 / 노드 상태 모니터링
# run_launch "monitoring" "$FULL_SETUP; ros2 launch monitoring monitoring_launch.py"
# sleep 2

# [후보] nav_safety_manager — nav 동작 중 안전 관리. sim 에서도 의미 있을 수 있음
# run_launch "nav_safety_manager" "$FULL_SETUP; ros2 launch nav_safety_manager nav_safety_manager.launch.py"
# sleep 1

# [후보] robot_status_manager — 로봇 상태 통합 매니저
run_launch "robot_status_manager" "$FULL_SETUP; ros2 launch robot_status_manager robot_status_manager.launch.py"
sleep 1

# [후보] replan_monitor — path validator. 주행 중 path 유효성 재검증
# Sim 환경에선 use_sim_time:=True 필수 (gz clock 으로 TF 조회). 안 주면
# wall-clock 으로 조회하다 sim-time TF buffer 와 어긋나서 "Lookup would
# require extrapolation into the future" 경고 무한 반복.
run_launch "replan_monitor" "$FULL_SETUP; ros2 launch replan_monitor run_path_validator.launch.py use_sim_time:=True"
sleep 1

# [후보] multi_agent_behavior — fleet decision. single-robot sim 이면 무의미할 수 있음
run_launch "multi_agent_behavior" "$FULL_SETUP; ros2 launch multi_agent_behavior run_fleet_decision.launch.py"
sleep 1

# [후보] navigation_manager — nav 동작 관리 (Python helper, tests/ 디렉토리)
run_launch "navigation_manager" "$FULL_SETUP; python3 ${WS_DIR}/src/amhs/tests/navigation_manager/navigation_manager.py"
sleep 1

# [후보] nav_stuck_manager — 정지/스턱 상태 복구
run_launch "nav_stuck_manager" "$FULL_SETUP; python3 ${WS_DIR}/src/amhs/tests/navigation_manager/nav_stuck_manager.py"
sleep 1

# [후보] param_manager — 런타임 파라미터 관리 helper
run_launch "param_manager" "$FULL_SETUP; python3 ${WS_DIR}/src/amhs/tests/param_manager/param_manager.py"
sleep 1


# =====================================================================
# === 3. REAL-ROBOT ONLY — sim 에서 불필요 (이유 명시) ===
# (모두 주석 처리. 필요 없는 것 확인용)
# =====================================================================

# [실물 외부 통신] winros_bridge — Windows fleet manager 와 통신
# run_launch "winros_bridge" "$FULL_SETUP; ros2 launch winros_bridge winros_bridge_launch.py"




echo "---------------------------------------------------"
echo "All ACTIVE launches dispatched. (TERMINAL_EMU=${TERMINAL_EMU})"
case "$TERMINAL_EMU" in
    tmux)        echo "View: tmux attach -t sim  |  cleanup: tmux kill-session -t sim" ;;
    background)  echo "Logs: ls -la /tmp/sim_*.log  |  tail -f /tmp/sim_gz_amr_bringup.log" ;;
esac
echo "Cleanup all: pkill -9 -f 'ros2|gz sim|nav2_|amcl|map_server|filter_mask|costmap_filter|opennav|robot_state_publisher|three_scan_merger|parameter_bridge|rviz2|velocity_modifier'"
echo "---------------------------------------------------"
