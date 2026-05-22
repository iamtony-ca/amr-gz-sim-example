# mobile_robot_gz_sim

nav 스택을 **Gazebo (gz sim Harmonic)** 환경에서 그대로 돌리기 위한 패키지. 실물에서 쓰는 `amr_bringup` / `nav2_bringup` launch 파일들을 **수정 없이 재사용** 하는 게 핵심 목표 — sim 전용 인프라 (gz 월드, robot spawn, ros_gz_bridge, scan merger) + AMCL 로 대체하는 것만 sim 측에서 따로 처리.

---

## 1. 아키텍처



### Sim launch 체인

```
mobile_robot_gz_sim/launch/gz_amr_bringup.launch.py
  ├─ gz sim 월드 + 로봇 spawn + ros_gz_bridge + three_scan_merger  (sim 전용)
  ├─ nav2_bringup/launch/localization_launch.py (AMCL + map_server)  
  └─ amr_bringup/run_amr_bringup.launch.py (use_localization=False)
       └─ bringup_launch.py
            ├─ (localization 스킵 — sim 은 nav2_bringup AMCL 이 대체)
            ├─ amr_bringup/costmap_filter.launch.py        (AS-IS)
            └─ amr_bringup/navigation_launch.py            (AS-IS)
```


---

## 2. 사전 준비 (Prerequisites)

### 시스템

- Ubuntu 24.04 (또는 ROS 2 Jazzy 가 도는 환경)
- **ROS 2 Jazzy**
- **gz sim Harmonic** (`gz sim --versions` 으로 확인)
- **xacro** (URDF / 월드 SDF 빌드용)

### Docker container 사용 시 추가

X11 forwarding 이 잘 되어있어야 gz GUI / RViz 가 호스트에 뜸 (`DISPLAY=:0` 같은 환경변수).

multi-terminal launch script 를 GUI 터미널로 쓰려면:

```bash
apt install -y xterm
# 또는 헤드리스 환경이면
apt install -y tmux
```

xterm/gnome-terminal 도 없고 tmux 도 없으면 스크립트가 자동으로 background 모드로 fallback (각 launch 의 로그는 `/tmp/sim_<name>.log`).



### 단일 패키지 재빌드

소스만 바뀌었을 때:

```bash
colcon build --symlink-install --packages-select mobile_robot_gz_sim amr_bringup
```


---

## 4. 실행 (Quick Start)

### 환경 변수

```bash
source /opt/ros/jazzy/setup.bash
source /root/work_ws/install/setup.bash
# export ROS_DOMAIN_ID=7
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# CYCLONEDDS_URI 는 README.md 의 .bashrc 블록 참고 (선택)
```

### 최소 실행 — gz_amr_bringup 1개

```bash
ros2 launch mobile_robot_gz_sim gz_amr_bringup.launch.py \
    headless:=False use_rviz:=True
```

> 이것만 띄우면 nav 스택이 활성화는 되지만 **로봇이 안 움직임** (cmd_vel pipeline 불완전). 별도 터미널에서 `velocity_modifier` 도 띄워야 주행 가능:

```bash
ros2 launch velocity_modifier velocity_modifier.launch.py
```

### 권장 — multi-terminal 스크립트

다중 터미널 구조로 sim 전체 스택을 한 번에:

```bash
bash /root/work_ws/src/mobile_robot/mobile_robot_gz_sim/scripts/mobile_robot_sim.sh
```

자동 처리되는 것:
- 사용 가능한 터미널 자동 감지 (gnome-terminal / xterm / konsole / … / tmux / background)
- `gz_amr_bringup` (gz + nav 스택), `velocity_modifier`, 그 외 CANDIDATES 
- 각 launch 가 별도 터미널 / tmux window 에서 실행

### gz_amr_bringup 의 Launch 인자

| arg | default | 설명 |
|---|---|---|
| `headless` | `True` | False 면 gz sim GUI 표시 |
| `use_rviz` | `True` | RViz2 띄울지 |
| `map` | `<pkg>/maps/depot.yaml` | map yaml 절대경로 (AMCL + map_server). **CLI override 가 yaml 내용보다 우선** (gz_amr_bringup 의 RewrittenYaml 처리 덕분) |
| `filter_mask_file` | `<pkg>/maps/depot.yaml` | costmap 필터 마스크 yaml (placeholder = map 자체) |
| `use_filters` | `True` | costmap filter 스택 켤지 |
| `params_file_source` | `<pkg>/params/nav2_params_gz_static_filter.yaml` | sim 의 nav2 params YAML. filter2.yaml 등으로 override 가능 |
| `x_pose / y_pose / z_pose / yaw` | `5.86 / 0.17 / 0.12 / 0.0` | gz 로봇 spawn pose. **AMCL initial_pose 와 일치 필요** (params 의 amcl 블록) |

예시 — filter2.yaml + 헤드리스:

```bash
ros2 launch mobile_robot_gz_sim gz_amr_bringup.launch.py \
    headless:=True use_rviz:=False \
    params_file_source:=/root/work_ws/src/mobile_robot/mobile_robot_gz_sim/params/nav2_params_gz_static_filter2.yaml
```

### mobile_robot_sim.sh 의 env 변수

```bash
PARAMS_YAML=<path>      # default: nav2_params_gz_static_filter.yaml
MAP_YAML=<path>         # default: depot.yaml
FILTER_MASK_YAML=<path> # default: depot.yaml
HEADLESS=False
USE_RVIZ=True
TERMINAL_EMU=xterm      # 강제 지정 (auto detect 무시)
```

---

## 5. 사용법

### tmux 모드 (헤드리스 / Docker 컨테이너 안에서)

```bash
TERMINAL_EMU=tmux bash /root/work_ws/src/mobile_robot/mobile_robot_gz_sim/scripts/mobile_robot_sim.sh
tmux attach -t sim
# Ctrl+b → 0~7    : window 직접 이동
# Ctrl+b → n / p  : 다음 / 이전 window
# Ctrl+b → w      : window 목록
# Ctrl+b → [      : scroll 모드 (방향키, q 로 나가기)
# Ctrl+b → d      : detach (sim 계속 동작)
# tmux kill-session -t sim   : 전체 종료
```

### Background 모드 (터미널 emulator 도 tmux 도 없을 때)

```bash
TERMINAL_EMU=background bash mobile_robot_sim.sh
# 로그 파일
ls -la /tmp/sim_*.log
tail -f /tmp/sim_gz_amr_bringup.log
```

### Nav goal 보내기 (CLI)

```bash
source /opt/ros/jazzy/setup.bash
source /root/work_ws/install/setup.bash


# spawn (5.86, 0.17) → +X 방향 ~1m
ros2 action send_goal /navigate_through_poses \
    nav2_msgs/action/NavigateThroughPoses \
    '{poses: [{header: {frame_id: "map"}, pose: {position: {x: 7.0, y: 0.17, z: 0.0}, orientation: {w: 1.0}}}]}'
```

> RViz 의 *2D Goal Pose* 툴은 기본적으로 `/goal_pose` → `NavigateToPose` 액션을 호출. `/navigate_through_poses` (실물 BT 가 사용) 를 검증하려면 위 CLI 명령 사용.

---

## 6. 활성화 검증 (정상 동작 확인)

launch 로그에서 다음 4줄이 모두 떠야 함:

```
[lifecycle_manager_localization]:        Managed nodes are active
[lifecycle_manager_costmap_filters]:     Managed nodes are active
[static_nav.lifecycle_manager_static]:   Managed nodes are active
[lifecycle_manager_navigation]:          Managed nodes are active
```

타이밍 (gz_amr_bringup 내부 TimerAction):
- t+0: gz 월드 spawn, robot, bridge, scan_merger
- t+10s: localization + costmap_filter 시작
- t+16s: navigation 시작
- ~t+25-30s: 4/4 모두 active 완료

런타임 점검:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /static_nav/static_planner_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /filter_mask_server
ros2 lifecycle get /costmap_filter_info_server

ros2 topic hz /scan                  # ~12 Hz
ros2 topic echo --once /amcl_pose
ros2 run tf2_ros tf2_echo map base_link
ros2 param get /map_server yaml_filename   # 실제로 로드된 map yaml
```

---

## 7. Sim 정리 (Cleanup)

여러 번 launch 하면 **DDS participant table (domain 21) 이 포화되어 다음 launch 가 RCLError 로 실패** 함. Ctrl+C 만으론 다 안 죽는 경우 많음.

```bash
# tmux 세션 종료 (있으면)
tmux kill-session -t sim 2>/dev/null

# 모든 sim 노드 강제 종료
pkill -9 -f "ros2|gz sim|nav2_|amcl|map_server|filter_mask|costmap_filter|opennav|robot_state_publisher|three_scan_merger|parameter_bridge|rviz2|velocity_modifier"

# 확인 (0 이어야 함)
pgrep -af "gz sim|nav2_|amcl|map_server" | wc -l
```

`/scripts/.../kill_all.sh` 도 쓸 수 있음 (실물 cleanup 용이지만 sim 에도 호환).

---

## 8. 파일 구성

```
mobile_robot_gz_sim/
├── launch/
│   ├── gz_amr_bringup.launch.py             ★ canonical sim entry (Option B/C/D 적용)
│   ├── spawn_robot.launch.py                 helper (gz_amr_bringup 가 사용)
│   ├── simulation.launch.py                  legacy single-planner 변형 (fallback)
│   ├── simulation_static.launch.py           legacy two-planner 변형 (fallback)
│   ├── simulation_static_filter.launch.py    legacy filter 포함 변형 (fallback)
│   ├── run_simulation_static_filter.launch.py  legacy run wrapper (fallback)
│   ├── navigation_launch_gz.py               legacy 중복 nav launch (gz_amr_bringup 가 amr_bringup 의 것을 그대로 씀)
│   └── navigation_launch_gz_static.py        legacy 중복 nav launch (위와 동일)
├── params/
│   ├── nav2_params_gz_static_filter.yaml    ★ 주 sim params (filter 포함, 2-planner)
│   ├── nav2_params_gz_static_filter2.yaml    대안 (MPPI 등 변형)
│   ├── nav2_params_gz_static.yaml            legacy
│   ├── nav2_params_gz.yaml                   legacy
│   └── nav2_params.yaml                      참고용
├── urdf/
│   └── mobile_robot_gz.urdf.xacro            gz용 URDF (gz-sim-pose-publisher-system 진단용 포함)
├── worlds/
│   └── depot.sdf                              gz 월드 (xacro)
├── maps/
│   ├── depot.yaml / depot_edit.pgm           sim 맵
├── models/
│   └── map/                                   STL 형태의 월드 메쉬 (scripts/gen_map_stl.py 로 생성)
├── behavior_trees/                            BT XML (옵션)
├── scripts/
│   ├── mobile_robot_sim.sh                   ★ multi-terminal sim launch script
│   ├── gen_map_stl.py                         depot.yaml → models/map/meshes/map.stl 변환기
│   └── README.md                              scripts/gen_map_stl.py 사용 가이드
└── readme.md
```

> `★` 가 권장 사용 entry. legacy 파일들은 옛 sim 셋업 시절의 fallback 으로 남겨둠 — `gz_amr_bringup.launch.py` 가 작동하는 한 굳이 안 써도 됨.

---


## 12. 새 맵 (map2.yaml + map2.pgm) 적용

실물 로봇에서 새로 따낸 `map2.yaml` + `map2.pgm` 을 sim 환경에서 그대로 쓰는 절차.

> **핵심**: Nav2 가 쓰는 맵 (`yaml` + `pgm`) 과 gz 의 물리/센서 월드 (`models/map/meshes/map.stl`) 가 **따로 놀면 sim 에서 로봇이 보이지 않는 벽에 부딪힘 / 통과함**. 두 개를 같이 맞춰야 함. (`worlds/depot.sdf` 가 `<uri>models://map</uri>` 로 STL 을 참조하므로, **STL 만 재생성하면 SDF 는 그대로 둬도 됨**.)

### Step 1 — 새 맵 파일 배치

```bash
cp /path/to/real/map2.yaml /path/to/real/map2.pgm \
   /root/work_ws/src/mobile_robot/mobile_robot_gz_sim/maps/
```

`map2.yaml` 의 `image:` 필드가 `map2.pgm` 을 가리키는지 확인 (실물에서 그대로 가져오면 보통 OK).

### Step 2 — gz 월드 STL 재생성 (필수)

```bash
cd /root/work_ws/src/mobile_robot/mobile_robot_gz_sim/scripts
python3 gen_map_stl.py ../maps/map2.yaml ../models/map/meshes/map.stl
```

`gen_map_stl.py` 가 `map2.pgm` 의 occupancy 픽셀을 읽어 gz 물리/센서 충돌 메쉬를 `models/map/meshes/map.stl` 로 덮어씀. SDF 의 `<uri>models://map</uri>` 가 그대로 새 STL 을 가리키므로 SDF 수정 불필요.

### Step 3 — params YAML 의 AMCL `initial_pose` 조정

`mobile_robot_gz_sim/params/nav2_params_gz_static_filter.yaml` (또는 `..._filter2.yaml`) 의 `amcl.initial_pose_*` 항목을 새 맵에서 로봇이 시작할 좌표로 변경. 안 맞추면 AMCL 이 엉뚱한 곳에서 시작해서 `map ↔ odom` TF 가 한참 흔들리고 particle 이 발산함.

### Step 4 — spawn pose 도 같은 값으로

gz 의 로봇 spawn pose 와 AMCL `initial_pose` 는 **반드시 일치** 해야 함. `gz_amr_bringup.launch.py` 의 현재 기본값은 `x_pose=5.86, y_pose=0.17, z_pose=0.12, yaw=0.0` (depot 맵 기준). 둘 중 한 방법:

**(a)** launch arg 로 매번 override:

```bash
ros2 launch mobile_robot_gz_sim gz_amr_bringup.launch.py \
    map:=/root/work_ws/src/mobile_robot/mobile_robot_gz_sim/maps/map2.yaml \
    x_pose:=10.0 y_pose:=5.0 z_pose:=0.12 yaw:=0.0
```

**(b)** `gz_amr_bringup.launch.py` 의 `pose = {...}` 블록 기본값을 직접 수정 (map2 가 새 기본이 될 경우).

> `mobile_robot_sim.sh` 는 현재 `x_pose / y_pose / yaw` 를 env 로 노출하지 않음. spawn pose 가 자주 바뀐다면 (a) 패턴을 스크립트의 `gz_amr_bringup` run_launch 줄에 `x_pose:=${X_POSE}` 같은 식으로 추가해두면 편함.

### Step 5 — costmap filter mask (선택)

`mobile_robot_sim.sh` 의 `FILTER_MASK_YAML` 기본값은 placeholder (맵 yaml 자기 자신). 새 맵용 keepout / speed mask 가 있으면 같이 갱신, 없으면 `use_filters:=False` 로 끄거나 placeholder 그대로 둠 (실제 필터링은 안 되지만 노드는 정상 활성화).

### Step 6 — 실행

`mobile_robot_sim.sh` 경로:

```bash
MAP_YAML=/root/work_ws/src/mobile_robot/mobile_robot_gz_sim/maps/map2.yaml \
FILTER_MASK_YAML=/root/work_ws/src/mobile_robot/mobile_robot_gz_sim/maps/map2.yaml \
bash /root/work_ws/src/mobile_robot/mobile_robot_gz_sim/scripts/mobile_robot_sim.sh
```

직접 launch:

```bash
ros2 launch mobile_robot_gz_sim gz_amr_bringup.launch.py \
    headless:=False use_rviz:=True \
    map:=/root/work_ws/src/mobile_robot/mobile_robot_gz_sim/maps/map2.yaml \
    x_pose:=<X> y_pose:=<Y> yaw:=<Y>
```

### 검증

```bash
ros2 param get /map_server yaml_filename      # → map2.yaml 절대경로
ros2 topic echo --once /amcl_pose             # → 새 initial_pose 근처
ros2 run tf2_ros tf2_echo map base_link       # → 발산 없이 안정적
ros2 topic hz /scan                           # → ~12 Hz
```

gz GUI (`HEADLESS=False`) 에서 로봇이 벽 속에 박혀 있거나 통과해버리면 **Step 2 의 STL 재생성을 빠뜨린 것**.

### Pitfall 요약

| 증상 | 원인 |
|---|---|
| gz 에서 로봇이 벽 통과 / 박힘 | STL 재생성 (Step 2) 누락 |
| `[amcl] No laser scan received` 또는 particle 발산 | initial_pose 와 spawn pose 불일치 (Step 3, 4) |
| `Waiting for map...` 무한 | `map:=` arg 가 안 먹힘 — `ros2 param get /map_server yaml_filename` 으로 확인 |
| 필터 마스크 lifecycle 만 활성화 안 됨 | `filter_mask_file:=` 가 존재하지 않는 경로 |

---

## 13. gz sim 시간 가속 (테스트 빠르게)

긴 주행 시나리오 검증 시 sim 을 wall clock 보다 빠르게 돌리는 방법.

### 방법 1 — SDF 영구 수정 (재시작 필요)

`mobile_robot_gz_sim/worlds/depot.sdf` 의 physics 블록:

```xml
<physics name="1ms" type="ode">
  <max_step_size>0.003</max_step_size>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <real_time_factor>1.0</real_time_factor>     <!-- ← 이 값 -->
</physics>
```

- **`<real_time_factor>` 를 N 으로 변경**: `3.0` → 3배속, `0` → 머신이 낼 수 있는 최대 속도. 가장 간단.
- 더 확실하게 가속하려면 `<real_time_update_rate>` 도 비례해서 올리기 (예: 3배속이면 `3000.0`). 실효 RTF 캡 = `max_step_size × real_time_update_rate` 이라 둘 다 맞춰주는 게 정석.
- 수정 후 `gz_amr_bringup` 재실행 필요 (gz sim 재시작).

### 방법 2 — GUI 슬라이더 (런타임)

`headless:=False` 로 띄운 경우, gz GUI 우상단 톱니바퀴 → "Physics" → **Real Time Factor** 입력 박스에서 값 변경. 즉시 반영, 재시작 불필요. 시나리오 중간에 잠깐만 가속하고 싶을 때 편함.

### 주의사항

- 너무 올리면 ROS 측 nav 노드 (controller, BT, AMCL, costmap) 가 sim time 을 못 따라잡아 **TF lookup 실패 / AMCL particle 발산 / `cmd_vel` 끊김 / `Lookup would require extrapolation`** 워닝 발생. 보통 **2–3배가 안전선**, 머신 사양에 따라 다름.
- `replan_monitor` 등 `use_sim_time:=True` 로 띄우는 노드들은 sim clock 기준으로 동작하므로 가속해도 정상. wall-clock 기반 노드 (sim 외 코드) 는 영향 안 받음.
- **실물 거동 정밀 검증용이면 RTF=1.0 유지**. controller / dynamics 가 wall clock 에서 검증된 게이트라 가속 상태에서 통과해도 실물에서 같으리란 보장은 없음.
- gz GUI 좌하단의 실제 RTF 표시값 (예: `RTF 2.85`) 이 설정값보다 낮게 떠 있으면 머신 성능 한계 — 그 이상은 안 빨라짐.

---

## 14. 참고 파일

- 워크스페이스 전반: `/root/work_ws/src/CLAUDE.md`
- Sim multi-terminal 스크립트: `mobile_robot_gz_sim/scripts/mobile_robot_sim.sh`
- gz 월드 STL 생성기: `mobile_robot_gz_sim/scripts/gen_map_stl.py` (map yaml ↔ gz mesh 동기화)
