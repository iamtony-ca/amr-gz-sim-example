# scripts — 맵 유지보수 도구

## gen_map_stl.py — gz 월드 메쉬(map.stl) 재생성

### 핵심 원칙

**Nav2 맵(`maps/depot.yaml` + `maps/depot_edit.pgm`)이 유일한 기준(source of truth)입니다.**
AMCL · costmap · planner가 전부 이 pgm을 사용합니다.
`models/map/meshes/map.stl`은 gz 물리/센서용 *파생물*일 뿐입니다.

→ 둘이 어긋나면 **항상 STL을 pgm으로부터 재생성**합니다.
STL을 손으로 스케일하거나 따로 수정하지 마세요.

### 언제 쓰나

| 상황 | 조치 |
|---|---|
| `depot_edit.pgm`를 GIMP 등으로 편집함 | STL 재생성 |
| 새 맵(pgm + yaml)으로 교체함 | STL 재생성 |
| RViz에서 scan이 주행거리에 비례해 벽에서 밀려남 | 스케일 불일치 — STL 재생성 |
| scan과 벽이 일정량 어긋남(거리 무관) | origin 불일치 — yaml의 `origin` 확인 후 재생성 |

### 진단 — 불일치 확인

STL 외곽 크기와 pgm 크기(`픽셀수 × resolution`)를 비교합니다:

```bash
python3 - <<'EOF'
import struct
f=open('models/map/meshes/map.stl','rb'); f.read(80)
n=struct.unpack('<I',f.read(4))[0]; mn=[1e9]*3; mx=[-1e9]*3
for _ in range(n):
    f.read(12)
    for _ in range(3):
        v=struct.unpack('<3f',f.read(12))
        for k in range(3): mn[k]=min(mn[k],v[k]); mx[k]=max(mx[k],v[k])
    f.read(2)
print("STL extent X=%.3f Y=%.3f"%(mx[0]-mn[0], mx[1]-mn[1]))
EOF
```

`depot.yaml`이 `604x307 @ 0.05`이면 기대값은 `30.20 x 15.35 m`.
STL extent가 이와 다르면 불일치이며, **드리프트 % ≈ 스케일 불일치 %**입니다.

### 사용법

```bash
cd <mobile_robot_gz_sim 패키지 루트>
cp models/map/meshes/map.stl models/map/meshes/map.stl.bak   # 백업

python3 scripts/gen_map_stl.py
#  또는 명시적으로:
python3 scripts/gen_map_stl.py maps/depot.yaml models/map/meshes/map.stl
```

install이 symlink(`colcon build --symlink-install`)라 **재빌드 불필요** —
sim만 재기동하면 새 메쉬가 반영됩니다.

재기동 전에는 기존 프로세스를 모두 종료하세요(`gz sim`, `parameter_bridge`,
`robot_state_publisher`, nav 노드). 남은 `parameter_bridge`가 `/clock`을
중복 발행하면 sim 시간이 꼬입니다.

### 동작 방식 / 설계 의도

- `depot.yaml`에서 `resolution` · `origin` · `occupied_thresh` · `negate`를
  읽어, 별도 인자 없이 항상 yaml과 일치하는 STL을 만듭니다.
- 점유 픽셀마다 벽 셀 하나를 0~`WALL_HEIGHT`(2.0 m)로 압출합니다.
- 각 셀 박스를 nav2_amcl의 점유점 기준(`origin + index·resolution`)에
  **센터링**(`-resolution/2`)합니다. 이렇게 해야 라이다가 보는 벽면과
  AMCL의 likelihood field가 정렬되어 scan-match 편향이 생기지 않습니다.
- 비점유 이웃에 노출된 면만 출력해 삼각형 수를 줄입니다(≈47.6k).

### 참고

- 이 도구는 `map.stl`만 생성합니다. 로봇 모델(URDF)·월드(`worlds/depot.sdf`)는
  건드리지 않습니다.
- 반대 방향(STL이 먼저 있고 pgm이 없음)은 이 도구로 처리하지 않습니다 —
  드문 경우이며, `map.stl`을 점유격자로 변환하는 별도 절차가 필요합니다.



# longrun script
---

# move_command_longrun_test.py 사용 가이드

## 작성한 스크립트

* `amhs/mobile_robot/mobile_robot_gz_sim/scripts/move_command_longrun_test.py`

### 동작

`WAYPOINTS/ROUTE`에 정의한 구간들을 한 개씩 `/move_command`(`NavigationCommand`)로 publish $\rightarrow$ `/ros2_nav2_monitoring_data`로 도착/중단을 판정 $\rightarrow$ 다음 구간 $\rightarrow$ `ROUTE` 끝나면 1바퀴 $\rightarrow$ `--loops`만큼 반복(0=무한).

### 완료 판정의 핵심

구간마다 `cmd_seq_num`을 1씩 바꿔 보내고, monitoring의 `ros_nav_cmd_seq_num`이 그 seq와 일치할 때만 유효 상태로 인정합니다. `navigation_manager`가 플래그(`is_destination_reached`, `driving_abort`)를 다음 명령까지 계속 들고 있어서 이전 구간의 stale 값을 잘못 읽는 문제를 이걸로 막았습니다.

* **Phase 1:** `cmd_seq==seq` && (`driving` || `activation`) $\rightarrow$ 이 goal이 실제 시작됨 확인
* **Phase 2:** ... && `destination_reached` $\rightarrow$ success / ... && `driving_abort` $\rightarrow$ abort

---

## 사용법

```bash
source /opt/ros/jazzy/setup.bash && source /root/work_ws/install/setup.bash
# (mobile_robot_sim.sh 로 sim + navigation_manager 가 떠 있는 상태에서)

python3 move_command_longrun_test.py            # 무한 반복
python3 move_command_longrun_test.py --loops 5  # 5바퀴
python3 move_command_longrun_test.py --on-abort retry   # 실패 시 같은 구간 재시도
python3 move_command_longrun_test.py --dry-run  # publish 없이 구간만 출력

```

### 주요 옵션

* `--start-timeout`: 기본 160s (manager 내부 150s wait보다 크게 설정)
* `--goal-timeout`: 기본 180s
* `--pause-between`: 기본 1s
* `--on-abort`: `{retry,skip,stop}` (기본 skip)

---

## ⚠️ 반드시 고쳐야 할 것 — 좌표

지금 `WAYPOINTS`의 `(x, y, yaw_deg)`는 예시 placeholder입니다. depot 맵 실제 좌표로 교체하세요. 파일 상단 `WAYPOINTS` dict와 `ROUTE` 리스트만 수정하면 됩니다.

### 좌표 얻는 법 (스크립트 docstring에도 적어둠)

1. RViz "2D Goal Pose" 찍고 `ros2 topic echo /goal_pose`
2. 로봇을 원하는 위치로 보낸 뒤 `ros2 topic echo /amcl_pose`

> [!NOTE]
> * 한 구간에 여러 점을 넣으면(`["P3","P4"]`) `MapsThroughPoses`처럼 연속 경유합니다.
> * `from_node_id`/`to_node_id`는 monitoring 표시용으로 자동 부여되며 주행에는 영향 없습니다.
> 
>
