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
