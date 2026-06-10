#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""move_command 롱런(long-run) 주행 테스트 스크립트.

navigation_manager.py(`/move_command` 를 sub 해서 nav2 action client 역할을
하는 최상위 노드) 에게 `/move_command` (navigation_command_msgs/NavigationCommand)
를 순차적으로 publish 해서, 미리 정의한 여러 "구간(segment)" 을 반복 주행시킨다.

동작 흐름
---------
1. 아래 WAYPOINTS / ROUTE 를 한 구간씩 NavigationCommand 로 만들어 publish.
2. `/ros2_nav2_monitoring_data` (NavigationMonitoring) 를 sub 해서 그 구간이
   "실제로 시작됐는지 → 도착(success) 또는 중단(abort)" 됐는지 판정.
3. 한 구간이 끝나면 다음 구간으로. ROUTE 끝까지 가면 1 loop 완료.
4. --loops 만큼(0=무한) 반복.

완료 판정 (stale 값 회피의 핵심)
--------------------------------
navigation_manager 는 monitoring 데이터를 10Hz 로 계속 publish 하고,
is_destination_reached / driving_abort 같은 플래그는 다음 명령 전까지 이전 값이
남아있을 수 있다. 그래서 매 구간마다 cmd_seq_num 을 1 씩 바꿔 보내고,
monitoring 의 ros_nav_cmd_seq_num 이 "이번에 보낸 seq" 와 일치할 때만
유효한 상태로 인정한다(= 이번 goal 이 manager 에서 accept 된 뒤의 신호).

  Phase 1 (시작 확인): cmd_seq==our_seq AND (driving or acvtivation)
  Phase 2 (종료 대기): cmd_seq==our_seq AND is_destination_reached  -> success
                       cmd_seq==our_seq AND driving_abort           -> abort

사용법
------
  source /opt/ros/jazzy/setup.bash
  source /root/work_ws/install/setup.bash
  # (mobile_robot_sim.sh 로 sim + navigation_manager 가 떠 있는 상태)

  python3 move_command_longrun_test.py                 # 무한 반복
  python3 move_command_longrun_test.py --loops 5       # 5 바퀴
  python3 move_command_longrun_test.py --on-abort retry # 중단 시 같은 구간 재시도

  # 좌표 확인이 안 됐으면 먼저 driver 없이 메시지만 보고 싶을 때:
  python3 move_command_longrun_test.py --dry-run

좌표 얻는 법
------------
WAYPOINTS 의 (x, y, yaw_deg) 는 map 좌표계 기준이다. 실제 값은:
  - RViz "2D Goal Pose" / "Publish Point" 로 찍고
    `ros2 topic echo /goal_pose` 또는 `ros2 topic echo /clicked_point`
  - 또는 로봇을 원하는 위치로 보낸 뒤 `ros2 topic echo /amcl_pose`
로 읽어서 아래 WAYPOINTS 를 채워 넣으면 된다.
"""

import argparse
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Pose
from std_msgs.msg import UInt8

from navigation_command_msgs.msg import NavigationCommand
from navigation_monitoring_msgs.msg import NavigationMonitoring


# ====================================================================== #
# 1. 주행 구간 정의  ───  여기만 고치면 됨
# ====================================================================== #
# 이름 -> (x, y, yaw_deg). map 좌표계(meter, degree).
# 아래 값은 예시 placeholder 다. depot 맵 / 실제 환경에 맞게 반드시 교체할 것.
WAYPOINTS: Dict[str, Tuple[float, float, float]] = {
    "P1": (0.0,  0.0,   0.0),
    "P2": (5.0,  0.0,   0.0),
    "P3": (5.0,  5.0,  90.0),
    "P4": (0.0,  5.0, 180.0),
}

# 각 구간(segment) = waypoint 이름 리스트.
# 리스트에 여러 개를 넣으면 NavigateThroughPoses 처럼 그 점들을 연속 경유한다.
# ROUTE 전체를 1 loop 으로 보고 반복한다.
ROUTE: List[List[str]] = [
    ["P2"],        # 구간 1: P2 로
    ["P3", "P4"],        # 구간 2: P3 로
    ["P1", "P2"],        # 구간 3: P4 로
    ["P1"],        # 구간 4: 출발점 복귀
]


# ====================================================================== #
# 2. 토픽 / 기본 타이밍
# ====================================================================== #
MOVE_COMMAND_TOPIC = "/move_command"
MONITORING_TOPIC = "/ros2_nav2_monitoring_data"
STOP_COMMAND_TOPIC = "/stop_command"
FRAME_ID = "map"


@dataclass
class MonitorState:
    """monitoring 콜백이 채우는 최신 스냅샷(락 보호)."""
    cmd_seq: int = -1
    driving: bool = False
    activation: bool = False
    destination_reached: bool = False
    driving_abort: bool = False
    distance_remaining: float = 0.0
    poses_remaining: int = 0
    current_node: int = 0
    next_node: int = 0
    stamp: float = 0.0  # 마지막 수신 시각(monotonic)


def yaw_deg_to_quat(yaw_deg: float) -> Tuple[float, float]:
    """yaw(도) -> (z, w) quaternion. roll=pitch=0 가정."""
    half = math.radians(yaw_deg) * 0.5
    return math.sin(half), math.cos(half)


def make_pose(x: float, y: float, yaw_deg: float) -> Pose:
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = 0.0
    qz, qw = yaw_deg_to_quat(yaw_deg)
    p.orientation.z = qz
    p.orientation.w = qw
    return p


class MoveCommandLongRun(Node):
    def __init__(self) -> None:
        super().__init__("move_command_longrun_test")

        self._lock = threading.Lock()
        self._state = MonitorState()

        self._cmd_pub = self.create_publisher(
            NavigationCommand, MOVE_COMMAND_TOPIC, 10)
        self._stop_pub = self.create_publisher(
            UInt8, STOP_COMMAND_TOPIC, 10)
        self._mon_sub = self.create_subscription(
            NavigationMonitoring, MONITORING_TOPIC,
            self._on_monitoring, 10)

        # 노드 id 자동 부여(monitoring 표시에만 쓰임, 주행에 영향 없음).
        self._wp_ids: Dict[str, int] = {
            name: idx + 1 for idx, name in enumerate(WAYPOINTS.keys())
        }
        self._current_node_id: int = 0  # 마지막으로 도달한 노드(시작 0)
        self._seq: int = 0              # uint8, 1..255 순환

    # ------------------------------------------------------------------ #
    # monitoring sub
    # ------------------------------------------------------------------ #
    def _on_monitoring(self, msg: NavigationMonitoring) -> None:
        with self._lock:
            self._state = MonitorState(
                cmd_seq=int(msg.ros_nav_cmd_seq_num),
                driving=bool(msg.ros_nav_driving),
                activation=bool(msg.ros_nav_acvtivation),
                destination_reached=bool(msg.ros_nav_is_destination_reached),
                driving_abort=bool(msg.ros_nav_driving_abort),
                distance_remaining=float(msg.ros_nav_distance_remaining),
                poses_remaining=int(msg.ros_nav_number_of_poses_remaining),
                current_node=int(msg.ros_nav_current_node_id),
                next_node=int(msg.ros_nav_next_node_id),
                stamp=time.monotonic(),
            )

    def snapshot(self) -> MonitorState:
        with self._lock:
            return self._state

    def monitoring_alive(self, max_age: float = 3.0) -> bool:
        s = self.snapshot()
        return s.stamp > 0.0 and (time.monotonic() - s.stamp) < max_age

    # ------------------------------------------------------------------ #
    # command 생성 / 전송
    # ------------------------------------------------------------------ #
    def _next_seq(self) -> int:
        # 1..255 순환 (0 은 "명령 없음" 으로 오해될 수 있어 회피).
        self._seq = self._seq % 255 + 1
        return self._seq

    def build_command(self, wp_names: List[str], seq: int) -> NavigationCommand:
        cmd = NavigationCommand()
        cmd.cmd_seq_num = seq
        cmd.goal_cnt = len(wp_names)

        prev_id = self._current_node_id
        for name in wp_names:
            x, y, yaw = WAYPOINTS[name]
            wp_id = self._wp_ids[name]
            cmd.goal_poses.append(make_pose(x, y, yaw))
            cmd.from_node_id.append(prev_id)
            cmd.to_node_id.append(wp_id)
            prev_id = wp_id
        return cmd

    def publish_command(self, cmd: NavigationCommand) -> None:
        self._cmd_pub.publish(cmd)

    def send_stop(self) -> None:
        msg = UInt8()
        msg.data = 0
        self._stop_pub.publish(msg)

    def mark_reached(self, wp_names: List[str]) -> None:
        self._current_node_id = self._wp_ids[wp_names[-1]]


# ====================================================================== #
# 3. 한 구간 주행 + 판정
# ====================================================================== #
def drive_segment(node: MoveCommandLongRun, wp_names: List[str],
                  start_timeout: float, goal_timeout: float,
                  poll: float = 0.1) -> str:
    """한 구간을 보내고 결과 문자열 반환: success / abort / start_timeout / goal_timeout."""
    seq = node._next_seq()
    cmd = node.build_command(wp_names, seq)
    node.publish_command(cmd)

    coords = " -> ".join(
        f"{n}({WAYPOINTS[n][0]:.1f},{WAYPOINTS[n][1]:.1f})" for n in wp_names)
    node.get_logger().info(f"[seq={seq}] move_command 전송: {coords}")

    # Phase 1: 이 seq 의 goal 이 실제로 시작(accept→driving/activation)될 때까지.
    t0 = time.monotonic()
    started = False
    while rclpy.ok() and (time.monotonic() - t0) < start_timeout:
        s = node.snapshot()
        if s.cmd_seq == seq and (s.driving or s.activation):
            started = True
            break
        time.sleep(poll)
    if not started:
        return "start_timeout"

    node.get_logger().info(f"[seq={seq}] 주행 시작 확인. 도착 대기...")

    # Phase 2: 도착(success) 또는 중단(abort) 까지.
    t1 = time.monotonic()
    last_log = 0.0
    while rclpy.ok() and (time.monotonic() - t1) < goal_timeout:
        s = node.snapshot()
        if s.cmd_seq == seq:
            if s.destination_reached:
                return "success"
            if s.driving_abort:
                return "abort"
        now = time.monotonic()
        if now - last_log > 2.0:
            node.get_logger().info(
                f"[seq={seq}] 주행중 dist={s.distance_remaining:.2f}m "
                f"remain={s.poses_remaining} "
                f"node {s.current_node}->{s.next_node}")
            last_log = now
        time.sleep(poll)
    return "goal_timeout"


def run(node: MoveCommandLongRun, args) -> None:
    # navigation_manager 의 monitoring 이 올라올 때까지 잠깐 대기.
    node.get_logger().info(f"{MONITORING_TOPIC} 대기중...")
    wait0 = time.monotonic()
    while rclpy.ok() and not node.monitoring_alive():
        if time.monotonic() - wait0 > 30.0:
            node.get_logger().warn(
                "monitoring 데이터가 안 들어옴. navigation_manager 가 떠 있는지 "
                "확인. 그래도 명령은 시도한다.")
            break
        time.sleep(0.2)

    total_segments = 0
    ok_segments = 0
    loop_idx = 0

    while rclpy.ok():
        loop_idx += 1
        if args.loops > 0 and loop_idx > args.loops:
            break
        node.get_logger().info(
            f"==================== LOOP {loop_idx}"
            f"{'/' + str(args.loops) if args.loops > 0 else ' (무한)'} "
            f"====================")

        for seg_idx, wp_names in enumerate(ROUTE, start=1):
            if not rclpy.ok():
                break
            attempt = 0
            while rclpy.ok():
                attempt += 1
                total_segments += 1
                result = drive_segment(
                    node, wp_names, args.start_timeout, args.goal_timeout)

                tag = f"loop {loop_idx} / seg {seg_idx} ({'>'.join(wp_names)})"
                if result == "success":
                    ok_segments += 1
                    node.mark_reached(wp_names)
                    node.get_logger().info(f"[OK] {tag} 도착")
                    break

                node.get_logger().warn(
                    f"[FAIL:{result}] {tag} (attempt {attempt})")

                if args.on_abort == "retry":
                    node.get_logger().info(f"{args.pause_between:.1f}s 후 재시도")
                    time.sleep(args.pause_between)
                    continue
                elif args.on_abort == "skip":
                    break
                else:  # stop
                    node.get_logger().error("on-abort=stop -> 테스트 종료")
                    _summary(node, loop_idx, total_segments, ok_segments)
                    return

            if args.pause_between > 0:
                time.sleep(args.pause_between)

    _summary(node, loop_idx, total_segments, ok_segments)


def _summary(node, loops, total, ok) -> None:
    node.get_logger().info(
        f"======== 종료: loops={loops} segments={total} "
        f"success={ok} fail={total - ok} ========")


# ====================================================================== #
# 4. main
# ====================================================================== #
def parse_args():
    p = argparse.ArgumentParser(
        description="move_command 롱런 주행 테스트")
    p.add_argument("--loops", type=int, default=0,
                   help="ROUTE 반복 횟수 (0=무한, 기본 0)")
    p.add_argument("--start-timeout", type=float, default=160.0,
                   help="goal 이 실제 시작될 때까지 최대 대기(s). "
                        "navigation_manager 내부 wait(150s) 보다 크게. 기본 160")
    p.add_argument("--goal-timeout", type=float, default=1800.0,
                   help="한 구간 도착까지 최대 대기(s). 기본 1800 (30분)")
    p.add_argument("--pause-between", type=float, default=1.0,
                   help="구간 사이 대기(s). 기본 1.0")
    p.add_argument("--on-abort", choices=["retry", "skip", "stop"],
                   default="skip",
                   help="구간 실패(abort/timeout) 시 동작. 기본 skip")
    p.add_argument("--dry-run", action="store_true",
                   help="실제 publish 없이 각 구간 NavigationCommand 만 출력")
    return p.parse_args()


def dry_run() -> None:
    print("=== DRY-RUN: 전송될 구간 목록 ===")
    wp_ids = {n: i + 1 for i, n in enumerate(WAYPOINTS.keys())}
    prev = 0
    for seg_idx, wp_names in enumerate(ROUTE, start=1):
        parts = []
        for n in wp_names:
            x, y, yaw = WAYPOINTS[n]
            qz, qw = yaw_deg_to_quat(yaw)
            parts.append(
                f"{n} from={prev} to={wp_ids[n]} "
                f"pos=({x:.2f},{y:.2f}) quat(z={qz:.3f},w={qw:.3f})")
            prev = wp_ids[n]
        print(f"  seg {seg_idx}: goal_cnt={len(wp_names)} | " + " ; ".join(parts))
    print(f"총 {len(ROUTE)} 구간 / loop")


def main() -> None:
    args = parse_args()

    if args.dry_run:
        dry_run()
        return

    rclpy.init()
    node = MoveCommandLongRun()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        run(node, args)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C -> stop_command 전송 후 종료")
        try:
            node.send_stop()
            time.sleep(0.3)
        except Exception:
            pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
