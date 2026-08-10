#!/usr/bin/env python3
"""Small Nav2 lifecycle watchdog for LimeSim/TurtleBot3 Lime.

The watchdog only intervenes after the stack has been unhealthy for several
consecutive probes. Before recovery it publishes a zero Twist, then asks the
Nav2 lifecycle managers to RESET and STARTUP their managed nodes.
"""

import argparse
import time
from typing import Dict, Iterable, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.action import ActionClient
from rclpy.node import Node


LOCALIZATION_NODES = (
    "map_server",
    "amcl",
)

NAVIGATION_NODES = (
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
)

DEFAULT_NODES = LOCALIZATION_NODES + NAVIGATION_NODES
LOCALIZATION_MANAGER = "/lifecycle_manager_localization/manage_nodes"
NAVIGATION_MANAGER = "/lifecycle_manager_navigation/manage_nodes"
DEFAULT_MANAGERS = (LOCALIZATION_MANAGER, NAVIGATION_MANAGER)

# nav2_msgs/srv/ManageLifecycleNodes command constants.
STARTUP = 0
RESET = 3


class Nav2Watchdog(Node):
    def __init__(self, nodes: Iterable[str], managers: Iterable[str]) -> None:
        super().__init__("lime_nav2_watchdog")
        self._nodes = tuple(nodes)
        self._navigate = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._state_clients: Dict[str, object] = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in self._nodes
        }
        self._manager_clients: Dict[str, object] = {
            service: self.create_client(ManageLifecycleNodes, service)
            for service in managers
        }
        self._stop_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _wait_future(self, future, timeout_sec: float):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().warning(f"service call failed: {exc}")
            return None

    def node_state(self, name: str, timeout_sec: float = 0.4) -> Optional[int]:
        client = self._state_clients[name]
        if not client.wait_for_service(timeout_sec=0.05):
            return None
        response = self._wait_future(client.call_async(GetState.Request()), timeout_sec)
        if response is None:
            return None
        return int(response.current_state.id)

    def health(self) -> Tuple[bool, Dict[str, Optional[int]], bool]:
        action_available = self._navigate.wait_for_server(timeout_sec=0.1)
        states = {name: self.node_state(name) for name in self._nodes}
        nodes_active = all(
            state == State.PRIMARY_STATE_ACTIVE for state in states.values()
        )
        return action_available and nodes_active, states, action_available

    def stop_robot(self) -> None:
        msg = Twist()
        for _ in range(3):
            self._stop_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    def _manage(self, service: str, command: int, timeout_sec: float = 4.0) -> bool:
        client = self._manager_clients[service]
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warning(f"lifecycle manager unavailable: {service}")
            return False
        request = ManageLifecycleNodes.Request()
        request.command = command
        response = self._wait_future(client.call_async(request), timeout_sec)
        success = bool(response is not None and response.success)
        self.get_logger().info(
            f"{service}: command={command} success={success}"
        )
        return success

    @staticmethod
    def affected_managers(
        states: Dict[str, Optional[int]], action_available: bool
    ) -> Tuple[str, ...]:
        localization_bad = any(
            states.get(name) != State.PRIMARY_STATE_ACTIVE
            for name in LOCALIZATION_NODES
            if name in states
        )
        navigation_bad = any(
            states.get(name) != State.PRIMARY_STATE_ACTIVE
            for name in NAVIGATION_NODES
            if name in states
        ) or not action_available

        # Navigation depends on localization, so recover both if localization is bad.
        if localization_bad:
            return (LOCALIZATION_MANAGER, NAVIGATION_MANAGER)
        if navigation_bad:
            return (NAVIGATION_MANAGER,)
        return ()

    def recover(
        self, states: Dict[str, Optional[int]], action_available: bool
    ) -> bool:
        managers = self.affected_managers(states, action_available)
        if not managers:
            return True

        self.get_logger().warning(
            "Nav2 unhealthy; stopping robot before lifecycle recovery: "
            + ", ".join(managers)
        )
        self.stop_robot()

        # Reset dependants before dependencies, then start dependencies first.
        reset_ok = True
        for service in reversed(managers):
            reset_ok = self._manage(service, RESET) and reset_ok

        time.sleep(2.0)

        startup_ok = True
        for service in managers:
            startup_ok = self._manage(service, STARTUP) and startup_ok

        return reset_ok and startup_ok


def _state_name(state_id: Optional[int]) -> str:
    names = {
        State.PRIMARY_STATE_UNKNOWN: "unknown",
        State.PRIMARY_STATE_UNCONFIGURED: "unconfigured",
        State.PRIMARY_STATE_INACTIVE: "inactive",
        State.PRIMARY_STATE_ACTIVE: "active",
        State.PRIMARY_STATE_FINALIZED: "finalized",
    }
    if state_id is None:
        return "unavailable"
    return names.get(state_id, str(state_id))


def parse_args():
    parser = argparse.ArgumentParser(description="Watch and recover the Lime Nav2 stack")
    parser.add_argument("--interval", type=float, default=2.0)
    parser.add_argument("--startup-grace", type=float, default=45.0)
    parser.add_argument("--failure-threshold", type=int, default=3)
    parser.add_argument("--cooldown", type=float, default=60.0)
    parser.add_argument("--recovery-grace", type=float, default=10.0)
    parser.add_argument(
        "--once", action="store_true", help="Check once and recover once if unhealthy"
    )
    parser.add_argument(
        "--check-only", action="store_true", help="Never issue lifecycle recovery commands"
    )
    parser.add_argument("--nodes", nargs="+", default=list(DEFAULT_NODES))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = Nav2Watchdog(args.nodes, DEFAULT_MANAGERS)
    failures = 0
    start = time.monotonic()
    last_recovery = float("-inf")

    try:
        while rclpy.ok():
            healthy, states, action_available = node.health()
            state_text = ", ".join(
                f"{name}={_state_name(state)}" for name, state in states.items()
            )

            if healthy:
                if failures:
                    node.get_logger().info("Nav2 recovered and is healthy again")
                failures = 0
            else:
                failures += 1
                node.get_logger().warning(
                    f"Nav2 health failed ({failures}/{args.failure_threshold}); "
                    f"navigate_to_pose={action_available}; {state_text}"
                )

            now = time.monotonic()
            grace_over = now - start >= args.startup_grace
            cooldown_over = now - last_recovery >= args.cooldown
            should_recover = (
                not healthy
                and grace_over
                and failures >= args.failure_threshold
                and cooldown_over
                and not args.check_only
            )

            if should_recover:
                last_recovery = now
                node.recover(states, action_available)
                failures = 0
                time.sleep(args.recovery_grace)
                if args.once:
                    recovered, _, _ = node.health()
                    return 0 if recovered else 1

            if args.once:
                return 0 if healthy else 1

            time.sleep(max(args.interval, 0.2))
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
