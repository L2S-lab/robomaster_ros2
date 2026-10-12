# Copyright 2026 Aarsh
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Exercise a heterogeneous RoboMaster fleet through its public ROS API."""

import argparse
from collections import Counter, defaultdict
from dataclasses import dataclass
import signal
from statistics import mean
import sys
from time import monotonic
from typing import Any, Iterable, List, Optional, Sequence, Tuple

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from robomaster_interface.srv import RobotLED
from robomaster_interface.srv import SetSpeed
from robomaster_interface.srv import Takeoff
from robomaster_interface.srv import TelloLED
from robomaster_interface.srv import TelloMled
from rosidl_runtime_py.utilities import get_message
from std_srvs.srv import Empty, SetBool, Trigger


DEFAULT_DRONES = tuple(f'rmtt_{number}' for number in range(1, 6))
DEFAULT_ROBOTS = ('rmep_1', 'rmep_2')
MOTION_CONFIRMATION = 'MOVE_5_DRONES_2_ROBOTS'
CONTINUOUS_TELEMETRY_TOPICS = frozenset((
    'front_tof', 'bottom_tof', 'attitude', 'baro', 'imu', 'mpad',
    'position', 'gimbal_angle', 'arm_position', 'arm_action_status',
    'gripper_state', 'image', 'image/compressed',
))


@dataclass(frozen=True)
class RobotSpec:
    """A named robot and its driver family."""

    name: str
    kind: str


@dataclass(frozen=True)
class ServiceCall:
    """One service request in a concurrent batch."""

    target: str
    suffix: str
    service_type: Any
    request: Any


@dataclass(frozen=True)
class CheckResult:
    """A single test result used to build the final summary."""

    operation: str
    target: str
    outcome: str
    latency_s: float = 0.0
    detail: str = ''


def parse_names(value: str) -> Tuple[str, ...]:
    """Parse and validate a comma-separated ROS namespace list."""
    names = tuple(item.strip().strip('/') for item in value.split(','))
    if not names or any(not name for name in names):
        raise argparse.ArgumentTypeError('robot names cannot be empty')
    if len(set(names)) != len(names):
        raise argparse.ArgumentTypeError('robot names must be unique')
    if any('/' in name or ' ' in name for name in names):
        raise argparse.ArgumentTypeError(
            'use simple namespaces such as rmtt_1 or rmep_1'
        )
    return names


def percentile(values: Sequence[float], percentage: float) -> float:
    """Return a nearest-rank percentile without an external dependency."""
    if not values:
        return 0.0
    ordered = sorted(values)
    rank = round((len(ordered) - 1) * percentage)
    return ordered[rank]


def make_parser() -> argparse.ArgumentParser:
    """Create the command-line parser."""
    parser = argparse.ArgumentParser(
        description=(
            'Concurrent API and deadman test for five RMTTs and two EPs. '
            'The default diagnostic mode never requests physical motion.'
        )
    )
    parser.add_argument(
        '--drones', type=parse_names, default=DEFAULT_DRONES,
        help='comma-separated drone namespaces (default: rmtt_1..rmtt_5)',
    )
    parser.add_argument(
        '--robots', type=parse_names, default=DEFAULT_ROBOTS,
        help='comma-separated EP namespaces (default: rmep_1,rmep_2)',
    )
    parser.add_argument(
        '--mode', choices=('diagnostic', 'ground', 'motion'),
        default='diagnostic',
        help='test level; only motion permits takeoff or non-zero cmd_vel',
    )
    parser.add_argument(
        '--rounds', type=int, default=10,
        help='concurrent diagnostic service rounds (default: 10)',
    )
    parser.add_argument(
        '--duration', type=float, default=10.0,
        help='topic observation or command-stream duration in seconds',
    )
    parser.add_argument(
        '--rate', type=float, default=20.0,
        help='cmd_vel publication rate per robot in Hz (default: 20)',
    )
    parser.add_argument(
        '--speed', type=float, default=0.10,
        help=(
            'bounded translation input: ground m/s, drone normalized RC '
            '(default: 0.10)'
        ),
    )
    parser.add_argument(
        '--takeoff-height', type=float, default=0.8,
        help='motion-test takeoff height in metres (default: 0.8)',
    )
    parser.add_argument(
        '--deadman-wait', type=float, default=1.0,
        help='silence per robot; must exceed the configured timeout',
    )
    parser.add_argument(
        '--min-battery', type=float, default=30.0,
        help='minimum reported drone battery percentage for motion mode',
    )
    parser.add_argument(
        '--service-wait', type=float, default=20.0,
        help='maximum time to discover each batch of services',
    )
    parser.add_argument(
        '--call-timeout', type=float, default=5.0,
        help='maximum time for a concurrent service batch',
    )
    parser.add_argument(
        '--preflight-delay', type=float, default=5.0,
        help='countdown before takeoff in motion mode',
    )
    parser.add_argument(
        '--confirm-motion', default='', metavar='PHRASE',
        help=f'required in motion mode: {MOTION_CONFIRMATION}',
    )
    parser.add_argument(
        '--no-topic-monitor', action='store_true',
        help='do not subscribe to and count fleet telemetry topics',
    )
    return parser


def validate_arguments(args: argparse.Namespace) -> None:
    """Reject unsafe or nonsensical stress-test settings."""
    if set(args.drones) & set(args.robots):
        raise ValueError('drone and EP namespace lists must not overlap')
    if not 1 <= args.rounds <= 1000:
        raise ValueError('--rounds must be in [1, 1000]')
    if not 0.1 <= args.duration <= 600.0:
        raise ValueError('--duration must be in [0.1, 600] seconds')
    if not 1.0 <= args.rate <= 100.0:
        raise ValueError('--rate must be in [1, 100] Hz')
    if not 0.01 <= args.speed <= 0.25:
        raise ValueError('--speed must be in [0.01, 0.25] m/s')
    if not 0.8 <= args.takeoff_height <= 1.5:
        raise ValueError('--takeoff-height must be in [0.8, 1.5] metres')
    if not 0.2 <= args.deadman_wait <= 10.0:
        raise ValueError('--deadman-wait must be in [0.2, 10] seconds')
    if not 10.0 <= args.min_battery <= 100.0:
        raise ValueError('--min-battery must be in [10, 100] percent')
    if args.service_wait <= 0.0 or args.call_timeout <= 0.0:
        raise ValueError('service wait and call timeout must be positive')
    if not 0.0 <= args.preflight_delay <= 30.0:
        raise ValueError('--preflight-delay must be in [0, 30] seconds')
    if (
        args.mode == 'motion'
        and args.confirm_motion != MOTION_CONFIRMATION
    ):
        raise ValueError(
            'motion mode requires --confirm-motion '
            f'{MOTION_CONFIRMATION}'
        )


class FleetStressTest(Node):
    """ROS node that runs concurrent tests against an existing server."""

    def __init__(self, args: argparse.Namespace) -> None:
        """Create clients, publishers and result collectors for the fleet."""
        super().__init__('fleet_stress_test')
        self.args = args
        self.drones = tuple(
            RobotSpec(name, 'drone') for name in args.drones
        )
        self.robots = tuple(
            RobotSpec(name, 'robot') for name in args.robots
        )
        self.fleet = self.drones + self.robots
        self.results: List[CheckResult] = []
        self._service_clients = {}
        self._command_publishers = {
            spec.name: self.create_publisher(
                Twist, f'/{spec.name}/cmd_vel', 10
            )
            for spec in self.fleet
        }
        self._telemetry_subscriptions = []
        self._topic_counts = Counter()
        self._monitor_started: Optional[float] = None
        self._airborne_attempted = False
        self._armed = False

    def _spin_for(self, duration: float) -> None:
        """Process callbacks for a bounded interval."""
        deadline = monotonic() + max(duration, 0.0)
        while rclpy.ok() and monotonic() < deadline:
            remaining = deadline - monotonic()
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))

    def _client_for(self, call: ServiceCall):
        """Return a cached client for a service call."""
        service_name = f'/{call.target}/{call.suffix}'
        key = (service_name, call.service_type)
        if key not in self._service_clients:
            self._service_clients[key] = self.create_client(
                call.service_type, service_name
            )
        return self._service_clients[key]

    def wait_for_required_services(self) -> bool:
        """Wait once for every API required by the selected test mode."""
        calls = []
        calls.extend(self._trigger_calls(self.fleet, 'get_sn'))
        calls.extend(self._trigger_calls(self.drones, 'get_battery'))
        calls.extend(self._trigger_calls(self.drones, 'status'))
        if self.args.mode != 'diagnostic':
            calls.extend(self._arm_calls(True))
            for drone in self.drones:
                calls.extend((
                    ServiceCall(
                        drone.name, 'set_speed',
                        SetSpeed, SetSpeed.Request(),
                    ),
                    ServiceCall(
                        drone.name, 'set_led',
                        TelloLED, TelloLED.Request(),
                    ),
                    ServiceCall(
                        drone.name, 'set_mled',
                        TelloMled, TelloMled.Request(),
                    ),
                ))
            for robot in self.robots:
                calls.append(ServiceCall(
                    robot.name, 'led_chassis',
                    RobotLED, RobotLED.Request(),
                ))
        if self.args.mode == 'motion':
            for drone in self.drones:
                calls.extend((
                    ServiceCall(
                        drone.name, 'takeoff', Takeoff, Takeoff.Request()
                    ),
                    ServiceCall(
                        drone.name, 'hover', Empty, Empty.Request()
                    ),
                    ServiceCall(
                        drone.name, 'land', Trigger, Trigger.Request()
                    ),
                ))

        waiting = [(call, self._client_for(call)) for call in calls]
        deadline = monotonic() + self.args.service_wait
        while waiting and rclpy.ok() and monotonic() < deadline:
            waiting = [
                item for item in waiting if not item[1].service_is_ready()
            ]
            if waiting:
                rclpy.spin_once(self, timeout_sec=0.05)

        missing_keys = {
            (call.target, call.suffix) for call, _client in waiting
        }
        for call in calls:
            key = (call.target, call.suffix)
            outcome = 'FAIL' if key in missing_keys else 'PASS'
            detail = (
                f'/{call.target}/{call.suffix} unavailable'
                if key in missing_keys else ''
            )
            self.results.append(CheckResult(
                'service_ready', f'{call.target}/{call.suffix}',
                outcome, detail=detail,
            ))
        if missing_keys:
            missing_text = ', '.join(
                f'/{target}/{suffix}'
                for target, suffix in sorted(missing_keys)
            )
            self.get_logger().error(
                f'Required services unavailable: {missing_text}'
            )
        return not missing_keys

    def call_batch(
        self, operation: str, calls: Iterable[ServiceCall]
    ) -> List[CheckResult]:
        """Issue a service batch together and collect bounded results."""
        prepared = [(call, self._client_for(call)) for call in calls]
        ready = []
        waiting = list(prepared)
        discovery_deadline = monotonic() + self.args.service_wait

        while waiting and rclpy.ok() and monotonic() < discovery_deadline:
            for item in tuple(waiting):
                if item[1].service_is_ready():
                    waiting.remove(item)
                    ready.append(item)
            if waiting:
                rclpy.spin_once(self, timeout_sec=0.05)

        batch_results = []
        for call, _client in waiting:
            batch_results.append(CheckResult(
                operation, call.target, 'FAIL',
                detail=f'/{call.target}/{call.suffix} unavailable',
            ))

        started = monotonic()
        pending = [
            (call, client.call_async(call.request))
            for call, client in ready
        ]
        call_deadline = started + self.args.call_timeout
        while pending and rclpy.ok() and monotonic() < call_deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            for call, future in tuple(pending):
                if not future.done():
                    continue
                latency = monotonic() - started
                try:
                    response = future.result()
                    if response is None:
                        success = False
                        detail = 'service returned no response'
                    else:
                        success = bool(getattr(response, 'success', True))
                        detail = str(
                            getattr(response, 'message', '')
                        ).strip()
                    outcome = 'PASS' if success else 'FAIL'
                except Exception as exc:  # ROS service exceptions vary.
                    outcome = 'FAIL'
                    detail = f'{type(exc).__name__}: {exc}'
                batch_results.append(CheckResult(
                    operation, call.target, outcome, latency, detail[:160]
                ))
                pending.remove((call, future))

        for call, _future in pending:
            batch_results.append(CheckResult(
                operation, call.target, 'FAIL',
                monotonic() - started, 'service response timeout',
            ))

        self.results.extend(batch_results)
        failures = sum(result.outcome == 'FAIL' for result in batch_results)
        self.get_logger().info(
            f'{operation}: {len(batch_results) - failures}/'
            f'{len(batch_results)} passed'
        )
        return batch_results

    @staticmethod
    def _trigger_calls(
        specs: Iterable[RobotSpec], suffix: str
    ) -> List[ServiceCall]:
        """Build Trigger requests for a group of robots."""
        return [
            ServiceCall(spec.name, suffix, Trigger, Trigger.Request())
            for spec in specs
        ]

    def _arm_calls(self, armed: bool) -> List[ServiceCall]:
        """Build per-robot arm or disarm service requests."""
        calls = []
        for spec in self.fleet:
            request = SetBool.Request()
            request.data = armed
            calls.append(ServiceCall(
                spec.name, 'set_armed', SetBool, request
            ))
        return calls

    def run_diagnostics(self) -> None:
        """Stress the read-only APIs concurrently across the fleet."""
        for number in range(1, self.args.rounds + 1):
            self.call_batch(
                'get_sn', self._trigger_calls(self.fleet, 'get_sn')
            )
            self.call_batch(
                'get_battery',
                self._trigger_calls(self.drones, 'get_battery'),
            )
            self.call_batch(
                'status', self._trigger_calls(self.drones, 'status')
            )
            if number != self.args.rounds:
                self._spin_for(0.02)

    def exercise_auxiliary_functions(self) -> None:
        """Exercise non-motion settings and lights on all devices."""
        speed_calls = []
        led_calls = []
        mled_calls = []
        for drone in self.drones:
            speed = SetSpeed.Request()
            speed.speed = 0.3
            speed_calls.append(ServiceCall(
                drone.name, 'set_speed', SetSpeed, speed
            ))

            led = TelloLED.Request()
            led.led.g = 32.0
            led_calls.append(ServiceCall(
                drone.name, 'set_led', TelloLED, led
            ))

            mled = TelloMled.Request()
            mled.brightness = 32
            mled.color = 'b'
            mled.mled_char = 'T'
            mled_calls.append(ServiceCall(
                drone.name, 'set_mled', TelloMled, mled
            ))

        robot_led_calls = []
        for robot in self.robots:
            led = RobotLED.Request()
            led.on = True
            led.which = 'all'
            led.led.b = 32.0
            led.effect = 'none'
            led.freq = 1
            robot_led_calls.append(ServiceCall(
                robot.name, 'led_chassis', RobotLED, led
            ))

        self.call_batch('set_speed', speed_calls)
        self.call_batch('drone_led', led_calls)
        self.call_batch('drone_mled', mled_calls)
        self.call_batch('robot_led', robot_led_calls)

    def setup_topic_monitor(self) -> None:
        """Subscribe to advertised telemetry below fleet namespaces."""
        if self.args.no_topic_monitor:
            return
        self._spin_for(0.5)
        prefixes = tuple(f'/{spec.name}/' for spec in self.fleet)
        for topic_name, type_names in self.get_topic_names_and_types():
            prefix = next((
                prefix for prefix in prefixes if topic_name.startswith(prefix)
            ), None)
            if prefix is None or not type_names:
                continue
            if topic_name[len(prefix):] not in CONTINUOUS_TELEMETRY_TOPICS:
                continue
            try:
                message_type = get_message(type_names[0])
                subscription = self.create_subscription(
                    message_type,
                    topic_name,
                    lambda _message, name=topic_name: (
                        self._topic_counts.update((name,))
                    ),
                    qos_profile_sensor_data,
                )
                self._telemetry_subscriptions.append(subscription)
                self._topic_counts[topic_name] = 0
            except Exception as exc:
                self.get_logger().warning(
                    f'Cannot monitor {topic_name} ({type_names[0]}): {exc}'
                )
        self._monitor_started = monotonic()
        self.get_logger().info(
            'Monitoring '
            f'{len(self._telemetry_subscriptions)} fleet telemetry topics'
        )

    def evaluate_telemetry(self) -> None:
        """Require a message from every advertised monitored topic."""
        if self._monitor_started is None:
            return
        for topic_name, count in sorted(self._topic_counts.items()):
            outcome = 'PASS' if count > 0 else 'FAIL'
            detail = '' if count > 0 else 'no messages received'
            self.results.append(CheckResult(
                'telemetry', topic_name, outcome, detail=detail
            ))

    def wait_for_command_subscribers(self) -> bool:
        """Wait until every cmd_vel publisher has a matching driver."""
        deadline = monotonic() + self.args.service_wait
        missing = set(self._command_publishers)
        while missing and rclpy.ok() and monotonic() < deadline:
            missing = {
                name for name, publisher in self._command_publishers.items()
                if publisher.get_subscription_count() == 0
            }
            if missing:
                rclpy.spin_once(self, timeout_sec=0.05)
        for name in self._command_publishers:
            outcome = 'FAIL' if name in missing else 'PASS'
            detail = 'no cmd_vel subscriber' if name in missing else ''
            self.results.append(CheckResult(
                'cmd_vel_ready', name, outcome, detail=detail
            ))
        if missing:
            self.get_logger().error(
                'Missing cmd_vel subscribers: ' + ', '.join(sorted(missing))
            )
        return not missing

    def _publish_frame(
        self,
        moving: Iterable[str] = (),
        silent: Iterable[str] = (),
        direction: float = 1.0,
    ) -> None:
        """Publish one bounded command frame across the fleet."""
        moving_names = set(moving)
        silent_names = set(silent)
        for spec in self.fleet:
            if spec.name in silent_names:
                continue
            message = Twist()
            if spec.name in moving_names:
                message.linear.x = self.args.speed * direction
            self._command_publishers[spec.name].publish(message)

    def stream_commands(
        self,
        duration: float,
        moving: Iterable[str] = (),
        silent: Iterable[str] = (),
        alternate: bool = False,
    ) -> None:
        """Stream bounded commands at the requested per-robot rate."""
        period = 1.0 / self.args.rate
        started = monotonic()
        deadline = started + duration
        next_frame = started
        while rclpy.ok() and monotonic() < deadline:
            elapsed = monotonic() - started
            direction = 1.0
            if alternate and int(elapsed / 0.5) % 2:
                direction = -1.0
            self._publish_frame(moving, silent, direction)
            next_frame += period
            next_frame = max(next_frame, monotonic())
            frame_deadline = min(next_frame, deadline)
            while rclpy.ok() and monotonic() < frame_deadline:
                remaining = frame_deadline - monotonic()
                rclpy.spin_once(self, timeout_sec=min(0.01, remaining))

    def publish_zero_burst(self) -> None:
        """Send several explicit zero frames to every device."""
        self.stream_commands(0.25)

    def check_motion_batteries(self) -> bool:
        """Require a valid, sufficiently charged battery from every drone."""
        battery_results = self.call_batch(
            'preflight_battery',
            self._trigger_calls(self.drones, 'get_battery'),
        )
        safe = True
        for result in battery_results:
            if result.outcome != 'PASS':
                safe = False
                continue
            try:
                battery = float(result.detail.rstrip('%'))
            except ValueError:
                battery = -1.0
            if battery < self.args.min_battery:
                safe = False
                self.results.append(CheckResult(
                    'battery_threshold', result.target, 'FAIL',
                    detail=(
                        f'{battery}% is below {self.args.min_battery}%'
                    ),
                ))
        return safe

    def takeoff(self) -> bool:
        """Request concurrent takeoff after an explicit countdown."""
        self.get_logger().warning(
            'MOTION ENABLED: clear the flight volume and robot floor area.'
        )
        if self.args.preflight_delay:
            self.get_logger().warning(
                f'Takeoff begins in {self.args.preflight_delay:.1f} seconds'
            )
            self._spin_for(self.args.preflight_delay)
        calls = []
        for drone in self.drones:
            request = Takeoff.Request()
            request.height = self.args.takeoff_height
            request.sync = False
            calls.append(ServiceCall(
                drone.name, 'takeoff', Takeoff, request
            ))
        self._airborne_attempted = True
        results = self.call_batch('takeoff', calls)
        return bool(results) and all(
            result.outcome == 'PASS' for result in results
        )

    def exercise_deadman(self) -> None:
        """Stop one command stream at a time while servicing all others."""
        warmup = max(0.3, 4.0 / self.args.rate)
        for spec in self.fleet:
            self.get_logger().warning(
                f'Deadman test for {spec.name}: command, then '
                f'{self.args.deadman_wait:.2f}s silence'
            )
            self.stream_commands(warmup, moving=(spec.name,))
            others = tuple(
                item.name for item in self.fleet if item.name != spec.name
            )
            self.stream_commands(
                self.args.deadman_wait,
                moving=(),
                silent=(spec.name,),
            )
            self._publish_frame(moving=(), silent=others)
            self.results.append(CheckResult(
                'deadman_observation', spec.name, 'OBSERVE',
                detail='verify stop/hover occurred without affecting peers',
            ))

    def _hover_calls(self) -> List[ServiceCall]:
        """Build hover calls for all drones."""
        return [
            ServiceCall(drone.name, 'hover', Empty, Empty.Request())
            for drone in self.drones
        ]

    def safe_cleanup(self) -> None:
        """Attempt zero, hover, land and disarm during every exit path."""
        if self.args.mode == 'diagnostic':
            return
        self.get_logger().warning('Running fleet safety cleanup')
        self.publish_zero_burst()
        if self._airborne_attempted:
            self.call_batch('cleanup_hover', self._hover_calls())
            self.call_batch(
                'cleanup_land', self._trigger_calls(self.drones, 'land')
            )
        self.call_batch('cleanup_disarm', self._arm_calls(False))
        self._armed = False

    def report_uninvoked_apis(self) -> None:
        """Report intentionally excluded hazardous or disruptive APIs."""
        service_names = {
            name for name, _types in self.get_service_names_and_types()
        }
        suffixes = (
            'emergency', 'soft_emergency', 'reboot', 'close_robot',
            'fire', 'move_arm', 'move_gimbal', 'gripper', 'move', 'goto',
        )
        found = []
        for spec in self.fleet:
            for suffix in suffixes:
                name = f'/{spec.name}/{suffix}'
                if name in service_names:
                    found.append(name)
        if found:
            self.get_logger().info(
                'Inventoried but intentionally not invoked: '
                + ', '.join(sorted(found))
            )

    def print_summary(self) -> int:
        """Print operation latency, failures, telemetry and exit status."""
        grouped = defaultdict(list)
        for result in self.results:
            grouped[result.operation].append(result)

        self.get_logger().info('========== fleet stress summary ==========')
        for operation in sorted(grouped):
            results = grouped[operation]
            passes = sum(item.outcome == 'PASS' for item in results)
            failures = sum(item.outcome == 'FAIL' for item in results)
            observations = sum(item.outcome == 'OBSERVE' for item in results)
            latencies = [
                item.latency_s for item in results if item.latency_s > 0.0
            ]
            latency_text = ''
            if latencies:
                latency_text = (
                    f', mean={mean(latencies) * 1000:.1f}ms, '
                    f'p95={percentile(latencies, 0.95) * 1000:.1f}ms, '
                    f'max={max(latencies) * 1000:.1f}ms'
                )
            self.get_logger().info(
                f'{operation}: pass={passes}, fail={failures}, '
                f'observe={observations}{latency_text}'
            )
            for item in results:
                if item.outcome == 'FAIL':
                    self.get_logger().error(
                        f'  {item.target}: {item.detail or "failed"}'
                    )

        if self._monitor_started is not None:
            elapsed = max(monotonic() - self._monitor_started, 0.001)
            if not self._topic_counts:
                self.get_logger().warning(
                    'No monitored telemetry messages were received'
                )
            for topic_name, count in sorted(self._topic_counts.items()):
                self.get_logger().info(
                    f'topic {topic_name}: {count} messages '
                    f'({count / elapsed:.1f} Hz average)'
                )

        failures = [
            result for result in self.results if result.outcome == 'FAIL'
        ]
        if failures:
            self.get_logger().error(
                f'RESULT: FAIL ({len(failures)} failed checks)'
            )
            return 1
        self.get_logger().info('RESULT: PASS')
        return 0

    def run(self) -> int:
        """Execute the selected test mode and return a process exit code."""
        self.get_logger().info(
            f'Starting {self.args.mode} test for {len(self.drones)} drones '
            f'and {len(self.robots)} robots'
        )
        try:
            if self.wait_for_required_services():
                self.setup_topic_monitor()
                self.run_diagnostics()
                self.report_uninvoked_apis()
                diagnostics_failed = any(
                    result.outcome == 'FAIL'
                    and result.operation in (
                        'get_sn', 'get_battery', 'status'
                    )
                    for result in self.results
                )

                if self.args.mode == 'diagnostic':
                    self._spin_for(self.args.duration)
                elif self.args.mode == 'motion' and diagnostics_failed:
                    self.get_logger().error(
                        'Motion aborted because diagnostic checks failed'
                    )
                elif self.wait_for_command_subscribers():
                    arm_results = self.call_batch(
                        'arm', self._arm_calls(True)
                    )
                    self._armed = bool(arm_results) and all(
                        result.outcome == 'PASS'
                        for result in arm_results
                    )
                    if self._armed:
                        self.exercise_auxiliary_functions()
                        if self.args.mode == 'ground':
                            self.stream_commands(self.args.duration)
                        elif self.check_motion_batteries():
                            if self.takeoff():
                                self.call_batch(
                                    'hover', self._hover_calls()
                                )
                                moving = tuple(
                                    spec.name for spec in self.fleet
                                )
                                self.get_logger().warning(
                                    'Beginning bounded parallel '
                                    'command-stream test'
                                )
                                self.stream_commands(
                                    self.args.duration,
                                    moving=moving,
                                    alternate=True,
                                )
                                self.publish_zero_burst()
                                self.exercise_deadman()
                            else:
                                self.get_logger().error(
                                    'Motion aborted: takeoff batch failed'
                                )
                        else:
                            self.get_logger().error(
                                'Motion aborted by battery preflight'
                            )
        finally:
            self.safe_cleanup()
        self.evaluate_telemetry()
        return self.print_summary()


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Parse CLI arguments, run the node and return a shell exit code."""
    raw_arguments = list(argv) if argv is not None else sys.argv
    parser = make_parser()
    args = parser.parse_args(remove_ros_args(raw_arguments)[1:])
    try:
        validate_arguments(args)
    except ValueError as exc:
        parser.error(str(exc))

    rclpy.init(
        args=raw_arguments,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = None
    exit_code = 1
    previous_handlers = {}

    def request_safe_stop(_signal_number, _frame):
        raise KeyboardInterrupt

    try:
        node = FleetStressTest(args)
        for signal_number in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signal_number] = signal.signal(
                signal_number, request_safe_stop
            )
        exit_code = node.run()
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warning('Interrupted by operator')
    except Exception as exc:  # Keep cleanup active for unexpected failures.
        if node is not None:
            node.get_logger().error(
                f'Unhandled test error: {type(exc).__name__}: {exc}'
            )
    finally:
        for signal_number, previous_handler in previous_handlers.items():
            signal.signal(signal_number, previous_handler)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
