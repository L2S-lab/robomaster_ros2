# Joystick mapping (as provided by user)
#
# Axes map:
# 1-2 analogue axes[0] value -1 to 1 JOYSTICK LEFT X
# 3-4 analogue axes[1] value -1 to 1 JOYSTICK LEFT Y
# 5 int axes[2] value 1 when pressed L2
# 6-7 analogue axes[3] value -1 to 1 JOYSTICK RIGHT X
# 8-9 analogue axes[4] value -1 to 1 JOYSTICK RIGHT Y
# 10 int axes[5] value 1 when pressed R2
# 11-12 int axes[6] value -1 for 11, +1 for 12 DPAD LEFT/RIGHT
# 13-14 int axes[7] value -1 for 13, +1 for 14 DPAD UP/DOWN
#
# Buttons are always 0 when released and 1 when pressed.
# Button array map:
# [15,16,17,18,19,20,21,22,-,23,24,-]
# [B,A,X,Y,L1,R1,Select,Start,-,J1, J2,-]
# => 15->buttons[0], 16->buttons[1], 17->buttons[2], 18->buttons[3],
#    19->buttons[4], 20->buttons[5], 21->buttons[6], 22->buttons[7],
#    23->buttons[9], 24->buttons[10]

from __future__ import annotations

from math import isfinite
import signal
import sys
from time import monotonic
from typing import Dict, List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from robomaster_interface.msg import GimbalVel
from robomaster_interface.srv import GripperCommand, Takeoff
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty, SetBool, Trigger


class JoyFleetTeleop(Node):
    ANSI_GREEN = '\033[32m'
    ANSI_RED = '\033[31m'
    ANSI_RESET = '\033[0m'

    AXIS_LEFT_STICK_X = 0
    AXIS_LEFT_STICK_Y = 1
    AXIS_L2 = 2
    AXIS_RIGHT_STICK_X = 3
    AXIS_RIGHT_STICK_Y = 4
    AXIS_R2 = 5
    AXIS_DPAD_LEFT_RIGHT = 6
    AXIS_DPAD_UP_DOWN = 7

    BTN_B = 0
    BTN_A = 1
    BTN_X = 3
    BTN_Y = 2
    BTN_L1 = 4
    BTN_R1 = 5
    BTN_SELECT = 6
    BTN_START = 7

    def __init__(self) -> None:
        super().__init__('joy_fleet_teleop')

        self.declare_parameter('robot_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('drone_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('robot_names_csv', 'rmep_1')
        self.declare_parameter('drone_names_csv', 'rmtt_1')
        self.declare_parameter('start_mode', 'robot')
        self.declare_parameter('takeoff_height', 1.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('require_deadman', True)
        self.declare_parameter('joy_timeout', 0.5)
        self.declare_parameter('stream_status_line', True)
        self.declare_parameter('show_control_legend', True)
        self.declare_parameter('robot_max_linear', 1.8)
        self.declare_parameter('robot_max_yaw', 2.4)
        self.declare_parameter('drone_max_linear', 0.8)
        self.declare_parameter('drone_max_yaw', 0.8)
        self.declare_parameter('gimbal_max_speed', 180.0)
        self.declare_parameter('arm_max_linear_x', 0.18)
        self.declare_parameter('arm_max_linear_z', 0.18)

        self.robot_names = self._parse_name_list('robot_names', 'robot_names_csv')
        self.drone_names = self._parse_name_list('drone_names', 'drone_names_csv')
        self.mode = str(self.get_parameter('start_mode').value).strip().lower()
        if self.mode not in ('robot', 'drone'):
            self.mode = 'robot'

        self.takeoff_height = float(self.get_parameter('takeoff_height').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.require_deadman = bool(self.get_parameter('require_deadman').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout').value)
        if not isfinite(self.joy_timeout) or self.joy_timeout <= 0.0:
            raise ValueError('joy_timeout must be finite and positive')
        self.stream_status_line = bool(self.get_parameter('stream_status_line').value)
        self.show_control_legend = bool(self.get_parameter('show_control_legend').value)
        self.robot_max_linear = float(self.get_parameter('robot_max_linear').value)
        self.robot_max_yaw = float(self.get_parameter('robot_max_yaw').value)
        self.drone_max_linear = float(self.get_parameter('drone_max_linear').value)
        self.drone_max_yaw = float(self.get_parameter('drone_max_yaw').value)
        self.gimbal_max_speed = float(self.get_parameter('gimbal_max_speed').value)
        self.arm_max_linear_x = float(self.get_parameter('arm_max_linear_x').value)
        self.arm_max_linear_z = float(self.get_parameter('arm_max_linear_z').value)

        self.active_robot_idx = 0
        self.active_drone_idx = 0
        self.robot_armed: Dict[str, bool] = {name: False for name in self.robot_names}
        self.drone_armed: Dict[str, bool] = {name: False for name in self.drone_names}

        self.axes: List[float] = []
        self.buttons: List[int] = []
        self.prev_buttons: List[int] = []
        self._last_joy_at = None
        self._input_stopped = False
        self.last_warn_ns: Dict[str, int] = {}
        self._status_line_width = 0

        self.robot_cmd_pubs = {
            name: self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            for name in self.robot_names
        }
        self.robot_gimbal_pubs = {
            name: self.create_publisher(GimbalVel, f'/{name}/gimbal_cmd_vel', 10)
            for name in self.robot_names
        }
        self.robot_arm_cmd_pubs = {
            name: self.create_publisher(Twist, f'/{name}/arm_cmd_vel', 10)
            for name in self.robot_names
        }
        self.drone_cmd_pubs = {
            name: self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            for name in self.drone_names
        }
        self.robot_recenter_clients = {
            name: self.create_client(Trigger, f'/{name}/recenter_arm')
            for name in self.robot_names
        }
        self.robot_gripper_clients = {
            name: self.create_client(GripperCommand, f'/{name}/gripper_command')
            for name in self.robot_names
        }
        self.robot_arm_gate_clients = {
            name: self.create_client(SetBool, f'/{name}/set_armed')
            for name in self.robot_names
        }

        self.drone_takeoff_clients = {
            name: self.create_client(Takeoff, f'/{name}/takeoff')
            for name in self.drone_names
        }
        self.drone_land_clients = {
            name: self.create_client(Trigger, f'/{name}/land')
            for name in self.drone_names
        }
        self.drone_hover_clients = {
            name: self.create_client(Empty, f'/{name}/hover')
            for name in self.drone_names
        }
        self.drone_emergency_clients = {
            name: self.create_client(Empty, f'/{name}/emergency')
            for name in self.drone_names
        }
        self.drone_arm_gate_clients = {
            name: self.create_client(SetBool, f'/{name}/set_armed')
            for name in self.drone_names
        }

        self.joy_sub = self.create_subscription(Joy, '/joy', self._joy_cb, 20)
        publish_rate = max(float(self.get_parameter('publish_rate').value), 1.0)
        self.motion_timer = self.create_timer(1.0 / publish_rate, self._publish_motion)

        self.get_logger().debug(
            f'Config: mode={self.mode}, robot_names={self.robot_names}, '
            f'drone_names={self.drone_names}'
        )
        if self.show_control_legend:
            self._print_controller_help()
        self._status_info(f'Ready | {self._active_target_summary()} | hold L1 to move')

    def _parse_name_list(self, array_key: str, csv_key: str) -> List[str]:
        array_val = self.get_parameter_or(array_key).value
        if isinstance(array_val, list):
            names = [str(v).strip() for v in array_val if str(v).strip()]
            if names:
                return names
        csv_val = str(self.get_parameter(csv_key).value)
        names = [part.strip() for part in csv_val.split(',') if part.strip()]
        return names

    def _joy_cb(self, msg: Joy) -> None:
        self._last_joy_at = monotonic()
        was_deadman = self._button_down(self.BTN_L1)
        buttons = [int(v) for v in msg.buttons]
        if len(self.prev_buttons) != len(buttons):
            self.prev_buttons = [0] * len(buttons)
        self.axes = list(msg.axes)
        self.buttons = buttons
        if self.require_deadman and was_deadman and not self._button_down(self.BTN_L1):
            self._stop_all_motion()

        if self._just_pressed(self.BTN_START, buttons):
            self._toggle_armed()

        if self._just_pressed(self.BTN_SELECT, buttons):
            self._cycle_target()

        if self.mode == 'robot':
            self._handle_robot_button_edges(buttons)
        else:
            self._handle_drone_button_edges(buttons)

        self.prev_buttons = buttons

    def _cycle_target(self) -> None:
        if self.mode == 'robot':
            if not self.robot_names:
                self._status_warn('No robots configured to cycle.')
                return
            self._stop_robot_motion(self.current_robot_name)
            self.active_robot_idx = (self.active_robot_idx + 1) % len(self.robot_names)
            self._status_info(f'Active robot: {self.current_robot_name}')
            return
        if not self.drone_names:
            self._status_warn('No drones configured to cycle.')
            return
        self._stop_drone_motion(self.current_drone_name)
        self.active_drone_idx = (self.active_drone_idx + 1) % len(self.drone_names)
        self._status_info(f'Active drone: {self.current_drone_name}')

    def _toggle_armed(self) -> None:
        if self.mode == 'robot':
            name = self.current_robot_name
            if name is None:
                return
            target_state = not self.robot_armed.get(name, False)
            request = SetBool.Request()
            request.data = target_state
            client = self.robot_arm_gate_clients[name]
            future = self._call_service(client, request, f'{name} set_armed={target_state}')
            if future is not None:
                future.add_done_callback(
                    lambda fut, robot_name=name, armed=target_state: self._on_arm_toggle_done(
                        fut, self.robot_armed, robot_name, armed
                    )
                )
            return

        name = self.current_drone_name
        if name is None:
            return
        target_state = not self.drone_armed.get(name, False)
        request = SetBool.Request()
        request.data = target_state
        client = self.drone_arm_gate_clients[name]
        future = self._call_service(client, request, f'{name} set_armed={target_state}')
        if future is not None:
            future.add_done_callback(
                lambda fut, drone_name=name, armed=target_state: self._on_arm_toggle_done(
                    fut, self.drone_armed, drone_name, armed
                )
            )

    def _on_arm_toggle_done(self, future, cache: Dict[str, bool], name: str, target_state: bool) -> None:
        response = self._future_response(future)
        if response is None:
            return
        if bool(getattr(response, 'success', False)):
            cache[name] = target_state
            self._status_info(self._armed_status_line())

    def _handle_robot_button_edges(self, buttons: List[int]) -> None:
        name = self.current_robot_name
        if name is None:
            return

        if self._just_pressed(self.BTN_B, buttons):
            self._send_gripper_command(name, 'close')
        if self._just_pressed(self.BTN_A, buttons):
            self._send_gripper_command(name, 'open')
        if self._just_pressed(self.BTN_X, buttons):
            self._send_gripper_command(name, 'reset')
        if self._just_pressed(self.BTN_Y, buttons):
            self._send_gripper_command(name, 'pause')

    def _handle_drone_button_edges(self, buttons: List[int]) -> None:
        name = self.current_drone_name
        if name is None:
            return

        if self._just_pressed(self.BTN_X, buttons):
            request = Takeoff.Request()
            request.height = float(self.takeoff_height)
            request.sync = False
            self._call_service(
                self.drone_takeoff_clients[name],
                request,
                f'{name} takeoff({self.takeoff_height:.2f}m)',
            )
        if self._just_pressed(self.BTN_Y, buttons):
            request = Trigger.Request()
            self._call_service(self.drone_land_clients[name], request, f'{name} land')
        if self._just_pressed(self.BTN_A, buttons):
            request = Empty.Request()
            self._call_service(self.drone_hover_clients[name], request, f'{name} hover')

    def _send_gripper_command(self, robot_name: str, command: str) -> None:
        request = GripperCommand.Request()
        request.command = command
        request.power = 0
        self._call_service(
            self.robot_gripper_clients[robot_name],
            request,
            f'{robot_name} gripper {command}',
        )

    def _publish_motion(self) -> None:
        if (
            not self.axes
            or self._last_joy_at is None
            or monotonic() - self._last_joy_at >= self.joy_timeout
        ):
            if not self._input_stopped:
                self._stop_all_motion()
                self._input_stopped = True
            return
        self._input_stopped = False
        if self.mode == 'robot':
            self._publish_robot_motion()
        else:
            self._publish_drone_motion()

    def _publish_robot_motion(self) -> None:
        name = self.current_robot_name
        if name is None:
            return

        deadman = self._button_down(self.BTN_L1)
        motion_enabled = (not self.require_deadman) or deadman
        gimbal_layer = self._button_down(self.BTN_R1)

        left_x = self._axis(self.AXIS_LEFT_STICK_X)
        left_y = self._axis(self.AXIS_LEFT_STICK_Y)
        right_x = self._axis(self.AXIS_RIGHT_STICK_X)
        right_y = self._axis(self.AXIS_RIGHT_STICK_Y)

        twist = Twist()
        if motion_enabled:
            twist.linear.x = left_y * self.robot_max_linear
            twist.linear.y = -left_x * self.robot_max_linear
            twist.angular.z = 0.0 if gimbal_layer else -right_x * self.robot_max_yaw
        self.robot_cmd_pubs[name].publish(twist)

        gimbal = GimbalVel()
        if motion_enabled and gimbal_layer:
            gimbal.pitch_speed = -right_y * self.gimbal_max_speed
            gimbal.yaw_speed = right_x * self.gimbal_max_speed
        else:
            gimbal.pitch_speed = 0.0
            gimbal.yaw_speed = 0.0
        self.robot_gimbal_pubs[name].publish(gimbal)

        dpad_x = self._axis(self.AXIS_DPAD_LEFT_RIGHT, apply_deadzone=False)
        dpad_y = self._axis(self.AXIS_DPAD_UP_DOWN, apply_deadzone=False)
        arm_twist = Twist()
        if motion_enabled and (abs(dpad_x) > 0.5 or abs(dpad_y) > 0.5):
            arm_twist.linear.x = float(dpad_y) * self.arm_max_linear_x
            arm_twist.linear.z = float(dpad_x) * self.arm_max_linear_z
        self.robot_arm_cmd_pubs[name].publish(arm_twist)

    def _publish_drone_motion(self) -> None:
        name = self.current_drone_name
        if name is None:
            return

        deadman = self._button_down(self.BTN_L1)
        motion_enabled = (not self.require_deadman) or deadman

        left_x = self._axis(self.AXIS_LEFT_STICK_X)
        left_y = self._axis(self.AXIS_LEFT_STICK_Y)
        right_x = self._axis(self.AXIS_RIGHT_STICK_X)

        up = 1.0 if self._axis(self.AXIS_L2, apply_deadzone=True) > 0.5 else 0.0
        down = 1.0 if self._axis(self.AXIS_R2, apply_deadzone=True) > 0.5 else 0.0

        twist = Twist()
        if motion_enabled:
            twist.linear.x = -left_y * self.drone_max_linear
            twist.linear.y = left_x * self.drone_max_linear
            twist.linear.z = (up - down) * self.drone_max_linear
            twist.angular.z = right_x * self.drone_max_yaw
        self.drone_cmd_pubs[name].publish(twist)

    def _stop_robot_motion(self, name: str) -> None:
        self.robot_cmd_pubs[name].publish(Twist())
        self.robot_gimbal_pubs[name].publish(GimbalVel())
        self.robot_arm_cmd_pubs[name].publish(Twist())

    def _stop_drone_motion(self, name: str) -> None:
        self.drone_cmd_pubs[name].publish(Twist())

    def _stop_all_motion(self) -> None:
        for name in self.robot_names:
            self._stop_robot_motion(name)
        for name in self.drone_names:
            self._stop_drone_motion(name)

    def stop_motion(self) -> None:
        self.motion_timer.cancel()
        self.axes = []
        self.buttons = []
        if self.context.ok():
            self._stop_all_motion()

    def _call_service(self, client, request, label: str, log_success: bool = True):
        if not client.wait_for_service(timeout_sec=0.0):
            self._throttled_warn(f'service:{label}', f'Service not ready: {label}')
            return None
        future = client.call_async(request)
        future.add_done_callback(
            lambda fut, desc=label, ok_log=log_success: self._log_service_result(fut, desc, ok_log)
        )
        return future

    def _log_service_result(self, future, label: str, log_success: bool) -> None:
        response = self._future_response(future)
        if response is None:
            return

        success = bool(getattr(response, 'success', True))
        message = str(getattr(response, 'message', '')).strip()
        if success:
            if log_success:
                if message:
                    self._status_info(f'{label}: {message}')
                else:
                    self._status_info(f'{label}: success')
        else:
            if message:
                self._status_warn(f'{label}: {message}')
            else:
                self._status_warn(f'{label}: failed')

    def _future_response(self, future):
        try:
            return future.result()
        except Exception as exc:
            self._clear_status_line()
            self.get_logger().error(f'Service call failed: {exc}')
            return None

    def _axis(self, index: int, apply_deadzone: bool = True) -> float:
        if index >= len(self.axes):
            return 0.0
        value = float(self.axes[index])
        if apply_deadzone and abs(value) < self.deadzone:
            return 0.0
        return value

    def _button_down(self, index: int) -> bool:
        return index < len(self.buttons) and self.buttons[index] == 1

    def _just_pressed(self, index: int, buttons: List[int]) -> bool:
        now = index < len(buttons) and buttons[index] == 1
        prev = index < len(self.prev_buttons) and self.prev_buttons[index] == 1
        return now and not prev

    def _throttled_warn(self, key: str, message: str, period_sec: float = 2.0) -> None:
        now_ns = self.get_clock().now().nanoseconds
        last_ns = self.last_warn_ns.get(key, 0)
        if now_ns - last_ns > int(period_sec * 1e9):
            self.last_warn_ns[key] = now_ns
            self._status_warn(message)

    def _status_info(self, message: str) -> None:
        self._emit_status_line('INFO', message)

    def _status_warn(self, message: str) -> None:
        self._emit_status_line('WARN', message)

    def _emit_status_line(self, level: str, message: str) -> None:
        if self.stream_status_line and sys.stdout.isatty():
            text = f'[{level}] {message}'
            padded = text.ljust(self._status_line_width)
            sys.stdout.write(f'\r{padded}')
            sys.stdout.flush()
            self._status_line_width = max(self._status_line_width, len(text))
            return
        if level == 'WARN':
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def _clear_status_line(self) -> None:
        if self.stream_status_line and self._status_line_width > 0 and sys.stdout.isatty():
            sys.stdout.write(f"\r{' ' * self._status_line_width}\r")
            sys.stdout.flush()
            self._status_line_width = 0

    def _active_target_summary(self) -> str:
        if self.mode == 'robot':
            if not self.robot_names:
                return 'robot:none'
            return f'robot:{self.robot_names[self.active_robot_idx % len(self.robot_names)]}'
        if not self.drone_names:
            return 'drone:none'
        return f'drone:{self.drone_names[self.active_drone_idx % len(self.drone_names)]}'

    def _print_controller_help(self) -> None:
        self._clear_status_line()
        if self.mode == 'robot':
            commands_title = f'Robot Commands ({self._active_target_summary()})'
            commands = self._robot_commands_legend()
        else:
            commands_title = f'Drone Commands ({self._active_target_summary()})'
            commands = self._drone_commands_legend()

        self.get_logger().info(
            '\n\nController Layout\n'
            f'{self._controller_ascii_art()}\n\n'
            f'{commands_title}\n'
            f'{commands}\n'
            f'{self._armed_status_line()}'
        )

    def _armed_status_line(self) -> str:
        if self.mode == 'robot':
            return self._format_armed_list('Robots', self.robot_names, self.robot_armed)
        return self._format_armed_list('Drones', self.drone_names, self.drone_armed)

    def _format_armed_list(self, label: str, names: List[str], armed_cache: Dict[str, bool]) -> str:
        if not names:
            return f'{label} Arm Status: none configured'

        entries: List[str] = []
        use_color = sys.stdout.isatty()
        for name in names:
            is_armed = armed_cache.get(name, False)
            state = 'ARMED' if is_armed else 'DISARMED'
            if use_color:
                color = self.ANSI_GREEN if is_armed else self.ANSI_RED
                state = f'{color}{state}{self.ANSI_RESET}'
            entries.append(f'{name}:{state}')
        return f'{label} Arm Status: ' + ' | '.join(entries)

    def _controller_ascii_art(self) -> str:
        return (
            '''\n
                        [L2]                     [R2]\n
                  .---------.               .---------.\n
                _/  [L1]    \\_____________/    [R1]   \\_\n
               /                                             \\\n
              |          ( LS )               ( RS )          |\n
              |           [SELECT]        [START]            |\n
              |   [DPAD]                      [X]   [Y]      |\n
              |                                [B]   [A]    |\n
               \\___________________________________________/\n
            '''
        )

    def _robot_commands_legend(self) -> str:
        return (
                '''\n
            L1: deadman (hold to move)       \tR1 + RS: gimbal yaw/pitch
            START: arm/disarm active robot   \tSELECT: cycle active robot
            Left Stick (LS): chassis X/Y     \tRight Stick X (RS-X): chassis yaw
            DPAD: arm_cmd_vel (X/Z),
            B: gripper close,
            A: gripper open,
            X: gripper reset,
            Y: gripper pause,
            '''
        )

    def _drone_commands_legend(self) -> str:
        return (
            '''\n
            L1: deadman (hold to move)       \tL2: ascend
            START: arm/disarm active drone   \tR2: descend
            SELECT: cycle active drone       \tRight Stick X (RS-X): yaw
            Left Stick (LS): planar X/Y      \tX: takeoff
            Y: land                          \tA: hover
            '''
        )

    @property
    def current_robot_name(self) -> str | None:
        if not self.robot_names:
            self._throttled_warn('robot_names', 'No robot names configured.')
            return None
        return self.robot_names[self.active_robot_idx % len(self.robot_names)]

    @property
    def current_drone_name(self) -> str | None:
        if not self.drone_names:
            self._throttled_warn('drone_names', 'No drone names configured.')
            return None
        return self.drone_names[self.active_drone_idx % len(self.drone_names)]


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    previous_handlers = {}

    def request_stop(_signal_number, _frame):
        raise KeyboardInterrupt

    try:
        node = JoyFleetTeleop()
        for signal_number in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signal_number] = signal.signal(signal_number, request_stop)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for signal_number, handler in previous_handlers.items():
            signal.signal(signal_number, handler)
        try:
            if node is not None:
                try:
                    node.stop_motion()
                finally:
                    node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
