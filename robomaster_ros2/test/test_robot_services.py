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

"""Focused regression tests for robot feature callbacks and planners."""

import struct
import threading

from robomaster_interface.srv import (
    BlasterLED,
    GimbalLED,
    GripperCommand,
    MoveArm,
    RobotLED,
    SetArmorSensitivity,
    SetGripperMode,
)
from std_srvs.srv import Trigger

from robomaster_ros2.modules.action import ACTION_ABORTED, ActionDispatcher
from robomaster_ros2.modules.robot import (
    ArmorHitEvent,
    BatterySubject,
    IrHitEvent,
    Robot,
    RoboticArmMoveAction,
)


def test_robot_led_callback_returns_populated_response():
    """A successful LED command must return its ROS response object."""
    robot = Robot.__new__(Robot)
    robot._set_led = lambda **_kwargs: True
    robot._close_robot = lambda: None
    request = RobotLED.Request()
    request.on = True
    request.which = 'all'
    request.led.b = 32.0
    request.effect = 'none'
    request.freq = 1
    response = RobotLED.Response()

    result = Robot.set_led(robot, request, response)

    assert result is response
    assert result.success
    assert result.message == 'LED set successfully'


def test_position_push_is_forwarded_to_ros_publisher():
    """Each chassis push must trigger publication without a polling timer."""
    calls = []
    robot = Robot.__new__(Robot)
    robot._close_robot = lambda: None
    robot.pub_position = lambda: calls.append(robot.position_data)

    Robot.get_position_callback(robot, (1.25, -0.5, 45.0))

    assert robot.position_data == [1.25, -0.5, 45.0]
    assert calls == [[1.25, -0.5, 45.0]]


def test_gimbal_push_is_forwarded_to_ros_publisher():
    """Each gimbal push must trigger publication without a polling timer."""
    calls = []
    robot = Robot.__new__(Robot)
    robot._close_robot = lambda: None
    robot.pub_gimbal_angle = lambda: calls.append(robot.gimbal_angle_data)

    Robot.get_gimbal_angle_callback(robot, (5.0, 10.0, 15.0, 20.0))

    assert robot.gimbal_angle_data == [5.0, 10.0, 15.0, 20.0]
    assert calls == [[5.0, 10.0, 15.0, 20.0]]


def test_gimbal_led_callback_passes_selection_to_led_module():
    """Selective LED requests must preserve component, color, and indices."""
    calls = []
    robot = Robot.__new__(Robot)
    robot._close_robot = lambda: None
    robot.led_control = type('LedControl', (), {
        'set_gimbal_led': lambda _self, **kwargs: calls.append(kwargs) or True,
    })()
    request = GimbalLED.Request()
    request.on = True
    request.which = 'top_left'
    request.led.r = 255.0
    request.led.g = 16.0
    request.led.b = 8.0
    request.led_list = [0, 2, 4, 6]
    response = GimbalLED.Response()

    result = Robot.set_gimbal_led(robot, request, response)

    assert result is response
    assert result.success
    assert calls[0]['comp'] == 'top_left'
    assert calls[0]['led_list'] == [0, 2, 4, 6]


def test_gimbal_led_callback_rejects_duplicate_indices():
    """Duplicate indices must not corrupt the DJI bit mask."""
    robot = Robot.__new__(Robot)
    robot._close_robot = lambda: None
    robot.led_control = None
    request = GimbalLED.Request()
    request.which = 'top_all'
    request.led_list = [1, 1]
    response = GimbalLED.Response()

    result = Robot.set_gimbal_led(robot, request, response)

    assert not result.success
    assert result.message == 'Gimbal LED indices must not be repeated.'


def test_blaster_led_callback_returns_command_result():
    """Blaster brightness and power state must reach the protocol helper."""
    calls = []
    robot = Robot.__new__(Robot)
    robot._close_robot = lambda: None
    robot._set_blaster_led = (
        lambda **kwargs: calls.append(kwargs) or True
    )
    request = BlasterLED.Request()
    request.on = True
    request.brightness = 128
    response = BlasterLED.Response()

    result = Robot.set_blaster_led(robot, request, response)

    assert result.success
    assert calls == [{'on': True, 'brightness': 128}]


def test_ep_battery_subject_and_trigger_use_percentage_format():
    """EP battery DDS data must match the drone Trigger response format."""
    subject = BatterySubject()
    subject.decode(struct.pack('<HhiBB', 12000, 250, -300, 73, 0))
    assert subject.data_info() == 73

    robot = Robot.__new__(Robot)
    robot._close_robot = lambda: None
    robot.battery_percent = subject.data_info()
    response = Trigger.Response()

    result = Robot.get_battery(robot, Trigger.Request(), response)

    assert result.success
    assert result.message == '73%'


def test_ep_battery_trigger_reports_missing_first_sample():
    """The service must not invent a battery value before DDS publishes one."""
    robot = Robot.__new__(Robot)
    robot._close_robot = lambda: None
    robot.battery_percent = None

    result = Robot.get_battery(
        robot, Trigger.Request(), Trigger.Response()
    )

    assert not result.success
    assert 'first DDS sample' in result.message


def test_armor_subjects_expose_component_strength_and_ir_metadata():
    """Armor events must preserve all fields available from DJI."""
    armor = ArmorHitEvent()
    armor.decode([3, 0, 321])
    assert armor.data_info() == (3, 'bottom_left', 'water', 321)

    ir = IrHitEvent()
    ir.decode([4, 2, 7, 1])
    ir.decode([5, 3, 8, 2])
    assert ir.data_info() == (2, 5, 3, 8, 2)


def test_armor_sensitivity_uses_dji_component_mask_and_coefficients():
    """Sensitivity requests must reproduce DJI's armor parameter mapping."""
    sent = []
    robot = Robot.__new__(Robot)
    robot._close_robot = lambda: None
    robot._send_sync_proto = (
        lambda proto, target: sent.append((proto, target)) or True
    )
    request = SetArmorSensitivity.Request()
    request.component = 'top_left'
    request.sensitivity = 5

    result = Robot.set_armor_sensitivity(
        robot, request, SetArmorSensitivity.Response()
    )

    assert result.success
    proto, _target = sent[0]
    assert proto._armor_mask == 1 << 5
    assert proto._voice_peak_min == 160
    assert proto._voice_peak_ave == 180
    assert proto._voice_peak_final == 200


def _make_gripper_planner():
    robot = Robot.__new__(Robot)
    robot._close_robot = lambda: None
    robot.gripper_position = None
    robot.gripper_full_travel_time = 1.25
    return robot


def test_gripper_unknown_position_requires_endpoint_calibration():
    """Intermediate timed moves must be rejected until an endpoint is known."""
    robot = _make_gripper_planner()

    duration, valid, direction = Robot.calculate_gripper_movement(robot, 50)

    assert duration == 0.0
    assert not valid
    assert direction == 0


def test_gripper_timing_preserves_calibrated_percentage_table():
    """Full-travel time must scale the existing timing table."""
    robot = _make_gripper_planner()
    robot.gripper_position = 25

    duration, valid, direction = Robot.calculate_gripper_movement(robot, 75)

    assert valid
    assert direction == 1
    assert round(duration, 2) == 0.63


def test_gripper_mode_can_switch_only_while_idle():
    """Manual timed/feedback selection must reject changes during motion."""
    robot = _make_gripper_planner()
    robot._gripper_lock = threading.RLock()
    robot.gripper_busy = False
    robot.gripper_control_mode = 'timed'
    robot.gripper_message = ''
    robot.publish_gripper_state = lambda: None
    request = SetGripperMode.Request()
    request.mode = 'feedback'

    result = Robot.set_gripper_mode(
        robot, request, SetGripperMode.Response()
    )

    assert result.success
    assert robot.gripper_control_mode == 'feedback'

    robot.gripper_busy = True
    request.mode = 'timed'
    result = Robot.set_gripper_mode(
        robot, request, SetGripperMode.Response()
    )
    assert not result.success
    assert robot.gripper_control_mode == 'feedback'


def test_gripper_feedback_calibrates_only_reported_endpoints():
    """Opened/closed DDS states must calibrate the percentage estimate."""
    robot = _make_gripper_planner()
    robot._gripper_lock = threading.RLock()
    robot._gripper_feedback_event = threading.Event()
    robot.gripper_status = 'unknown'
    robot.publish_gripper_state = lambda: None

    Robot.get_gripper_status_callback(robot, 'opened')

    assert robot.gripper_status == 'opened'
    assert robot.gripper_position == 100
    assert robot._gripper_feedback_event.is_set()


def test_gripper_open_uses_configured_power_when_request_is_zero():
    """Power zero must select the configured default."""
    calls = []
    robot = _make_gripper_planner()
    robot._start_gripper_position_command = (
        lambda distance, power: calls.append((distance, power))
        or (True, 'accepted')
    )
    request = GripperCommand.Request()
    request.command = 'open'
    request.power = 0

    result = Robot.gripper_command(
        robot, request, GripperCommand.Response()
    )

    assert result.success
    assert calls == [(100, None)]


def test_cancelled_gripper_worker_never_starts_directional_motion():
    """A cancellation won before worker start must send only a pause."""
    calls = []
    robot = _make_gripper_planner()
    robot._gripper_lock = threading.RLock()
    robot._gripper = (
        lambda position, power, async_flag: calls.append(
            (position, power, async_flag)
        ) or True
    )
    robot.gripper_last_command_success = False
    robot.gripper_busy = True
    robot.gripper_message = ''
    robot.publish_gripper_state = lambda: None
    cancel_event = threading.Event()
    cancel_event.set()
    robot._gripper_cancel_event = cancel_event
    robot._gripper_worker = object()

    Robot._run_gripper_position_command(
        robot, 100, 1, 1.25, 50, 'timed', cancel_event
    )

    assert calls == [(0, 0, False)]
    assert not robot.gripper_busy
    assert not robot.gripper_last_command_success


def test_paused_gripper_motion_invalidates_percentage_estimate():
    """A partial physical move must not retain its stale starting estimate."""
    calls = []
    cancel_event = threading.Event()
    robot = _make_gripper_planner()
    robot.gripper_position = 25
    robot._gripper_lock = threading.RLock()

    def send_gripper(position, power, async_flag):
        calls.append((position, power, async_flag))
        if position != 0:
            cancel_event.set()
        return True

    robot._gripper = send_gripper
    robot.gripper_last_command_success = False
    robot.gripper_busy = True
    robot.gripper_message = ''
    robot.publish_gripper_state = lambda: None
    robot._gripper_cancel_event = cancel_event
    robot._gripper_worker = object()

    Robot._run_gripper_position_command(
        robot, 100, 1, 1.25, 50, 'timed', cancel_event
    )

    assert calls == [(1, 50, False), (0, 0, False)]
    assert robot.gripper_position is None
    assert 'recalibrate' in robot.gripper_message


class _FakeArmDispatcher:
    def __init__(self):
        self.action = None

    def send_action(self, action):
        action._action_id = 7
        self.action = action
        return action


def _make_arm_robot():
    robot = Robot.__new__(Robot)
    robot._close_robot = lambda: None
    robot._arm_lock = threading.RLock()
    robot.current_arm_action = None
    robot.current_arm_command = 'idle'
    robot.arm_action_pub = None
    robot.arm_completion_timeout = 10.0
    robot._conf = type('Config', (), {'_name': 'rmep_1'})()
    robot._action_dispatcher = _FakeArmDispatcher()
    return robot


def test_relative_and_absolute_arm_services_select_dji_modes():
    """move and moveto must use DJI action modes zero and one respectively."""
    robot = _make_arm_robot()
    relative = MoveArm.Request()
    relative.x = 0.1
    relative.z = 0.05

    relative_result = Robot.move_arm(
        robot, relative, MoveArm.Response()
    )

    assert relative_result.success
    assert robot._action_dispatcher.action._mode == 0
    assert robot._action_dispatcher.action._x == 100.0
    assert robot._action_dispatcher.action._y == 50.0

    robot.current_arm_action._percent = 100
    absolute = MoveArm.Request()
    absolute.x = 0.12
    absolute.z = -0.04

    absolute_result = Robot.move_arm_to(
        robot, absolute, MoveArm.Response()
    )

    assert absolute_result.success
    assert robot._action_dispatcher.action._mode == 1
    assert robot._action_dispatcher.action._x == 120.0
    assert robot._action_dispatcher.action._y == -40.0


def test_arm_push_updates_completion_without_nonexistent_z_field():
    """The two-axis DJI arm push must update progress without a z lookup."""
    action = RoboticArmMoveAction(x=10, y=20, z=0)
    proto = action._push_proto_cls()
    proto._action_id = 7
    proto._percent = 100
    proto._action_state = 1
    proto._x = 10
    proto._y = 20

    action.update_from_push(proto)

    assert action.is_completed
    assert action.has_succeeded
    assert action._percent == 100
    assert action._x == 10
    assert action._y == 20


def test_absolute_arm_service_rejects_out_of_range_target():
    """Absolute moveto must reject unsafe input rather than clamp it."""
    robot = _make_arm_robot()
    request = MoveArm.Request()
    request.x = 0.3

    result = Robot.move_arm_to(robot, request, MoveArm.Response())

    assert not result.success
    assert robot._action_dispatcher.action is None


def test_arm_service_rejects_unbounded_wait_before_motion():
    """A non-finite completion wait must never start arm motion."""
    robot = _make_arm_robot()
    request = MoveArm.Request()
    request.x = 0.01
    request.wait_for_completion = True
    request.timeout = float('inf')

    result = Robot.move_arm(robot, request, MoveArm.Response())

    assert not result.success
    assert 'finite' in result.message
    assert robot._action_dispatcher.action is None


def test_arm_cancel_service_reports_dispatch_result():
    """Cancellation must target the currently tracked arm action."""
    robot = _make_arm_robot()
    action = type('ArmAction', (), {
        'is_completed': False,
        '_action_id': 9,
    })()
    robot.current_arm_action = action
    robot._cancel_arm_action = lambda selected: selected is action

    result = Robot.cancel_arm(robot, Trigger.Request(), Trigger.Response())

    assert result.success
    assert '9' in result.message


def test_arm_completion_fields_report_terminal_action_state():
    """MoveArm responses must expose progress and terminal state."""
    robot = _make_arm_robot()
    request = MoveArm.Request()
    request.x = 0.01
    result = Robot.move_arm(robot, request, MoveArm.Response())
    action = robot.current_arm_action
    action._percent = 100
    action._state = 'action_succeeded'

    completed = Robot._fill_arm_response(
        robot, MoveArm.Response(), action, True, 'complete'
    )

    assert result.action_id == 7
    assert completed.completed
    assert completed.percent == 100
    assert completed.state == 'action_succeeded'


def test_recenter_and_reset_start_absolute_origin_action():
    """Both lifecycle services must use an absolute origin command."""
    calls = []
    robot = _make_arm_robot()
    fake_action = type('ArmAction', (), {'_action_id': 11})()
    robot._start_arm_action = (
        lambda **kwargs: calls.append(kwargs)
        or (fake_action, 'accepted')
    )

    recenter = Robot.recenter_arm(
        robot, Trigger.Request(), Trigger.Response()
    )

    assert recenter.success
    assert calls[-1] == {
        'x': 0.0,
        'z': 0.0,
        'absolute': True,
        'command': 'recenter',
    }

    robot.current_arm_action = None
    reset = Robot.reset_arm(robot, Trigger.Request(), Trigger.Response())

    assert reset.success
    assert calls[-1] == {
        'x': 0.0,
        'z': 0.0,
        'absolute': True,
        'command': 'reset',
    }


def test_action_dispatcher_encodes_and_retires_arm_cancellation():
    """Cancellation must set DJI action_ctrl and retire the tracked action."""
    sent = []

    class FakeClient:
        hostbyte = 1

        def send_sync_msg(self, msg, timeout):
            sent.append((msg, timeout))
            response_proto = type('ResponseProto', (), {'_retcode': 0})()
            return type('Response', (), {
                'get_proto': lambda _self: response_proto,
            })()

    dispatcher = ActionDispatcher(FakeClient())
    action = RoboticArmMoveAction(x=10, z=5)
    action._action_id = 7
    action._obj = dispatcher
    action._on_state_changed = dispatcher._on_action_state_changed
    dispatcher._in_progress[action.make_action_key()] = action

    result = dispatcher.cancel_action(action)

    assert result
    assert sent[0][0].get_proto()._action_ctrl == 1
    assert sent[0][0].get_proto()._action_id == 7
    assert action.state == ACTION_ABORTED
    assert dispatcher._in_progress == {}
