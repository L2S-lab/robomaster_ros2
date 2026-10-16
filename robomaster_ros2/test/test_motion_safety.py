"""Unit tests for the ROS-independent motion safety supervisor."""

from math import inf, nan

from robomaster_ros2.modules.motion_safety import MotionSafety, SafetyState


class FakeClock:
    """Controllable monotonic clock for deadman tests."""

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def test_disarmed_gate_requires_explicit_arm():
    safety = MotionSafety(initially_armed=False)

    rejected = safety.authorize('flight', (0.2, 0.0))
    assert not rejected.accepted
    assert rejected.reason == 'motion is disarmed'

    assert safety.arm()
    accepted = safety.authorize('flight', (0.2, 0.0))
    assert accepted.accepted
    assert safety.state == SafetyState.ACTIVE


def test_non_finite_values_are_rejected_without_refreshing_deadman():
    clock = FakeClock()
    safety = MotionSafety(default_timeout=0.5, clock=clock)
    assert safety.authorize('chassis', (0.5, 0.0)).accepted

    clock.advance(0.4)
    assert not safety.authorize('chassis', (nan, 0.0)).accepted
    assert not safety.authorize('chassis', (inf, 0.0)).accepted

    clock.advance(0.1)
    assert safety.poll_deadman() == ('chassis',)


def test_limits_are_clamped_without_mutating_the_input():
    safety = MotionSafety()
    source_values = (2.0, -3.0, 0.25)

    decision = safety.authorize(
        'flight',
        source_values,
        limits=((-1.0, 1.0), (-1.0, 1.0), (-1.0, 1.0)),
    )

    assert decision.accepted
    assert decision.clamped
    assert decision.values == (1.0, -1.0, 0.25)
    assert source_values == (2.0, -3.0, 0.25)


def test_deadman_trips_once_and_non_latched_gate_can_resume():
    clock = FakeClock()
    safety = MotionSafety(default_timeout=0.5, clock=clock)
    assert safety.authorize('flight', (0.4, 0.0)).accepted

    clock.advance(0.49)
    assert safety.poll_deadman() == ()
    clock.advance(0.01)
    assert safety.poll_deadman() == ('flight',)
    assert safety.poll_deadman() == ()
    assert safety.state == SafetyState.FAILSAFE
    assert safety.armed

    assert safety.authorize('flight', (0.1, 0.0)).accepted
    assert safety.state == SafetyState.ACTIVE


def test_latched_deadman_stops_all_channels_and_requires_rearm():
    clock = FakeClock()
    safety = MotionSafety(
        default_timeout=0.5,
        latch_deadman=True,
        clock=clock,
    )
    assert safety.authorize('chassis', (0.4,)).accepted
    clock.advance(0.2)
    assert safety.authorize('gimbal', (10.0,)).accepted

    clock.advance(0.3)
    assert set(safety.poll_deadman()) == {'chassis', 'gimbal'}
    assert not safety.armed
    assert safety.state == SafetyState.DISARMED
    assert not safety.authorize('chassis', (0.1,)).accepted

    assert safety.arm()
    assert safety.authorize('chassis', (0.1,)).accepted


def test_zero_command_releases_channel_without_future_timeout():
    clock = FakeClock()
    safety = MotionSafety(default_timeout=0.5, clock=clock)
    assert safety.authorize('chassis', (0.5, 0.0)).accepted
    assert safety.authorize('chassis', (0.0, 0.0)).accepted
    assert safety.state == SafetyState.READY

    clock.advance(1.0)
    assert safety.poll_deadman() == ()


def test_channel_owner_prevents_competing_sources_until_release():
    safety = MotionSafety()
    assert safety.authorize(
        'chassis', (0.5,), source='trajectory'
    ).accepted

    rejected = safety.authorize('chassis', (0.5,), source='joystick')
    assert not rejected.accepted
    assert 'another command source' in rejected.reason

    assert safety.authorize(
        'chassis', (0.0,), source='trajectory'
    ).accepted
    assert safety.authorize('chassis', (0.5,), source='joystick').accepted


def test_driver_state_can_reject_motion_without_refreshing_command():
    clock = FakeClock()
    safety = MotionSafety(default_timeout=0.5, clock=clock)
    assert safety.authorize('flight', (0.3,)).accepted

    clock.advance(0.4)
    decision = safety.authorize(
        'flight', (0.3,), motion_allowed=False
    )
    assert not decision.accepted
    assert 'current state' in decision.reason

    clock.advance(0.1)
    assert safety.poll_deadman() == ('flight',)
