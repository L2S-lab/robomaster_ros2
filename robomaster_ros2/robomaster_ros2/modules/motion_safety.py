"""Shared motion-command safety and deadman supervision."""

from dataclasses import dataclass
from enum import Enum
from math import isfinite
from threading import RLock
from time import monotonic
from typing import Callable, Dict, Iterable, Optional, Sequence, Tuple


class SafetyState(Enum):
    """High-level state of the motion-command gate."""

    DISARMED = 'disarmed'
    READY = 'ready'
    ACTIVE = 'active'
    FAILSAFE = 'failsafe'
    EMERGENCY = 'emergency'
    CLOSED = 'closed'


@dataclass(frozen=True)
class CommandDecision:
    """Result returned when a motion command passes through the gate."""

    accepted: bool
    values: Tuple[float, ...]
    reason: str = ''
    clamped: bool = False


@dataclass
class _Channel:
    """Freshness and ownership information for one actuator channel."""

    active: bool = False
    last_command_at: Optional[float] = None
    owner: Optional[str] = None
    timeout: float = 0.0


class MotionSafety:
    """Authorize and supervise commands before they reach robot transports.

    The class deliberately has no ROS dependency. Drivers call
    :meth:`authorize` before sending a command and periodically call
    :meth:`poll_deadman`. The returned expired channel names tell the driver
    which safe command to send.
    """

    def __init__(
        self,
        default_timeout: float = 0.5,
        initially_armed: bool = True,
        latch_deadman: bool = False,
        clock: Optional[Callable[[], float]] = None,
    ) -> None:
        self._validate_timeout(default_timeout)
        self._default_timeout = float(default_timeout)
        self._latch_deadman = bool(latch_deadman)
        self._clock = clock or monotonic
        self._lock = RLock()
        self._channels: Dict[str, _Channel] = {}
        self._armed = bool(initially_armed)
        self._state = (
            SafetyState.READY if self._armed else SafetyState.DISARMED
        )

    @staticmethod
    def _validate_timeout(timeout: float) -> None:
        if not isfinite(float(timeout)) or float(timeout) < 0.0:
            raise ValueError('command timeout must be finite and non-negative')

    @property
    def armed(self) -> bool:
        """Return whether ordinary motion commands are currently accepted."""

        with self._lock:
            return self._armed

    @property
    def state(self) -> SafetyState:
        """Return the current state of the safety gate."""

        with self._lock:
            return self._state

    @property
    def default_timeout(self) -> float:
        """Return the configured default deadman timeout in seconds."""

        return self._default_timeout

    def arm(self) -> bool:
        """Arm the gate, clearing a previous non-closed safety trip."""

        with self._lock:
            if self._state == SafetyState.CLOSED:
                return False
            self._armed = True
            self._clear_channels_locked()
            self._state = SafetyState.READY
            return True

    def disarm(self) -> Tuple[str, ...]:
        """Disarm the gate and return channels that require a safe command."""

        with self._lock:
            active = self._active_channels_locked()
            self._armed = False
            self._clear_channels_locked()
            self._state = SafetyState.DISARMED
            return active

    def emergency(self) -> Tuple[str, ...]:
        """Latch emergency state and return active channels to stop."""

        with self._lock:
            active = self._active_channels_locked()
            self._armed = False
            self._clear_channels_locked()
            self._state = SafetyState.EMERGENCY
            return active

    def close(self) -> Tuple[str, ...]:
        """Permanently close the gate and return active channels to stop."""

        with self._lock:
            active = self._active_channels_locked()
            self._armed = False
            self._clear_channels_locked()
            self._state = SafetyState.CLOSED
            return active

    def mark_safe(self, channel: Optional[str] = None) -> None:
        """Mark one or all channels as already stopped."""

        with self._lock:
            if channel is None:
                self._clear_channels_locked()
            elif channel in self._channels:
                self._clear_channel_locked(self._channels[channel])
            self._refresh_state_locked()

    def authorize(
        self,
        channel: str,
        values: Iterable[float],
        *,
        limits: Optional[Sequence[Tuple[float, float]]] = None,
        source: str = 'default',
        motion_allowed: bool = True,
        timeout: Optional[float] = None,
    ) -> CommandDecision:
        """Validate, clamp, authorize and timestamp one motion command."""

        try:
            normalized = tuple(float(value) for value in values)
        except (TypeError, ValueError):
            return CommandDecision(
                False, (), 'command contains non-numeric data'
            )

        if not normalized:
            return CommandDecision(False, (), 'command has no values')
        if not all(isfinite(value) for value in normalized):
            return CommandDecision(False, normalized, 'command is not finite')
        if not channel or not source:
            return CommandDecision(
                False, normalized, 'channel and source must be non-empty'
            )

        resolved_timeout = (
            self._default_timeout if timeout is None else timeout
        )
        try:
            self._validate_timeout(resolved_timeout)
            adjusted, clamped = self._apply_limits(normalized, limits)
        except (TypeError, ValueError) as exc:
            return CommandDecision(False, normalized, str(exc))

        with self._lock:
            if self._state == SafetyState.CLOSED:
                return CommandDecision(
                    False, adjusted, 'safety gate is closed'
                )
            if self._state == SafetyState.EMERGENCY:
                return CommandDecision(False, adjusted, 'emergency is active')
            if not self._armed:
                return CommandDecision(False, adjusted, 'motion is disarmed')
            if not motion_allowed:
                return CommandDecision(
                    False,
                    adjusted,
                    'motion is not allowed in the current state',
                )

            now = self._clock()
            command_channel = self._channels.setdefault(channel, _Channel())
            if self._channel_expired_locked(command_channel, now):
                if self._latch_deadman:
                    self._armed = False
                    self._clear_channels_locked()
                    self._state = SafetyState.DISARMED
                    return CommandDecision(
                        False, adjusted, 'deadman timeout requires re-arming'
                    )
                self._clear_channel_locked(command_channel)

            if (
                command_channel.active
                and command_channel.owner is not None
                and command_channel.owner != source
            ):
                return CommandDecision(
                    False,
                    adjusted,
                    'motion channel is owned by another command source',
                )

            if all(value == 0.0 for value in adjusted):
                self._clear_channel_locked(command_channel)
                self._refresh_state_locked()
            else:
                command_channel.active = True
                command_channel.last_command_at = now
                command_channel.owner = source
                command_channel.timeout = float(resolved_timeout)
                self._state = SafetyState.ACTIVE

            return CommandDecision(True, adjusted, clamped=clamped)

    def poll_deadman(self) -> Tuple[str, ...]:
        """Return channels whose last non-zero command has expired."""

        with self._lock:
            now = self._clock()
            expired = tuple(
                name
                for name, channel in self._channels.items()
                if self._channel_expired_locked(channel, now)
            )
            if not expired:
                return ()

            if self._latch_deadman:
                to_stop = self._active_channels_locked()
                self._armed = False
                self._clear_channels_locked()
                self._state = SafetyState.DISARMED
                return to_stop

            for name in expired:
                self._clear_channel_locked(self._channels[name])
            self._state = (
                SafetyState.ACTIVE
                if self._active_channels_locked()
                else SafetyState.FAILSAFE
            )
            return expired

    def _channel_expired_locked(
        self, channel: _Channel, current_time: float
    ) -> bool:
        return bool(
            channel.active
            and channel.timeout > 0.0
            and channel.last_command_at is not None
            and current_time - channel.last_command_at >= channel.timeout
        )

    @staticmethod
    def _apply_limits(
        values: Tuple[float, ...],
        limits: Optional[Sequence[Tuple[float, float]]],
    ) -> Tuple[Tuple[float, ...], bool]:
        if limits is None:
            return values, False
        if len(limits) != len(values):
            raise ValueError('command limits do not match command values')

        adjusted = []
        clamped = False
        for value, limit in zip(values, limits):
            if len(limit) != 2:
                raise ValueError('each command limit must contain two values')
            lower = float(limit[0])
            upper = float(limit[1])
            if not isfinite(lower) or not isfinite(upper) or lower > upper:
                raise ValueError('command limits must be finite and ordered')
            bounded = min(max(value, lower), upper)
            adjusted.append(bounded)
            clamped = clamped or bounded != value
        return tuple(adjusted), clamped

    def _active_channels_locked(self) -> Tuple[str, ...]:
        return tuple(
            name for name, channel in self._channels.items() if channel.active
        )

    @staticmethod
    def _clear_channel_locked(channel: _Channel) -> None:
        channel.active = False
        channel.last_command_at = None
        channel.owner = None
        channel.timeout = 0.0

    def _clear_channels_locked(self) -> None:
        for channel in self._channels.values():
            self._clear_channel_locked(channel)

    def _refresh_state_locked(self) -> None:
        if self._state in (SafetyState.CLOSED, SafetyState.EMERGENCY):
            return
        if not self._armed:
            self._state = SafetyState.DISARMED
        elif self._active_channels_locked():
            self._state = SafetyState.ACTIVE
        else:
            self._state = SafetyState.READY
