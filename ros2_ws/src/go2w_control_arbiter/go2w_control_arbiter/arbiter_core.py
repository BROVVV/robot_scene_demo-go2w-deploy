from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Twist2D:
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0

    @property
    def is_zero(self) -> bool:
        return self.linear_x == 0.0 and self.linear_y == 0.0 and self.angular_z == 0.0


@dataclass(frozen=True)
class CommandSample:
    command: Twist2D
    received_at: float


@dataclass(frozen=True)
class Selection:
    command: Twist2D
    source: str
    blocked_reason: str | None


PRIORITY = ("manual", "nav2", "search")


def select_command(
    commands: dict[str, CommandSample],
    *,
    now: float,
    timeout: float,
    software_armed: bool,
    emergency_stop: bool,
    remote_override: bool,
) -> Selection:
    zero = Twist2D()
    if emergency_stop:
        return Selection(zero, "emergency_stop", "emergency_stop_active")
    if remote_override:
        return Selection(zero, "remote_override", "remote_override_active_or_unverified")
    if not software_armed:
        return Selection(zero, "disarmed", "software_control_not_armed")
    for source in PRIORITY:
        sample = commands.get(source)
        if sample is None or now - sample.received_at < 0.0 or now - sample.received_at > timeout:
            continue
        return Selection(
            Twist2D(sample.command.linear_x, 0.0, sample.command.angular_z),
            source,
            "linear_y_forced_zero" if sample.command.linear_y != 0.0 else None,
        )
    return Selection(zero, "watchdog", "all_command_sources_stale")
