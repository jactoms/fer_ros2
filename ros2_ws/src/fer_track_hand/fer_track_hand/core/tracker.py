"""
Hand tracking, free of ROS.

The tracker (a cube) is driven to sit side by side with the hand (a sphere): face to face, at
the same height, on one horizontal side of the hand, with centre to centre distance `gap`
(hand radius + cube half size = 2 * 0.04 m). The side (+x, -x, +y or -y) is fixed when
tracking starts: `side` from the configuration, or with 'auto' the side the tracker comes
from. Fixing it keeps the cube from swinging around through the hand.

The command velocity is the estimated hand velocity (feedforward, so it keeps up with a
moving hand) plus a proportional term on the remaining offset, limited to max_speed.

Success: the centre distance stays within gap +- tolerance for hold_time, i.e. side by side and
moving along with the hand. The state drops back to TRACKING when the distance leaves the band.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

Vec3 = Tuple[float, float, float]

WAITING = 'WAITING'
TRACKING = 'TRACKING'
SUCCESS = 'SUCCESS'

SIDES = {
    '+x': (1.0, 0.0, 0.0),
    '-x': (-1.0, 0.0, 0.0),
    '+y': (0.0, 1.0, 0.0),
    '-y': (0.0, -1.0, 0.0),
}


@dataclass
class TrackerConfig:
    """Parameters of the tracker."""

    gap: float = 0.08                # m, centre distance when side by side (touching)
    side: str = 'auto'               # auto | +x | -x | +y | -y, seen from the hand
    gain: float = 2.0                # 1/s, on the offset to the side-by-side position
    max_speed: float = 0.25          # m/s, command speed limit
    velocity_time_constant: float = 0.05  # s, low-pass of the hand velocity estimate
    tolerance: float = 0.01          # m, allowed |distance - gap|
    hold_time: float = 2.0           # s, in the band this long = SUCCESS

    def __post_init__(self) -> None:
        """Validate."""
        for name in ('gap', 'gain', 'max_speed', 'tolerance'):
            if getattr(self, name) <= 0.0:
                raise ValueError(f'{name} must be > 0')
        if self.velocity_time_constant < 0.0 or self.hold_time < 0.0:
            raise ValueError('velocity_time_constant and hold_time must be >= 0')
        if self.side != 'auto' and self.side not in SIDES:
            raise ValueError(f"side must be 'auto' or one of {tuple(SIDES)}, got '{self.side}'")


def distance(a: Sequence[float], b: Sequence[float]) -> float:
    """Euclidean distance."""
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def nearest_side(hand: Sequence[float], tracker: Sequence[float]) -> Vec3:
    """Horizontal axis direction from the hand towards the tracker (-y if right above it)."""
    dx, dy = tracker[0] - hand[0], tracker[1] - hand[1]
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return SIDES['-y']
    if abs(dx) > abs(dy):
        return SIDES['+x'] if dx > 0.0 else SIDES['-x']
    return SIDES['+y'] if dy > 0.0 else SIDES['-y']


def side_by_side(hand: Sequence[float], side: Sequence[float], gap: float) -> Vec3:
    """Tracker centre side by side with the hand: `gap` from it along `side`."""
    return tuple(h + s * gap for h, s in zip(hand, side))


class HandTracker:
    """Tracker command and success state; call step() once per hand sample."""

    def __init__(self, cfg: TrackerConfig) -> None:
        self.cfg = cfg
        self.state = WAITING
        self.command: Optional[Vec3] = None
        self.side: Optional[Vec3] = None   # fixed at the first step
        self.hand_velocity: Vec3 = (0.0, 0.0, 0.0)
        self.distance: Optional[float] = None
        self.success_time: Optional[float] = None  # first time SUCCESS was reached
        self._hand_prev: Optional[Vec3] = None
        self._in_band_since: Optional[float] = None

    @property
    def gap(self) -> float:
        """Centre to centre distance when side by side."""
        return self.cfg.gap

    def observe_hand(self, hand: Sequence[float], dt: float) -> None:
        """Update the hand velocity estimate (finite difference, first-order low-pass)."""
        hand = tuple(hand)
        if self._hand_prev is not None and dt > 0.0:
            raw = [(h - p) / dt for h, p in zip(hand, self._hand_prev)]
            a = dt / (self.cfg.velocity_time_constant + dt)
            self.hand_velocity = tuple(v + a * (r - v) for v, r in zip(self.hand_velocity, raw))
        self._hand_prev = hand

    def step(self, t: float, dt: float, hand: Sequence[float],
             tracker: Sequence[float]) -> Vec3:
        """
        Advance the command by dt and update the success state.

        t: time (s), dt: time since the last step (s), hand / tracker: measured centres.
        Returns the new tracker command.
        """
        if self.command is None:  # first step: start from where the tracker is
            self.command = tuple(tracker)
            self.side = (nearest_side(hand, tracker) if self.cfg.side == 'auto'
                         else SIDES[self.cfg.side])
            self.state = TRACKING
        desired = side_by_side(hand, self.side, self.gap)
        v = [hv + self.cfg.gain * (d - c)
             for hv, d, c in zip(self.hand_velocity, desired, self.command)]
        speed = math.sqrt(sum(x * x for x in v))
        if speed > self.cfg.max_speed:
            v = [x * self.cfg.max_speed / speed for x in v]
        self.command = tuple(c + x * dt for c, x in zip(self.command, v))
        self._update_state(t, distance(hand, tracker))
        return self.command

    def _update_state(self, t: float, d: float) -> None:
        self.distance = d
        if abs(d - self.gap) > self.cfg.tolerance:
            self._in_band_since = None
            self.state = TRACKING
            return
        if self._in_band_since is None:
            self._in_band_since = t
        if t - self._in_band_since >= self.cfg.hold_time:
            self.state = SUCCESS
            if self.success_time is None:
                self.success_time = t
