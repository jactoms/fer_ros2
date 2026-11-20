"""
Hand motions, free of ROS and MuJoCo.

Used by the fer_hand_mjc node and by the fer_track_hand tests, so both move the hand the same
way. Call HandMotion.position() once per physics step, in order: the random motion advances
by random_speed * dt on every call.
"""
from __future__ import annotations

import math
import random
from typing import List, Optional, Sequence

MOTIONS = ('static', 'sine', 'random')


class HandMotion:
    """
    Scripted hand position.

    static: hold start; sine: oscillate around start with sine_amplitude and sine_frequency;
    random: move at random_speed to seeded random waypoints inside random_center +- random_range
    (random_center defaults to start), a new one on arrival (same random_seed, same path).
    random_keepout_radius > 0 keeps the hand centre out of the vertical cylinder of that radius
    around the z axis (the robot base): a waypoint is drawn again until it lies outside and the
    straight path to it does not cross the cylinder.
    """

    def __init__(
        self,
        motion: str = 'sine',
        start: Sequence[float] = (0.45, 0.0, 0.35),
        sine_amplitude: Sequence[float] = (0.1, 0.25, 0.04),
        sine_frequency: float = 0.08,
        random_range: Sequence[float] = (0.1, 0.25, 0.04),
        random_speed: float = 0.1,
        random_seed: int = 0,
        random_center: Optional[Sequence[float]] = None,
        random_keepout_radius: float = 0.0,
    ) -> None:
        if motion not in MOTIONS:
            raise ValueError(f"motion must be one of {MOTIONS}, got '{motion}'")
        center = start if random_center is None else random_center
        if (len(start) != 3 or len(sine_amplitude) != 3 or len(random_range) != 3
                or len(center) != 3):
            raise ValueError('start_position, sine_amplitude, random_range and random_center '
                             'must have 3 elements')
        if random_speed <= 0.0:
            raise ValueError('random_speed must be > 0')
        if random_keepout_radius < 0.0:
            raise ValueError('random_keepout_radius must be >= 0')
        if motion == 'random' and math.hypot(start[0], start[1]) <= random_keepout_radius:
            raise ValueError('start_position lies inside random_keepout_radius')
        self.motion = motion
        self._start = list(start)
        self._amplitude = list(sine_amplitude)
        self._frequency = sine_frequency
        self._range = list(random_range)
        self._center = list(center)
        self._keepout = random_keepout_radius
        self._speed = random_speed
        self._rng = random.Random(random_seed)
        self._random_pos = list(self._start)
        self._random_goal = self._new_goal()

    def position(self, t: float, dt: float) -> List[float]:
        """Hand position at time t (s since start); dt is the time since the last call."""
        if self.motion == 'sine':
            s = math.sin(2.0 * math.pi * self._frequency * t)
            return [p + a * s for p, a in zip(self._start, self._amplitude)]
        if self.motion == 'random':
            return self._random_step(dt)
        return list(self._start)

    def _new_goal(self) -> List[float]:
        """Next waypoint: random in the box, reachable without crossing the keep-out."""
        for _ in range(1000):
            goal = [c + self._rng.uniform(-r, r) for c, r in zip(self._center, self._range)]
            if self._clear_of_keepout(self._random_pos, goal):
                return goal
        raise RuntimeError('no waypoint outside random_keepout_radius after 1000 draws; '
                           'the keep-out covers (nearly) the whole random box')

    def _clear_of_keepout(self, a: Sequence[float], b: Sequence[float]) -> bool:
        """Return True if the segment a-b stays outside the keep-out cylinder (x-y distance)."""
        if self._keepout <= 0.0:
            return True
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx * dx + dy * dy
        t = 0.0 if length2 == 0.0 else min(1.0, max(0.0, -(a[0] * dx + a[1] * dy) / length2))
        return math.hypot(a[0] + t * dx, a[1] + t * dy) > self._keepout

    def _random_step(self, dt: float) -> List[float]:
        """Move towards the waypoint by speed * dt; pick the next one on arrival."""
        to_goal = [g - p for g, p in zip(self._random_goal, self._random_pos)]
        distance = math.sqrt(sum(d * d for d in to_goal))
        step = self._speed * dt
        if distance <= step:
            self._random_pos = list(self._random_goal)
            self._random_goal = self._new_goal()
        else:
            self._random_pos = [p + d * step / distance for p, d in zip(self._random_pos, to_goal)]
        return list(self._random_pos)
