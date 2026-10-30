"""
Hand motions, free of ROS and MuJoCo.

Used by the fer_hand_mjc node and by the fer_track_hand tests, so both move the hand the same
way. Call HandMotion.position() once per physics step, in order: the random motion advances
by random_speed * dt on every call.
"""
from __future__ import annotations

import math
import random
from typing import List, Sequence

MOTIONS = ('static', 'sine', 'random')


class HandMotion:
    """
    Scripted hand position.

    static: hold start; sine: oscillate around start with sine_amplitude and sine_frequency;
    random: move at random_speed to seeded random waypoints inside start +- random_range,
    a new one on arrival (same random_seed, same path).
    """

    def __init__(
        self,
        motion: str = 'sine',
        start: Sequence[float] = (0.5, 0.0, 0.5),
        sine_amplitude: Sequence[float] = (0.1, 0.3, 0.15),
        sine_frequency: float = 0.08,
        random_range: Sequence[float] = (0.25, 0.35, 0.2),
        random_speed: float = 0.1,
        random_seed: int = 0,
    ) -> None:
        if motion not in MOTIONS:
            raise ValueError(f"motion must be one of {MOTIONS}, got '{motion}'")
        if len(start) != 3 or len(sine_amplitude) != 3 or len(random_range) != 3:
            raise ValueError(
                'start_position, sine_amplitude and random_range must have 3 elements')
        if random_speed <= 0.0:
            raise ValueError('random_speed must be > 0')
        self.motion = motion
        self._start = list(start)
        self._amplitude = list(sine_amplitude)
        self._frequency = sine_frequency
        self._range = list(random_range)
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
        return [s + self._rng.uniform(-r, r) for s, r in zip(self._start, self._range)]

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
