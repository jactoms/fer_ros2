"""Unit tests of the hand motions (no ROS, no MuJoCo)."""
import math

from fer_hand_mjc.motions import HandMotion
import pytest

DT = 0.002
START = (0.5, 0.0, 0.5)


def test_static_holds_start():
    m = HandMotion(motion='static', start=START)
    assert m.position(0.0, DT) == pytest.approx(START)
    assert m.position(10.0, DT) == pytest.approx(START)


def test_sine_oscillates_around_start():
    m = HandMotion(motion='sine', start=START, sine_amplitude=(0.0, 0.15, 0.05),
                   sine_frequency=0.2)
    assert m.position(0.0, DT) == pytest.approx(START)
    assert m.position(1.25, DT) == pytest.approx((0.5, 0.15, 0.55))  # quarter period


# fer_moveit follow_workspace_min/max and follow_max_speed.
FOLLOW_BOX = ((0.25, -0.30, 0.25), (0.60, 0.30, 0.50))
FOLLOW_MAX_SPEED = 0.25
ABOVE = 0.1  # the gripper 10 cm above the hand


def _above_inside_the_follow_box(path):
    lo, hi = FOLLOW_BOX
    return all(lo[i] <= p[i] + (ABOVE if i == 2 else 0.0) <= hi[i] for p in path for i in range(3))


def test_defaults_keep_the_gripper_above_the_hand_inside_the_follow_box():
    sine = HandMotion()
    path = [sine.position(k * DT, DT) for k in range(int(12.5 / DT))]  # one period
    span = [max(p[i] for p in path) - min(p[i] for p in path) for i in range(3)]
    assert span == pytest.approx([0.2, 0.5, 0.08], abs=1e-3)
    assert _above_inside_the_follow_box(path)
    speed = max(math.dist(p, q) for p, q in zip(path, path[1:])) / DT
    assert speed < FOLLOW_MAX_SPEED
    rnd = HandMotion(motion='random')
    path = [rnd.position(k * DT, DT) for k in range(int(60.0 / DT))]
    span = [max(p[i] for p in path) - min(p[i] for p in path) for i in range(3)]
    assert all(s > 0.5 * 2 * r for s, r in zip(span, (0.1, 0.25, 0.04)))
    assert _above_inside_the_follow_box(path)


def test_random_stays_in_box_at_speed_and_is_seeded():
    a = HandMotion(motion='random', start=START, random_range=(0.2, 0.2, 0.15),
                   random_speed=0.1, random_seed=3)
    b = HandMotion(motion='random', start=START, random_range=(0.2, 0.2, 0.15),
                   random_speed=0.1, random_seed=3)
    path = [a.position(k * DT, DT) for k in range(10000)]  # 20 s
    assert path == [b.position(k * DT, DT) for k in range(10000)]
    for p in path:
        assert abs(p[0] - 0.5) <= 0.2 + 1e-9
        assert abs(p[1]) <= 0.2 + 1e-9
        assert abs(p[2] - 0.5) <= 0.15 + 1e-9
    steps = [math.dist(p, q) for p, q in zip(path, path[1:])]
    assert max(steps) <= 0.1 * DT + 1e-12
    assert sum(steps) == pytest.approx(0.1 * DT * len(steps), rel=0.02)


@pytest.mark.parametrize('kwargs', [
    {'motion': 'teleport'},
    {'start': (0.5, 0.0)},
    {'random_speed': 0.0},
])
def test_invalid_rejected(kwargs):
    with pytest.raises(ValueError):
        HandMotion(**kwargs)
