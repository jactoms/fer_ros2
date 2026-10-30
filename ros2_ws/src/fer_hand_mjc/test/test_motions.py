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


def test_defaults_cover_a_wide_area_slower_than_the_tracker():
    sine = HandMotion(motion='sine', start=START)
    path = [sine.position(k * DT, DT) for k in range(int(12.5 / DT))]  # one period
    span = [max(p[i] for p in path) - min(p[i] for p in path) for i in range(3)]
    assert span == pytest.approx([0.2, 0.6, 0.3], abs=1e-3)
    speed = max(math.dist(p, q) for p, q in zip(path, path[1:])) / DT
    assert speed < 0.25  # fer_track_hand max_speed
    rnd = HandMotion(motion='random', start=START)
    path = [rnd.position(k * DT, DT) for k in range(int(60.0 / DT))]
    span = [max(p[i] for p in path) - min(p[i] for p in path) for i in range(3)]
    assert all(s > 0.5 * 2 * r for s, r in zip(span, (0.25, 0.35, 0.2)))


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
