"""
Closed-loop tests of the tracker against each fer_hand_mjc motion (no ROS, no MuJoCo).

The hand moves with fer_hand_mjc's own HandMotion (its defaults); the tracker cube is stepped at
100 Hz from its scene position, as track_hand_node does once tracking starts.
"""
from fer_hand_mjc.motions import HandMotion
from fer_track_hand.core.tracker import (
    distance, HandTracker, nearest_side, side_by_side, SIDES, SUCCESS, TrackerConfig,
    TRACKING, WAITING)
import pytest

DT = 0.01
START = (0.5, 0.0, 0.5)
TRACKER_START = (0.2, -0.6, 0.2)  # hand_scene.xml


def hand_motion(motion, seed=0):
    """fer_hand_mjc's hand motion (its defaults), sampled every DT."""
    m = HandMotion(motion=motion, start=START, random_seed=seed)
    return lambda t: tuple(m.position(t, DT))


def run(hand_at, duration=30.0, cfg=None, tracker_start=TRACKER_START):
    """Track for `duration` s; return the tracker and the (t, hand, tracker) history."""
    tracker = HandTracker(cfg or TrackerConfig())
    position = tracker_start
    history = []
    for k in range(int(duration / DT)):
        t = k * DT
        hand = hand_at(t)
        tracker.observe_hand(hand, DT if k else 0.0)
        position = tracker.step(t, DT, hand, position)
        history.append((t, hand, position))
    return tracker, history


CASES = [('static', 0), ('sine', 0)] + [('random', seed) for seed in range(5)]


@pytest.mark.parametrize('motion, seed', CASES)
def test_reaches_success_and_moves_along(motion, seed):
    tracker, history = run(hand_motion(motion, seed))
    name = f'{motion} (seed {seed})'
    assert tracker.success_time is not None, f'{name}: no SUCCESS'
    assert tracker.success_time < 8.0, f'{name}: SUCCESS only at {tracker.success_time:.2f} s'
    # After success the cube stays side by side: in the band all the time.
    err = [abs(distance(h, p) - tracker.gap) for t, h, p in history if t >= tracker.success_time]
    assert max(err) <= tracker.cfg.tolerance, f'{name}: max error {1000 * max(err):.1f} mm'
    # Never overlapping the hand by more than the tolerance.
    assert min(distance(h, p) for _, h, p in history) > tracker.gap - tracker.cfg.tolerance


@pytest.mark.parametrize('motion, seed', CASES)
def test_cube_stays_on_its_side(motion, seed):
    tracker, history = run(hand_motion(motion, seed))
    assert tracker.side == SIDES['-y']  # the cube starts at -y of the hand area
    for t, h, p in history:
        if t >= tracker.success_time:
            assert p[1] - h[1] == pytest.approx(-tracker.gap, abs=tracker.cfg.tolerance)


def test_fixed_side_is_used():
    tracker, history = run(hand_motion('static'), cfg=TrackerConfig(side='+x'), duration=15.0)
    _, h, p = history[-1]
    assert tracker.side == SIDES['+x']
    assert p == pytest.approx((h[0] + 0.08, h[1], h[2]), abs=1e-3)


def test_speed_is_limited_and_faster_than_the_hand():
    cfg = TrackerConfig()
    tracker, history = run(hand_motion('sine'), cfg=cfg)
    steps = [distance(a[2], b[2]) / DT for a, b in zip(history, history[1:])]
    assert max(steps) <= cfg.max_speed + 1e-9
    hand_speed = max(distance(a[1], b[1]) / DT for a, b in zip(history, history[1:]))
    assert hand_speed < cfg.max_speed


def test_success_needs_hold_time():
    tracker, history = run(hand_motion('static'), duration=15.0)
    first_in_band = next(
        t for t, h, p in history if abs(distance(h, p) - tracker.gap) <= tracker.cfg.tolerance)
    assert tracker.success_time == pytest.approx(
        first_in_band + tracker.cfg.hold_time, abs=2 * DT)


def test_state_drops_back_when_hand_jumps():
    tracker, _ = run(hand_motion('static'), duration=15.0)
    assert tracker.state == SUCCESS
    jumped = (START[0], START[1] + 0.3, START[2])
    tracker.observe_hand(jumped, DT)
    tracker.step(15.0, DT, jumped, tracker.command)
    assert tracker.state == TRACKING


def test_starts_waiting():
    tracker = HandTracker(TrackerConfig())
    assert tracker.state == WAITING and tracker.command is None


def test_nearest_side_and_side_by_side():
    assert nearest_side((0.0, 0.0, 0.0), (0.3, -0.1, 0.5)) == SIDES['+x']
    assert nearest_side((0.0, 0.0, 0.0), (0.1, -0.3, 0.0)) == SIDES['-y']
    assert nearest_side((0.0, 0.0, 0.0), (0.0, 0.0, 1.0)) == SIDES['-y']  # right above
    assert side_by_side((0.5, 0.0, 0.5), SIDES['-y'], 0.08) == pytest.approx((0.5, -0.08, 0.5))


@pytest.mark.parametrize('kwargs', [{'gap': 0.0}, {'max_speed': -1.0}, {'side': 'up'}])
def test_invalid_config_rejected(kwargs):
    with pytest.raises(ValueError):
        TrackerConfig(**kwargs)
