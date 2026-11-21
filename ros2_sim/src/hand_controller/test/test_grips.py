import numpy as np

from hand_controller.grips import GESTURE_TO_GRIP, GRIPS, GripPlanner


def run(planner, seconds, force_of, dt=0.02):
    phase = planner.phase
    for _ in range(int(seconds / dt)):
        _, phase = planner.step(dt, force_of(planner.target))
    return planner.target.copy(), phase


def test_all_grips_in_range():
    for g in GRIPS.values():
        for pose in (g.preshape, g.close):
            assert len(pose) == 5 and all(0.0 <= v <= 1.0 for v in pose)
    assert set(GESTURE_TO_GRIP.values()) <= set(GRIPS)


def test_free_closing_reaches_final_pose():
    p = GripPlanner()
    p.command("power")
    target, phase = run(p, 4.0, lambda t: np.zeros(5))
    assert phase == "holding"
    np.testing.assert_allclose(target, GRIPS["power"].close, atol=1e-6)


def test_finger_stops_at_force_limit():
    p = GripPlanner(f_hold=2.0)
    p.command("power")
    # the index finger meets an object when its curl passes 0.6 (force grows with penetration)
    force = lambda t: np.array([0, 20.0 * max(0.0, t[1] - 0.6), 0, 0, 0])
    target, phase = run(p, 5.0, force)
    assert phase == "holding"
    assert 0.6 < target[1] < 0.8          # stopped on the object, not at 1.0
    assert target[0] == 1.0 and target[2] == 1.0   # free fingers closed fully


def test_backs_off_when_force_exceeds_max():
    p = GripPlanner(f_hold=2.0, f_max=5.0)
    p.command("power")
    run(p, 3.0, lambda t: np.zeros(5))
    before = p.target[1]
    for _ in range(25):
        p.step(0.02, [0, 9.0, 0, 0, 0])
    assert p.target[1] < before


def test_open_releases():
    p = GripPlanner()
    p.command("power")
    run(p, 4.0, lambda t: np.zeros(5))
    p.command("open", current=p.target)
    target, phase = run(p, 1.0, lambda t: np.zeros(5))
    assert phase == "idle" and np.allclose(target, 0.0)

