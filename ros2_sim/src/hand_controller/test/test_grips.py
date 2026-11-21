import numpy as np

from hand_controller.grips import GESTURE_TO_GRIP, GRIPS, OBJECT_WIDTH, GripPlanner, select_grip


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



# --- automatic grip selection (hand pose + nearest object) -------------------------------------------------------------
def test_object_decides_the_grip_for_a_closing_hand():
    for gesture in ("Fist", "Pinch"):                       # either closing pose: the object decides
        assert select_grip(gesture, ("ball", 0.13)) == "power"
        assert select_grip(gesture, ("cup", 0.13)) == "power"
        assert select_grip(gesture, ("pen", 0.09)) == "pinch"


def test_gesture_decides_when_no_object_is_in_reach():
    assert select_grip("Fist", None) == "power"
    assert select_grip("Pinch", None) == "pinch"
    assert select_grip("Fist", ("ball", 0.5)) == "power"    # too far away: ignored
    assert select_grip("Pinch", ("cup", 0.5)) == "pinch"


def test_open_hand_releases_and_other_gestures_keep_their_grip():
    assert select_grip("Open Hand", ("ball", 0.1)) == "open"
    assert select_grip("Pointing", ("ball", 0.1)) == "point"
    assert select_grip("No hand", ("ball", 0.1)) is None


def test_unknown_object_falls_back_to_the_gesture():
    assert select_grip("Fist", ("brick", 0.1)) == "power"


def test_object_table():
    assert OBJECT_WIDTH["pen"] < OBJECT_WIDTH["ball"] < OBJECT_WIDTH["cup"]


# --- contact by blocking, verification and retries ------------------------------------------------------------------------
def test_a_blocked_finger_stops_without_a_tip_force():
    p = GripPlanner()
    p.command("power")
    for _ in range(int(0.7 / 0.02)):
        p.step(0.02, np.zeros(5))
    blocked = np.array([False, True, False, False, False])
    for _ in range(100):
        p.step(0.02, np.zeros(5), blocked)
    stopped = p.target[1]
    for _ in range(100):
        p.step(0.02, np.zeros(5), blocked)
    assert p.target[1] == stopped < 1.0           # the blocked index finger no longer advances
    assert p.target[2] > stopped                  # the free ones kept closing


def test_verified_grasp_with_enough_contacts_holds():
    p = GripPlanner()
    p.command("power", verify=True)
    blocked = np.array([False, True, True, False, False])
    ph = None
    for _ in range(int(8 / 0.02)):
        _, ph = p.step(0.02, np.zeros(5), blocked)
    assert ph == "holding" and p.attempt == 1


def test_verified_grasp_that_touches_nothing_retries_then_fails_open():
    p = GripPlanner(max_attempts=3)
    p.command("power", verify=True)
    phases = set()
    for _ in range(int(30 / 0.02)):
        _, ph = p.step(0.02, np.zeros(5))
        phases.add(ph)
    assert p.attempt == 3 and ph == "failed" and "holding" not in phases
    np.testing.assert_allclose(p.target, 0.0, atol=1e-6)      # open again


def test_second_attempt_can_succeed():
    p = GripPlanner(max_attempts=3)
    p.command("pinch", verify=True)
    ph = None
    for k in range(int(30 / 0.02)):
        touching = p.attempt >= 2                          # the object only gets touched on the second try
        _, ph = p.step(0.02, np.zeros(5), np.array([False, touching, False, False, touching]))
        if ph == "holding":
            break
    assert ph == "holding" and p.attempt == 2


def test_unverified_grasp_in_free_air_still_just_closes():
    p = GripPlanner()
    p.command("power")                                     # no object near: no check
    for _ in range(int(5 / 0.02)):
        _, ph = p.step(0.02, np.zeros(5))
    assert ph == "holding"


def test_a_finger_that_loses_its_support_closes_on():
    """Held with the index finger blocked; then the support goes away (the hand lifted): the finger keeps closing."""
    p = GripPlanner(regrip=True)
    p.command("power")
    blocked = np.array([False, True, False, False, False])
    for _ in range(int(5 / 0.02)):
        p.step(0.02, np.zeros(5), blocked)
    assert p.phase == "holding"
    held = p.target[1]
    assert held < 1.0
    for _ in range(int(1.0 / 0.02)):                       # still blocked: stays put
        p.step(0.02, np.zeros(5), blocked)
    assert p.target[1] == held
    for _ in range(int(3.0 / 0.02)):                       # free now
        p.step(0.02, np.zeros(5), np.zeros(5, bool))
    assert p.target[1] > held and p.phase == "holding"


def test_regrip_is_off_by_default():
    p = GripPlanner()
    p.command("power")
    blocked = np.array([False, True, False, False, False])
    for _ in range(int(5 / 0.02)):
        p.step(0.02, np.zeros(5), blocked)
    held = p.target[1]
    for _ in range(int(3.0 / 0.02)):
        p.step(0.02, np.zeros(5), np.zeros(5, bool))
    assert p.target[1] == held
