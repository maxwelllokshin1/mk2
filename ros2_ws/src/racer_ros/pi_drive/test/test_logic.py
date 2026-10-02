import math

from pi_drive.mixer import Inputs, Mixer, MixerConfig, apply_deadzone
from pi_drive.pwm import PulseMap, ThrottleShaper

M = Mixer(MixerConfig())
BOTH = dict(have_steer=True, have_buttons=True)


def mix(**kw):
    auto = {k: kw.pop(k) for k in ('auto_steer_rad', 'auto_speed', 'auto_age_s') if k in kw}
    return M.mix(Inputs(**BOTH, **kw), **auto)


def test_no_controller_is_all_zero():
    assert M.mix(Inputs(), 0.3, 1.0, 0.0) == (0.0, 0.0, 'NO_CONTROLLER')


def test_rc_idle():
    assert mix() == (0.0, 0.0, 'RC')


def test_a_forward_b_reverse_both_stop():
    assert mix(a=True)[1] == M.cfg.rc_forward
    assert mix(b=True)[1] == -M.cfg.rc_reverse
    assert mix(a=True, b=True)[1] == 0.0


def test_stick_deadzone_and_sign():
    assert mix(stick=0.05)[0] == 0.0
    assert mix(stick=-1.0)[0] == 1.0     # stick left -> steering + (left)
    assert mix(stick=1.0)[0] == -1.0
    assert apply_deadzone(1.0, 0.12) == 1.0


def test_autonomy_while_r_held():
    s, t, mode = mix(r=True, auto_steer_rad=0.26, auto_speed=3.0, auto_age_s=0.1)
    assert mode == 'AUTO' and math.isclose(s, 0.5) and t == M.cfg.auto_min_throttle  # 3 of 15 m/s -> floor
    s, t, _ = mix(r=True, auto_steer_rad=-9.0, auto_speed=-1.0, auto_age_s=0.1)
    assert s == -1.0 and t == -M.cfg.auto_reverse_throttle   # clamped, reverse recovery


def test_autonomy_speed_follows_reactive_node():
    slow = mix(r=True, auto_speed=3.0, auto_age_s=0.0)[1]
    mid = mix(r=True, auto_speed=11.0, auto_age_s=0.0)[1]
    fast = mix(r=True, auto_speed=15.0, auto_age_s=0.0)[1]
    over = mix(r=True, auto_speed=99.0, auto_age_s=0.0)[1]
    assert slow < mid < fast == over == M.cfg.auto_max_throttle      # ceiling holds
    fixed = Mixer(MixerConfig(auto_speed_ref_mps=0.0))
    assert fixed.mix(Inputs(have_steer=True, have_buttons=True, r=True), 0.0, 9.0, 0.0)[1] == 0.25


def test_autonomy_ignores_a_b_and_needs_fresh_drive():
    assert mix(r=True, a=True, auto_age_s=9.0) == (0.0, 0.0, 'AUTO_WAITING')
    assert mix(r=True, a=True, auto_speed=0.0, auto_age_s=0.0)[1] == 0.0


def test_releasing_r_returns_to_rc():
    assert mix(r=True, auto_speed=3.0, auto_age_s=0.0)[2] == 'AUTO'
    assert mix(r=False, a=False, auto_speed=3.0, auto_age_s=0.0) == (0.0, 0.0, 'RC')


def test_only_one_joycon():
    s, t, mode = M.mix(Inputs(have_steer=True, stick=-1.0, a=True, r=True))
    assert (s, t, mode) == (1.0, 0.0, 'RC')   # buttons joy-con missing -> no throttle, no autonomy


def test_pulse_map():
    p = PulseMap()
    assert p.steering_to_us(0.0) == 1500
    assert p.steering_to_us(1.0) == 1700 and p.steering_to_us(-1.0) == 1300
    assert p.steering_to_us(5.0) == 1700                       # clamped
    assert PulseMap(steer_sign=-1.0).steering_to_us(1.0) == 1300
    assert p.throttle_to_us(0.0) == 1500
    assert p.throttle_to_us(0.001) > 1525 - 1e-6               # jumps the ESC deadband
    assert p.throttle_to_us(1.0) == 1700 and p.throttle_to_us(-1.0) == 1300
    assert PulseMap(steer_min_us=1600, steer_max_us=1400).steering_to_us(0) == 1500


def test_shaper_ramps_up_drops_instantly():
    s = ThrottleShaper(slew_per_s=2.0, reverse_neutral_s=0.25)
    assert math.isclose(s.step(1.0, 0.1), 0.2)
    assert math.isclose(s.step(1.0, 0.1), 0.4)
    assert s.step(0.0, 0.1) == 0.0


def test_shaper_reverse_goes_through_neutral():
    s = ThrottleShaper(slew_per_s=100.0, reverse_neutral_s=0.25)
    assert s.step(0.3, 0.02) == 0.3
    assert s.step(-0.3, 0.02) == 0.0          # forced to neutral first
    for _ in range(5):                         # 0.1 s < 0.25 s hold
        assert s.step(-0.3, 0.02) == 0.0
    out = [s.step(-0.3, 0.02) for _ in range(20)]
    assert out[-1] == -0.3 and out[0] == 0.0


def test_shaper_same_direction_not_blocked():
    s = ThrottleShaper(slew_per_s=100.0)
    s.step(0.3, 0.02)
    s.step(0.0, 0.02)
    assert s.step(0.3, 0.02) == 0.3
