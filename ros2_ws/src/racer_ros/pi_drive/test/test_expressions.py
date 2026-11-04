import random

from pi_drive.expressions import (CLOSING, DRIVE_POOL, EXPRESSIONS, IDLE_POOL, Director, ShuffleBag, Timing,
                                  blinks, eye_plan)


def run(director, seconds, driving=False, person=False, start=0.0, dt=0.1):
    """Step the director; returns (events, end_time). A line 'plays' for 3 s."""
    events, t, speak_until = [], start, -1.0
    while t < start + seconds:
        for e in director.step(t, driving, person, t < speak_until):
            events.append((t, e))
            if e[0] == 'say':
                speak_until = t + 3.0
        t += dt
    return events, t


def names(events):
    return {e[1] for _, e in events if e[0] == 'expression'}


def test_blink_table():
    assert [n for n in ('normal', 'excited', 'sarcastic') if blinks(n)] == ['normal', 'excited', 'sarcastic']
    assert not any(blinks(n) for n in ('surprised', 'dizzy', 'sleepy', 'angry', 'bored'))


def test_parked_faces_are_the_idle_pool():
    d = Director(rng=random.Random(1))
    ev, _ = run(d, 6000)
    assert names(ev) == set(IDLE_POOL)


def test_driving_faces_are_the_driving_pool_only():
    d = Director(rng=random.Random(2))
    ev, _ = run(d, 3000, driving=True)
    assert names(ev) == set(DRIVE_POOL)
    assert not names(ev) & {'dizzy', 'surprised', 'excited', 'sleepy', 'bored'}


def test_dizzy_only_ever_comes_from_a_collision():
    d = Director(rng=random.Random(3))
    ev, t = run(d, 3000)
    ev2, t = run(d, 3000, driving=True, start=t)
    assert 'dizzy' not in names(ev) | names(ev2)
    d.force(t, 'dizzy')
    ev3 = d.step(t, True, False, False)
    assert ('expression', 'dizzy') in ev3 and ('say', 'dizzy', True) in ev3


def test_surprised_forced_while_driving_then_back_to_driving_faces():
    d = Director(rng=random.Random(4))
    _, t = run(d, 30, driving=True)
    d.force(t, 'surprised')
    ev, _ = run(d, 60, driving=True, start=t)
    shown = [e[1] for _, e in ev if e[0] == 'expression']
    assert shown[0] == 'surprised' and set(shown[1:]) <= set(DRIVE_POOL)


def test_dizzy_beats_surprised_but_not_the_other_way_round():
    d = Director(rng=random.Random(5))
    _, t = run(d, 10, driving=True)
    d.force(t, 'dizzy')
    d.step(t, True, False, False)
    d.force(t + 0.5, 'surprised')
    assert d.step(t + 0.5, True, False, False) == [] and d.expression == 'dizzy'


def test_lid_faces_close_before_they_end():
    assert set(CLOSING) == {'angry', 'bored'}
    d = Director(rng=random.Random(6))
    ev, _ = run(d, 4000)
    kinds = [e for _, e in ev if e[0] in ('expression', 'closing')]
    for prev, cur in zip(kinds, kinds[1:]):
        if prev[0] == 'expression' and prev[1] in CLOSING:
            assert cur == ('closing', prev[1])                 # a lid face is always followed by its closing event
        if cur[0] == 'closing':
            assert prev == ('expression', cur[1])


def test_closing_gives_the_lid_time_to_reverse():
    d = Director(Timing(close_s=1.5), rng=random.Random(7))
    ev, _ = run(d, 4000)
    last_closing, seen = None, 0
    for t, e in ev:
        if e[0] == 'closing':
            last_closing = t
        elif e[0] == 'expression' and last_closing is not None:
            assert t - last_closing >= 1.4
            last_closing, seen = None, seen + 1
    assert seen > 0


def test_lines_come_from_the_face_that_is_showing():
    d = Director(rng=random.Random(3))
    shown, bad = None, 0
    t, speak_until = 0.0, -1.0
    while t < 2000:
        for e in d.step(t, t > 1000, False, t < speak_until):
            if e[0] == 'expression':
                shown = e[1]
            elif e[0] == 'say':
                bad += e[1] != shown
                speak_until = t + 3.0
        t += 0.1
    assert bad == 0


def test_face_never_changes_while_speaking():
    d = Director(rng=random.Random(4))
    t, speak_until = 0.0, -1.0
    while t < 2000:
        speaking = t < speak_until
        for e in d.step(t, False, False, speaking):
            assert not (e[0] == 'expression' and speaking)
            if e[0] == 'say':
                speak_until = t + 8.0
        t += 0.1


def test_lap_triggers_excited_and_interrupts():
    d = Director(rng=random.Random(5))
    _, t = run(d, 30, driving=True)
    d.excite(t)
    ev = d.step(t, True, False, True)
    assert ('expression', 'excited') in ev and ('say', 'excited', True) in ev


def test_excited_after_a_long_drive_but_not_a_short_one():
    d = Director(Timing(excited_after_drive_s=60.0), rng=random.Random(6))
    _, t = run(d, 90, driving=True)
    ev, _ = run(d, 5, driving=False, start=t)
    assert 'excited' in names(ev)
    d = Director(rng=random.Random(6))
    _, t = run(d, 20, driving=True)
    ev, _ = run(d, 5, driving=False, start=t)
    assert 'excited' not in names(ev)


def test_excited_ends_and_goes_back_to_normal_faces():
    d = Director(rng=random.Random(7))
    _, t = run(d, 5)
    d.excite(t)
    ev, _ = run(d, 60, start=t)
    shown = [e[1] for _, e in ev if e[0] == 'expression']
    assert shown[0] == 'excited' and set(shown[1:]) <= set(IDLE_POOL)


def test_person_makes_him_talk_more():
    def lines(person):
        d = Director(rng=random.Random(8))
        ev, _ = run(d, 1200, person=person)
        return sum(e[0] == 'say' for _, e in ev)
    assert lines(True) > lines(False)


def test_shuffle_bag_plays_everything_before_repeating():
    bag, files = ShuffleBag(random.Random(9)), ['a', 'b', 'c']
    for _ in range(5):
        assert sorted(bag.next(files) for _ in range(3)) == files


def test_every_face_builds_on_the_normal_eyeball_and_blinking_ones_borrow_its_blink(tmp_path):
    for name, spec in EXPRESSIONS.items():
        (tmp_path / ('blinking' if spec.blinks else 'static') / name / 'eyes').mkdir(parents=True, exist_ok=True)
    normal = tmp_path / 'blinking' / 'normal' / 'eyes'
    for f in ('base.png', 'iris.png', 'blink.gif'):
        (normal / f).write_bytes(b'x')
    (tmp_path / 'blinking' / 'excited' / 'eyes' / 'excited.gif').write_bytes(b'x')
    (tmp_path / 'blinking' / 'excited' / 'eyes' / 'excited_2.png').write_bytes(b'x')
    (tmp_path / 'static' / 'angry' / 'eyes' / 'eyelid.gif').write_bytes(b'x')
    for name, spec in EXPRESSIONS.items():
        plan = eye_plan(str(tmp_path), name)
        assert plan['base'] == str(normal / 'base.png')                       # eyeball under every expression
        assert bool(plan['blink']) == spec.blinks
        assert plan['blink'] is None or plan['blink'] == str(normal / 'blink.gif')
    assert len(eye_plan(str(tmp_path), 'excited')['variants']) == 2
    assert eye_plan(str(tmp_path), 'angry')['iris'] == str(normal / 'iris.png')    # the iris is normal's
    assert eye_plan(str(tmp_path), 'angry')['lid'].endswith('eyelid.gif')
    assert eye_plan(str(tmp_path), 'dizzy')['iris'] is None
