import os
import random

from pi_drive.expressions import (CLOSING, DRIVE_POOL, EXPRESSIONS, IDLE_POOL, Director, ShuffleBag, Timing,
                                  blinks, eye_plan, find_voice_files)


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
    assert set(CLOSING) == {'angry'}
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


def test_voice_files_accept_m4a_and_mp3(tmp_path):
    voice = tmp_path / 'static' / 'angry' / 'voice'
    voice.mkdir(parents=True)
    for f in ('a.mp3', 'b.M4A', 'c.wav', 'README.txt', 'd.png'):
        (voice / f).write_bytes(b'x')
    assert [os.path.basename(f) for f in find_voice_files(str(tmp_path), 'angry')] == ['a.mp3', 'b.M4A', 'c.wav']


def drive_director(director, seconds, line_s=3.0, driving=False, person=False, start=0.0, dt=0.1):
    """Like run(), but returns [(t, event)] and lets every line last line_s seconds."""
    events, t, speak_until = [], start, -1.0
    while t < start + seconds:
        for e in director.step(t, driving, person, t < speak_until):
            events.append((t, e))
            if e[0] == 'say':
                speak_until = t + line_s
        t += dt
    return events


def test_every_expression_says_one_line_within_its_first_six_seconds():
    d = Director(rng=random.Random(11))
    ev = drive_director(d, 4000)
    shown = [(t, e[1]) for t, e in ev if e[0] == 'expression']
    says = [(t, e[1]) for t, e in ev if e[0] == 'say']
    assert len(shown) > 50
    for (t0, name), nxt in zip(shown, shown[1:] + [(1e9, None)]):
        lines = [t for t, who in says if t0 <= t < nxt[0] and who == name]
        assert len(lines) == 1, (name, t0, lines)               # exactly one line per expression
        assert lines[0] - t0 <= d.t.say_window_s + 0.2          # ... within the first six seconds


def test_the_moment_of_the_line_is_random():
    delays = set()
    for seed in range(20):
        ev = drive_director(Director(rng=random.Random(seed)), 60)
        first = next(t for t, e in ev if e[0] == 'expression')
        say = next(t for t, e in ev if e[0] == 'say')
        delays.add(round(say - first, 1))
    assert len(delays) > 10


def test_forced_faces_speak_at_once():
    d = Director(rng=random.Random(12))
    drive_director(d, 20)
    d.force(100.0, 'surprised')
    ev = d.step(100.0, False, False, False)
    assert ev == [('expression', 'surprised'), ('say', 'surprised', True)]


def test_bored_lasts_as_long_as_its_gif_and_its_line():
    for line_s in (2.0, 14.0):                                  # a short line, and one longer than the gif
        d = Director(Timing(), rng=random.Random(13), lengths={'bored': 6.2})
        ev = drive_director(d, 8000, line_s=line_s)
        seen = 0
        for (t0, e), (t1, nxt) in zip(ev, ev[1:]):
            if e == ('expression', 'bored'):
                seen += 1
                start = t0
                end = next(t for t, x in ev if x[0] == 'expression' and t > start)
                say = next(t for t, x in ev if x == ('say', 'bored', False) and t >= start)
                assert end - start >= 6.2 - 0.1                     # the gif plays through
                assert end >= say + line_s - 0.15                   # the line is never cut off
                assert end - start <= max(6.2, say - start + line_s) + 0.5     # and the next face follows right after
        assert seen > 3


def test_bored_and_surprised_are_plain_gifs():
    assert EXPRESSIONS['bored'].loop == ('bored',) and EXPRESSIONS['bored'].once and not EXPRESSIONS['bored'].lid
    assert EXPRESSIONS['surprised'].loop == ('surprised',) and not EXPRESSIONS['surprised'].lid
