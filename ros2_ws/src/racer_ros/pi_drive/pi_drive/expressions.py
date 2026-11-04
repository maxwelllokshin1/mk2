"""
expressions.py: the car's personality. Pure Python, no ROS, no PIL: nodes call it, tests drive it.

Every expression lives in its own folder, with its eyes and its voice lines together:

    config/expressions/
        blinking/<name>/eyes/  voice/      expressions that blink every 10 s (normal, excited, sarcastic)
        static/<name>/eyes/    voice/      expressions that never blink      (surprised, angry, dizzy, sleepy, bored)

Every face is built bottom -> top from these pictures (see eye_plan):
    eyeball   normal/eyes/base.png                       under EVERY expression
    iris      normal/eyes/iris.png                       the part that looks around (follows a person / wanders)
    lid       angry/eyes/eyelid.gif, bored/eyes/bored.gif   plays once, HOLDS its last frame, and plays backwards
                                                         just before the expression ends. Flipped on screen 2.
    loop      dizzy/eyes/dizzy.gif, sleepy/eyes/sleepy.gif   loops
    variants  excited/eyes/excited.gif + excited_2.png   he flips between them at random times
    blink     normal/eyes/blink.gif                      on top of every blinking expression (normal, excited, sarcastic)
A file of the same name inside an expression's own folder (base.png, iris.png, blink.gif) overrides the one from normal.

When each expression can show up:
    parked (idle)         normal  surprised  angry  sarcastic  excited  sleepy  bored
    driving               normal  angry  sarcastic
    surprised (forced)    about to hit a wall            (hazard_node, /surprise)
    dizzy (forced)        collision                      (hazard_node, /collision)
    excited (forced)      lap completed, the car stops after excited_after_drive_s of driving, or the excite button

Nothing is scheduled ahead: the expression and the speaking each run on their own random timers.
"""
import os
import random
from dataclasses import dataclass, fields

@dataclass(frozen=True)
class Spec:
    blinks: bool                 # blinks every blink_every_s (also decides the folder: blinking/ or static/)
    look: str                    # iris: 'track' = follows people, 'wander' = looks around, 'none' = no iris
    iris: bool = True            # draws the normal iris
    lid: str = ''                # eyelid gif in its own folder: holds the last frame, reverses at the end
    loop: tuple = ()             # gifs that just loop on top of the eyeball
    variants: tuple = ()         # looks he flips between at random times


EXPRESSIONS = {
    'normal':    Spec(True,  'track'),
    'sarcastic': Spec(True,  'track'),
    'surprised': Spec(False, 'track'),
    'angry':     Spec(False, 'track', lid='eyelid'),
    'bored':     Spec(False, 'wander', lid='bored'),
    'dizzy':     Spec(False, 'none', iris=False, loop=('dizzy',)),
    'sleepy':    Spec(False, 'none', iris=False, loop=('sleepy',)),
    'excited':   Spec(True,  'none', iris=False, variants=('excited', 'excited_2')),
}
IDLE_POOL = ['normal', 'surprised', 'angry', 'sarcastic', 'excited', 'sleepy', 'bored']
DRIVE_POOL = ['normal', 'angry', 'sarcastic']
CLOSING = [n for n, sp in EXPRESSIONS.items() if sp.lid]     # these play their lid backwards before they end
# forced expressions: name -> (priority, Timing field of its min seconds, of its max seconds)
FORCED = {'dizzy': (3, 'dizzy_min_s', 'dizzy_max_s'),
          'excited': (2, 'excited_min_s', 'excited_max_s'),
          'surprised': (1, 'surprised_min_s', 'surprised_max_s')}


def blinks(name):
    return EXPRESSIONS[name].blinks


def look_mode(name):
    return EXPRESSIONS[name].look


def expression_dir(root, name):
    return os.path.join(root, 'blinking' if blinks(name) else 'static', name)


def _file(folder, stem, exts):
    for ext in exts:
        path = os.path.join(folder, 'eyes', stem + ext)
        if os.path.isfile(path):
            return path
    return None


def eye_plan(root, name):
    """The pictures of one expression (None / [] = not there yet):
    {'base', 'iris', 'blink', 'lid': path, 'loop': [paths], 'variants': [[path], ...]}."""
    spec, own, normal = EXPRESSIONS[name], expression_dir(root, name), expression_dir(root, 'normal')
    plan = {
        'base': _file(own, 'base', ('.png', '.gif')) or _file(normal, 'base', ('.png', '.gif')),
        'iris': None, 'blink': None, 'lid': None, 'loop': [], 'variants': [],
    }
    if spec.iris:
        plan['iris'] = _file(own, 'iris', ('.png', '.gif')) or _file(normal, 'iris', ('.png', '.gif'))
    if spec.blinks:
        plan['blink'] = _file(own, 'blink', ('.gif',)) or _file(normal, 'blink', ('.gif',))
    if spec.lid:
        plan['lid'] = _file(own, spec.lid, ('.gif', '.png'))
    plan['loop'] = [f for f in (_file(own, stem, ('.gif', '.png')) for stem in spec.loop) if f]
    plan['variants'] = [[f] for f in (_file(own, stem, ('.gif', '.png')) for stem in spec.variants) if f]
    return plan


VOICE_EXTENSIONS = ('.mp3', '.m4a', '.wav', '.ogg', '.flac', '.aac')


def find_voice_files(root, name):
    folder = os.path.join(expression_dir(root, name), 'voice')
    try:
        return sorted(os.path.join(folder, f) for f in os.listdir(folder) if f.lower().endswith(VOICE_EXTENSIONS))
    except OSError:
        return []


class ShuffleBag:
    """Every file once in random order, then reshuffle (the first of a new round is never the last played)."""

    def __init__(self, rng=random):
        self.bag, self.last, self.rng = [], None, rng

    def next(self, files):
        self.bag = [f for f in self.bag if f in files]            # forget files that were deleted
        if not self.bag:
            self.bag = list(files)
            self.rng.shuffle(self.bag)
            if len(self.bag) > 1 and self.bag[-1] == self.last:
                self.bag[0], self.bag[-1] = self.bag[-1], self.bag[0]
        self.last = self.bag.pop()
        return self.last


@dataclass
class Timing:
    """All times in seconds. Every one is a node parameter (float)."""
    expr_hold_min_s: float = 8.0         # an expression stays this long at least ...
    expr_hold_max_s: float = 25.0        # ... and at most this long, then a new random one is picked
    say_gap_drive_min_s: float = 6.0     # pause between voice lines while driving
    say_gap_drive_max_s: float = 18.0
    say_gap_person_min_s: float = 8.0    # parked, a person in view: talks more often
    say_gap_person_max_s: float = 20.0
    say_gap_alone_min_s: float = 25.0    # parked, nobody around
    say_gap_alone_max_s: float = 60.0
    greet_min_s: float = 1.0             # a person shows up while parked: first line after this long ...
    greet_max_s: float = 3.0             # ... (only if the next line was due later than that)
    excited_min_s: float = 8.0           # how long excited stays
    excited_max_s: float = 12.0
    excited_after_drive_s: float = 60.0  # stopping after at least this much driving = excited
    surprised_min_s: float = 3.0         # about to hit a wall: surprised stays this long
    surprised_max_s: float = 5.0
    dizzy_min_s: float = 5.0             # collision: dizzy stays this long
    dizzy_max_s: float = 8.0
    force_cooldown_s: float = 3.0        # ignore the same trigger again this soon
    close_s: float = 1.5                 # lid expressions play their lid backwards this long before they end


class Director:
    """
    step() once every ~100 ms. Returns events:
        ('expression', name)             show this expression now
        ('closing', name)                lid expressions: start playing the lid backwards, the next face follows
        ('say', name, interrupt)         speak one random line of this expression (interrupt = cut the current line)
    """

    def __init__(self, timing=None, rng=None):
        self.t = timing or Timing()
        self.rng = rng or random.Random()
        self.expression = None
        self.expr_until = 0.0
        self.closing_until = 0.0
        self.forced_name = None
        self.forced_until = 0.0
        self.last_forced = {}
        self.next_say = 0.0
        self.ctx = None
        self.was_driving = False
        self.drive_start = 0.0
        self.was_speaking = False
        self.was_person = False
        self.pending = []

    # ---- triggers (any time, from callbacks) ----
    def force(self, now, name):
        """dizzy / excited / surprised: show it right away, with a line."""
        if now - self.last_forced.get(name, -1e9) >= self.t.force_cooldown_s:
            self.last_forced[name] = now
            self.pending.append(name)

    def excite(self, now):
        self.force(now, 'excited')

    # ---- helpers ----
    def _gap(self, ctx, person):
        t = self.t
        if ctx == 'drive':
            lo, hi = t.say_gap_drive_min_s, t.say_gap_drive_max_s
        elif person:
            lo, hi = t.say_gap_person_min_s, t.say_gap_person_max_s
        else:
            lo, hi = t.say_gap_alone_min_s, t.say_gap_alone_max_s
        return self.rng.uniform(lo, hi)

    def _pick(self, now, pool):
        choices = [n for n in pool if n != self.expression] or list(pool)
        self.expression = self.rng.choice(choices)
        self.expr_until = now + self.rng.uniform(self.t.expr_hold_min_s, self.t.expr_hold_max_s)
        return ('expression', self.expression)

    def _apply_forced(self, now, person):
        """Highest priority pending trigger wins, unless something more important is showing."""
        best = max(self.pending, key=lambda n: FORCED[n][0])
        self.pending.clear()
        if self.forced_name and now < self.forced_until and FORCED[best][0] < FORCED[self.forced_name][0]:
            return []
        _, lo, hi = FORCED[best]
        self.forced_name, self.expression = best, best
        self.forced_until = self.expr_until = now + self.rng.uniform(getattr(self.t, lo), getattr(self.t, hi))
        self.closing_until = 0.0
        self.next_say = self.forced_until + self._gap(self.ctx or 'idle', person)
        return [('expression', best), ('say', best, True)]

    def step(self, now, driving, person, speaking):
        ev = []

        # a drive that just ended after long enough = excited
        if driving and not self.was_driving:
            self.drive_start = now
        if self.was_driving and not driving and now - self.drive_start >= self.t.excited_after_drive_s:
            self.excite(now)
        self.was_driving = driving

        if self.pending:
            ev = self._apply_forced(now, person)
            if ev:
                self.was_speaking, self.was_person = speaking, person
                return ev
        forced_now = now < self.forced_until
        if self.forced_name and not forced_now:
            self.forced_name, self.expr_until = None, 0.0           # forced face is over: pick a normal one below

        ctx = 'drive' if driving else 'idle'
        pool = DRIVE_POOL if driving else IDLE_POOL
        ctx_changed = ctx != self.ctx
        self.ctx = ctx

        # the expression: random hold times, never swapped while a line is playing
        if not forced_now:
            if self.closing_until and (ctx_changed or self.expression not in pool):
                self.closing_until = 0.0                            # context changed under it: no time to reverse
            if self.closing_until:
                if now >= self.closing_until:
                    self.closing_until = 0.0
                    ev.append(self._pick(now, pool))
            elif not speaking:
                if self.expression is None or ctx_changed or self.expression not in pool:
                    ev.append(self._pick(now, pool))
                elif now >= self.expr_until:
                    if self.expression in CLOSING:                  # lid goes back up first, then the next face
                        self.closing_until = now + self.t.close_s
                        ev.append(('closing', self.expression))
                    else:
                        ev.append(self._pick(now, pool))

        # the voice
        if self.was_speaking and not speaking:
            self.next_say = now + self._gap(ctx, person)            # always a pause after a line
        if ctx_changed:
            self.next_say = now + self._gap(ctx, person)
        if ctx == 'idle' and person and not self.was_person:
            self.next_say = min(self.next_say, now + self.rng.uniform(self.t.greet_min_s, self.t.greet_max_s))
        if self.expression is not None and not speaking and not self.closing_until and now >= self.next_say:
            ev.append(('say', self.expression, False))
            self.next_say = now + self._gap(ctx, person)            # replaced by the gap after the line ends
        self.was_speaking, self.was_person = speaking, person
        return ev
