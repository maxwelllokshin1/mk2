"""
oled_layers.py: layered image compositing for a 128x64 1-bit OLED (pure PIL, no ROS, no I2C).

Layers are drawn bottom -> top. Each layer is a PNG (static) or a GIF (animated), has a
size in pixels and a CENTER position on the display. PNG/GIF transparency is respected,
so a lower layer shows through. One layer can be moved at runtime (render(positions=...)).

GIF layers can play continuously (gap 0) or once every `gap` seconds and then disappear
(blink-style: the lid is only drawn while it is moving), or be a "hold" layer (an eyelid): play once,
stay on the last frame, and play backwards when render(closing_t=...) says the expression is ending.
"""
import bisect
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from PIL import Image, ImageChops, ImageOps, ImageSequence

_LANCZOS = getattr(Image, 'Resampling', Image).LANCZOS


@dataclass
class Layer:
    frames: List[Image.Image]            # RGBA, already resized
    delays: List[float]                  # seconds per frame
    size: Tuple[int, int]
    center: Tuple[float, float]
    lum: List[Image.Image] = field(default_factory=list)     # per-frame luminance
    alpha: List[Image.Image] = field(default_factory=list)   # per-frame mask
    starts: List[float] = field(default_factory=list)
    duration: float = 0.0
    gap: Optional[float] = None          # rest between plays for THIS layer (None = the compositor's gif_gap_s)
    t_offset: float = 0.0                # shifts this layer's clock (a blink starts resting, not playing)
    hold: bool = False                   # eyelid: play once, hold the last frame, reverse when closing

    @property
    def animated(self):
        return len(self.frames) > 1


def load_layer(path, size=None, center=(64, 32), black_transparent=False, mirror=False) -> Layer:
    """Load a PNG/GIF/JPG. size=(w,h) resizes every frame (None keeps the native size). mirror = flip left <-> right."""
    img = Image.open(path)
    frames, delays = [], []
    for fr in ImageSequence.Iterator(img):
        delays.append(max(fr.info.get('duration', 100) / 1000.0, 0.02))
        frames.append(fr.convert('RGBA'))                    # convert before the next seek
    if mirror:
        frames = [ImageOps.mirror(f) for f in frames]
    if size is not None and tuple(size) != frames[0].size:
        frames = [f.resize(tuple(size), _LANCZOS) for f in frames]
    return make_layer(frames, delays, center, black_transparent)


def make_layer(frames, delays, center, black_transparent=False) -> Layer:
    layer = Layer(frames=frames, delays=delays, size=frames[0].size, center=tuple(center))
    for f in frames:
        lum = f.convert('L')
        alpha = f.getchannel('A')
        if black_transparent:
            alpha = ImageChops.multiply(alpha, lum.point(lambda p: 255 if p > 8 else 0))
        layer.lum.append(lum)
        layer.alpha.append(alpha)
    t = 0.0
    for d in delays:
        layer.starts.append(t)
        t += d
    layer.duration = t
    return layer


def frame_index(layer: Layer, t: float, gap: float) -> Optional[int]:
    """Which frame is showing at time t. None = animation is resting between plays (draw nothing)."""
    if not layer.animated:
        return 0
    cycle = layer.duration + max(gap, 0.0)
    phase = t % cycle
    if phase >= layer.duration:
        return None
    return bisect.bisect_right(layer.starts, phase) - 1


def hold_index(layer: Layer, t: float, closing_t: Optional[float]) -> int:
    """Eyelid frame: forward once and hold the last frame; once closing_t is set, backwards to the first frame."""
    last = len(layer.frames) - 1
    if closing_t is not None and t >= closing_t:
        t = layer.duration - (t - closing_t)                  # time runs backwards from the end of the gif
        return 0 if t <= 0.0 else max(bisect.bisect_right(layer.starts, t) - 1, 0)
    return last if t >= layer.duration else max(bisect.bisect_right(layer.starts, t) - 1, 0)


class Compositor:
    def __init__(self, layers: List[Layer], size=(128, 64), movable=(), gif_gap_s=0.0,
                 threshold=128, dither=False):
        self.layers = layers
        self.size = size
        self.movable = set(movable)
        self.gap = gif_gap_s
        self.threshold = threshold
        self.dither = dither
        # Cache everything below the first layer that animates or moves.
        self._split = 0
        for i, layer in enumerate(layers):
            if layer.animated or i in self.movable:
                break
            self._split = i + 1
        self._base = self._draw(Image.new('L', size, 0), range(self._split), 0.0, {})

    def _draw(self, canvas, indices, t, positions, closing_t=None):
        for i in indices:
            layer = self.layers[i]
            if layer.hold:
                idx = hold_index(layer, t, closing_t)
            else:
                idx = frame_index(layer, t + layer.t_offset, self.gap if layer.gap is None else layer.gap)
            if idx is None:
                continue
            cx, cy = positions.get(i, layer.center)
            x = int(round(cx - layer.size[0] / 2.0))
            y = int(round(cy - layer.size[1] / 2.0))
            canvas.paste(layer.lum[idx], (x, y), layer.alpha[idx])
        return canvas

    def render(self, t: float, positions: Optional[Dict[int, Tuple[float, float]]] = None,
               closing_t: Optional[float] = None) -> Image.Image:
        canvas = self._base.copy()
        self._draw(canvas, range(self._split, len(self.layers)), t, positions or {}, closing_t)
        return self.to_1bit(canvas)

    def to_1bit(self, canvas):
        if self.dither:
            return canvas.convert('1')                       # Floyd-Steinberg
        th = self.threshold
        return canvas.point(lambda p: 255 if p >= th else 0).convert('1')


# ---- helpers for moving a layer (pure, tested) ----

def person_to_center(px, py, limits, invert_x=True, invert_y=False):
    """
    Person position in the camera image (0..1, origin top-left) -> where the movable layer's
    center should go, inside limits = (x_min, x_max, y_min, y_max) in display pixels.
    invert_x is on by default: the display faces the same way as the camera, so a person on the
    camera's right is on the viewer's left of the screen.
    """
    x_min, x_max, y_min, y_max = limits
    fx = 1.0 - px if invert_x else px
    fy = 1.0 - py if invert_y else py
    fx = min(max(fx, 0.0), 1.0)
    fy = min(max(fy, 0.0), 1.0)
    return x_min + fx * (x_max - x_min), y_min + fy * (y_max - y_min)


def ease(current, target, alpha):
    """Move a fraction alpha (0..1) of the way to the target; alpha 1 = jump."""
    return current[0] + (target[0] - current[0]) * alpha, current[1] + (target[1] - current[1]) * alpha


def open_centers(iris: Layer, blockers, limits, step=2, tolerance=2):
    """
    Where the iris may sit without going under an eyelid. `blockers` = layers whose LAST frame covers part of the
    screen (the lid, and its mirror image). A centre is allowed when at most `tolerance` iris pixels land on the lid.
    Returns [(x, y), ...] inside limits = (x_min, x_max, y_min, y_max). Needs numpy; without it, returns None (no limit).
    """
    try:
        import numpy as np
    except ImportError:
        return None
    w, h = 128, 64
    covered = np.zeros((h, w), dtype=bool)
    for lid in blockers:
        x0 = int(round(lid.center[0] - lid.size[0] / 2.0))
        y0 = int(round(lid.center[1] - lid.size[1] / 2.0))
        a = np.asarray(lid.alpha[-1]) > 128
        ys, xs = np.nonzero(a)
        ys, xs = ys + y0, xs + x0
        keep = (ys >= 0) & (ys < h) & (xs >= 0) & (xs < w)
        covered[ys[keep], xs[keep]] = True
    ia = np.asarray(iris.alpha[0]) > 128
    iy, ix = np.nonzero(ia)
    iy, ix = iy - iris.size[1] // 2, ix - iris.size[0] // 2          # pixel offsets from the iris centre
    x_min, x_max, y_min, y_max = (int(round(v)) for v in limits)
    out = []
    for cy in range(y_min, y_max + 1, step):
        for cx in range(x_min, x_max + 1, step):
            ys, xs = iy + cy, ix + cx
            ok = (ys >= 0) & (ys < h) & (xs >= 0) & (xs < w)
            if int(covered[ys[ok], xs[ok]].sum()) <= tolerance:
                out.append((float(cx), float(cy)))
    return out


def nearest(points, target):
    """The allowed point closest to target (points non-empty)."""
    return min(points, key=lambda p: (p[0] - target[0]) ** 2 + (p[1] - target[1]) ** 2)
