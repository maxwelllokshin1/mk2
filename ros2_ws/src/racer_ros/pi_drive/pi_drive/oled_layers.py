"""
oled_layers.py: layered image compositing for a 128x64 1-bit OLED (pure PIL, no ROS, no I2C).

Layers are drawn bottom -> top. Each layer is a PNG (static) or a GIF (animated), has a
size in pixels and a CENTER position on the display. PNG/GIF transparency is respected,
so a lower layer shows through. One layer can be moved at runtime (render(positions=...)).

GIF layers can play continuously (gap 0) or once every `gap` seconds and then disappear
(blink-style: the lid is only drawn while it is moving).
"""
import bisect
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from PIL import Image, ImageChops, ImageSequence

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

    @property
    def animated(self):
        return len(self.frames) > 1


def load_layer(path, size=None, center=(64, 32), black_transparent=False) -> Layer:
    """Load a PNG/GIF/JPG. size=(w,h) resizes every frame (None keeps the native size)."""
    img = Image.open(path)
    frames, delays = [], []
    for fr in ImageSequence.Iterator(img):
        delays.append(max(fr.info.get('duration', 100) / 1000.0, 0.02))
        frames.append(fr.convert('RGBA'))                    # convert before the next seek
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

    def _draw(self, canvas, indices, t, positions):
        for i in indices:
            layer = self.layers[i]
            idx = frame_index(layer, t, self.gap)
            if idx is None:
                continue
            cx, cy = positions.get(i, layer.center)
            x = int(round(cx - layer.size[0] / 2.0))
            y = int(round(cy - layer.size[1] / 2.0))
            canvas.paste(layer.lum[idx], (x, y), layer.alpha[idx])
        return canvas

    def render(self, t: float, positions: Optional[Dict[int, Tuple[float, float]]] = None) -> Image.Image:
        canvas = self._base.copy()
        self._draw(canvas, range(self._split, len(self.layers)), t, positions or {})
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
