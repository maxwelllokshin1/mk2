from PIL import Image

from pi_drive.oled_layers import Compositor, ease, frame_index, make_layer, person_to_center


def solid(w, h, value, alpha=255):
    return Image.new('RGBA', (w, h), (value, value, value, alpha))


def px(img, x, y):
    return img.getpixel((x, y)) != 0      # '1' image: True = lit


def static(w, h, value, center, alpha=255):
    return make_layer([solid(w, h, value, alpha)], [0.1], center)


def test_top_layer_covers_bottom_and_transparency_shows_through():
    bottom = static(128, 64, 255, (64, 32))
    top_opaque = static(20, 20, 0, (64, 32))
    img = Compositor([bottom, top_opaque]).render(0)
    assert px(img, 5, 5) and not px(img, 64, 32)         # black square punched into white

    top_clear = static(20, 20, 0, (64, 32), alpha=0)
    img = Compositor([bottom, top_clear]).render(0)
    assert px(img, 64, 32)                               # fully transparent -> white shows


def test_center_positioning_and_offscreen_clipping():
    dot = static(10, 10, 255, (20, 30))
    img = Compositor([dot]).render(0)
    assert px(img, 20, 30) and not px(img, 40, 30)
    off = static(10, 10, 255, (-50, 200))                # entirely off screen: no crash, blank
    assert not any(Compositor([off]).render(0).tobytes())
    edge = static(10, 10, 255, (0, 0))                   # half off the corner
    assert px(Compositor([edge]).render(0), 0, 0)


def test_movable_layer_moves_and_base_stays():
    white = static(128, 64, 255, (64, 32))
    iris = static(8, 8, 0, (64, 32))
    comp = Compositor([white, iris], movable=[1])
    a = comp.render(0, {1: (30, 20)})
    b = comp.render(0, {1: (100, 50)})
    assert not px(a, 30, 20) and px(a, 100, 50)
    assert not px(b, 100, 50) and px(b, 30, 20)


def test_gif_plays_once_per_gap_then_hides():
    frames = [solid(4, 4, 255)] * 4
    layer = make_layer(frames, [0.1] * 4, (10, 10))      # 0.4 s long
    assert frame_index(layer, 0.05, 4.0) == 0
    assert frame_index(layer, 0.35, 4.0) == 3
    assert frame_index(layer, 1.0, 4.0) is None          # resting between blinks
    assert frame_index(layer, 4.45, 4.0) == 0            # next play
    assert frame_index(layer, 1.0, 0.0) is not None      # gap 0 = loop non-stop
    assert frame_index(static(4, 4, 255, (0, 0)), 99, 4.0) == 0   # PNGs always show


def test_hidden_gif_frame_is_not_drawn():
    base = static(128, 64, 255, (64, 32))
    lid = make_layer([solid(128, 64, 0)] * 2, [0.1, 0.1], (64, 32))
    comp = Compositor([base, lid], gif_gap_s=4.0)
    assert not px(comp.render(0.05), 64, 32)             # lid down: black
    assert px(comp.render(1.0), 64, 32)                  # lid gone: white eye visible


def test_static_cache_matches_full_draw():
    a = static(128, 64, 255, (64, 32))
    b = static(30, 30, 0, (40, 30))
    cached = Compositor([a, b]).render(0)
    plain = Compositor([a, b], movable=[0, 1]).render(0)  # nothing cached
    assert cached.tobytes() == plain.tobytes()


def test_person_to_center_mapping():
    lim = (30, 98, 16, 48)
    assert person_to_center(0.5, 0.5, lim) == (64.0, 32.0)
    assert person_to_center(1.0, 0.0, lim) == (30.0, 16.0)                       # person on image right -> iris screen-left
    assert person_to_center(1.0, 0.0, lim, invert_x=False) == (98.0, 16.0)
    assert person_to_center(9.0, -3.0, lim, invert_x=False) == (98.0, 16.0)      # clamped


def test_ease():
    assert ease((0, 0), (10, 20), 0.5) == (5.0, 10.0)
    assert ease((0, 0), (10, 20), 1.0) == (10.0, 20.0)


def _lid(n=4, delay=0.1):
    from pi_drive.oled_layers import make_layer
    frames = [Image.new('RGBA', (128, 64), (255, 255, 255, 255)) for _ in range(n)]
    layer = make_layer(frames, [delay] * n, (64, 32))
    layer.hold = True
    return layer


def test_lid_plays_once_then_holds_the_last_frame():
    from pi_drive.oled_layers import hold_index
    lid = _lid()
    assert [hold_index(lid, t, None) for t in (0.0, 0.15, 0.25, 0.35)] == [0, 1, 2, 3]
    assert hold_index(lid, 5.0, None) == 3 and hold_index(lid, 500.0, None) == 3     # holds


def test_lid_plays_backwards_once_closing_starts_and_ends_on_the_first_frame():
    from pi_drive.oled_layers import hold_index
    lid = _lid()
    seen = [hold_index(lid, 10.0 + u, 10.0) for u in (0.0, 0.05, 0.15, 0.25, 0.35, 0.39, 0.5, 9.0)]
    assert seen[0] == 3 and seen[-1] == 0 and seen[-2] == 0
    assert seen == sorted(seen, reverse=True)                                        # never goes forward again


def test_mirror_flips_the_layer_left_to_right(tmp_path):
    from pi_drive.oled_layers import load_layer
    img = Image.new('RGBA', (8, 4), (0, 0, 0, 0))
    img.putpixel((0, 0), (255, 255, 255, 255))
    img.save(tmp_path / 'a.png')
    plain, flipped = load_layer(str(tmp_path / 'a.png')), load_layer(str(tmp_path / 'a.png'), mirror=True)
    assert plain.alpha[0].getpixel((0, 0)) == 255 and flipped.alpha[0].getpixel((7, 0)) == 255


def test_iris_stays_out_from_under_the_lid():
    from pi_drive.oled_layers import make_layer, open_centers
    lid = make_layer([Image.new('RGBA', (128, 64), (255, 255, 255, 255))], [0.1], (64, 32))
    lid.frames[0].paste((0, 0, 0, 0), (0, 40, 128, 64))                               # lid covers rows 0-39
    lid = make_layer([lid.frames[0]], [0.1], (64, 32))
    iris = make_layer([Image.new('RGBA', (10, 10), (255, 255, 255, 255))], [0.1], (64, 32))
    pts = open_centers(iris, [lid], (10, 118, 5, 60), tolerance=0)
    assert pts and min(y for _, y in pts) >= 45 and max(y for _, y in pts) >= 58      # only below the lid
