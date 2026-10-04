import numpy as np

from pi_drive.person_detect import Detection, center_norm, decode, letterbox, pick_one


def fake_output(boxes, n=100):
    """YOLOv8-style (1, 84, N): rows 0-3 box (cx,cy,w,h), rows 4.. class scores. boxes = [(cx,cy,w,h,person,other)]"""
    out = np.zeros((1, 84, n), np.float32)
    for i, (cx, cy, w, h, person, other) in enumerate(boxes):
        out[0, :4, i] = (cx, cy, w, h)
        out[0, 4, i] = person
        out[0, 5, i] = other
    return out


def test_letterbox_geometry():
    img = np.zeros((240, 320, 3), np.uint8)
    canvas, r, px, py = letterbox(img, 320)
    assert canvas.shape == (320, 320, 3) and r == 1.0 and px == 0 and py == 40


def test_decode_maps_back_to_original_pixels():
    # 640x480 frame -> 320 model: scale 0.5, pad_y 40. Person box at original (200,100)-(300,400)
    out = fake_output([((250 * .5), (250 * .5) + 40, 100 * .5, 300 * .5, 0.9, 0.0)])
    dets = decode(out, 0.5, 0.45, 0.5, 0, 40, 640, 480)
    assert len(dets) == 1
    d = dets[0]
    assert abs(d.x1 - 200) < 1 and abs(d.y1 - 100) < 1 and abs(d.x2 - 300) < 1 and abs(d.y2 - 400) < 1


def test_only_people_above_threshold_and_nms_dedupes():
    out = fake_output([
        (100, 100, 50, 100, 0.90, 0),     # person
        (102, 101, 50, 100, 0.80, 0),     # same person, overlapping -> suppressed
        (250, 100, 50, 100, 0.30, 0),     # too unsure
        (200, 200, 50, 100, 0.10, 0.95),  # confidently a different class
    ])
    dets = decode(out, 0.5, 0.45, 1.0, 0, 0, 320, 320)
    assert len(dets) == 1 and dets[0].conf > 0.85


def test_pick_one_largest_or_confident():
    small = Detection(0, 0, 10, 10, 0.95)
    big = Detection(0, 0, 100, 100, 0.55)
    assert pick_one([small, big]) is big
    assert pick_one([small, big], 'confident') is small
    assert pick_one([]) is None


def test_center_norm():
    assert center_norm(Detection(100, 50, 300, 150, 0.9), 400, 200) == (0.5, 0.5)
    assert center_norm(Detection(0, 0, 100, 100, 0.9), 400, 200) == (0.125, 0.25)


def test_empty_output():
    assert decode(np.zeros((1, 84, 50), np.float32), 0.5, 0.45, 1.0, 0, 0, 320, 320) == []
