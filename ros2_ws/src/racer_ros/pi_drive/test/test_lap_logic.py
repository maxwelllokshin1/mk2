import numpy as np

from pi_drive.lap_logic import LapDetector, distance, signature


def fake_track(n=200, seed=1):
    """Signature of the car at 'n' places around a loop; place 0 == place n (one lap)."""
    rng = np.random.default_rng(seed)
    return [rng.uniform(0.5, 5.0, 36) for _ in range(n)]


def test_signature_handles_inf_and_zero():
    s = signature([1.0, float('inf'), 0.0, float('nan')], -3.1, 1.5, n_bins=4, max_range=6.0)
    assert np.nanmax(s) <= 6.0 and np.isnan(s).any()


def test_counts_one_lap_per_loop():
    track = fake_track()
    det = LapDetector(match_m=0.6, away_m=1.2, min_lap_s=8.0)
    for _ in range(5):
        det.standby_scan(track[0])
    assert det.arm(0.0)
    t, laps = 0.0, 0
    for _ in range(3):                                   # three loops, 0.5 s per place
        for sig in track[1:] + [track[0]]:
            t += 0.5
            laps += det.update(sig + np.random.default_rng(int(t * 10)).normal(0, 0.1, 36), t)
    assert laps == 3


def test_no_lap_while_still_at_start():
    track = fake_track()
    det = LapDetector()
    det.standby_scan(track[0])
    det.arm(0.0)
    assert not any(det.update(track[0], 10.0 + i) for i in range(20))   # never left: no lap


def test_too_little_overlap_never_matches():
    a = np.full(36, np.nan)
    assert distance(a, a) == float('inf')
