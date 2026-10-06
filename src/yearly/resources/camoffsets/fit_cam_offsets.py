import argparse
import json
import math
import time

import numpy as np

DT = "/SmartDashboard/SwerveDrivetrain/"
BEARING = DT + "readings/bearing (deg)"
YAW_RATE = DT + "readings/yaw_rate (s^-1 deg)"
EMPTY = np.empty((0, 2))

CONFIGURED = {
    1: (6.25, -13.625),
    2: (12.625, -11.875),
}

HELP = "put apriltags around the robot and spin in place more tags is better\n"

def result_topic(cam):
    return f"/AprilTagsCam{cam}/result"


def capture(server, seconds):
    import ntcore

    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("cam-offset-fit")
    inst.setServer(server)
    opts = ntcore.PubSubOptions(sendAll=True, keepDuplicates=True, pollStorage=10000)
    subs = {t: inst.getDoubleTopic(t).subscribe(math.nan, opts) for t in (BEARING, YAW_RATE)}
    subs.update({result_topic(c): inst.getDoubleArrayTopic(result_topic(c)).subscribe([], opts)
                 for c in CONFIGURED})
    data = {t: [] for t in subs}
    start = time.monotonic()
    try:
        while seconds <= 0 or time.monotonic() - start < seconds:
            time.sleep(0.05)
            for t, sub in subs.items():
                data[t] += [(v.serverTime, v.value if t in (BEARING, YAW_RATE) else list(v.value))
                            for v in sub.readQueue()]
            seen = " ".join(f"cam{c}={sum(len(v) > 2 for _, v in data[result_topic(c)])}"
                            for c in CONFIGURED)
            status = "connected" if inst.isConnected() else "waiting"
            print(f"\r{status}  gyro={len(data[BEARING])}  frames with tags: {seen}   ",
                  end="", flush=True)
    except KeyboardInterrupt:
        pass
    print()
    if not data[BEARING]:
        print("no gyro samples; drivetrain topics seen:",
            [t.getName() for t in inst.getTopics(DT)][:20])
    path = time.strftime("camfit_%Y%m%d_%H%M%S.json")
    with open(path, "w") as f:
        json.dump(data, f)
    print("saved", path)
    return data


def as_series(values):
    s = np.array(values, dtype=float).reshape(-1, 2)
    return s[np.argsort(s[:, 0])]


def at(series, t, tol_us):
    if len(series) == 0:
        return np.full(len(t), np.nan)
    ts, vs = series[:, 0], series[:, 1]
    i = np.clip(np.searchsorted(ts, t), 0, len(ts) - 1)
    j = np.clip(i - 1, 0, len(ts) - 1)
    k = np.where(np.abs(ts[j] - t) < np.abs(ts[i] - t), j, i)
    return np.where(np.abs(ts[k] - t) <= tol_us, vs[k], np.nan)


def observations(frames):
    rows = []
    for server_time, v in frames:
        if len(v) < 5:
            continue
        t = v[0] * 1e6 if v[0] > 0 else server_time - v[1] * 1e6
        for k in range(2, len(v) - 2, 3):
            rows.append((t, v[k], v[k + 1], v[k + 2]))
    return np.array(rows, dtype=float).reshape(-1, 4)


def rotate_cw(x, y, deg):
    r = np.radians(deg)
    return x * np.cos(r) + y * np.sin(r), -x * np.sin(r) + y * np.cos(r)


def fit(bearing, tag, theta, r):
    ids = np.unique(tag)
    j = np.searchsorted(ids, tag)
    n, k = len(r), len(ids)
    a, g = np.radians(bearing), np.radians(bearing + theta)
    A = np.zeros((2 * n, 2 + 2 * k))
    A[:n, 0], A[:n, 1] = -np.cos(a), -np.sin(a)
    A[n:, 0], A[n:, 1] = np.sin(a), -np.cos(a)
    A[np.arange(n), 2 + 2 * j] = 1
    A[n + np.arange(n), 3 + 2 * j] = 1
    b = np.concatenate([r * np.sin(g), r * np.cos(g)])
    keep = np.ones(n, bool)
    for _ in range(3):
        rows = np.tile(keep, 2)
        sol = np.linalg.lstsq(A[rows], b[rows], rcond=None)[0]
        res = np.hypot(*(A @ sol - b).reshape(2, n))
        keep = res <= max(3 * np.median(res[keep]), 1.0)
    rows = np.tile(keep, 2)
    rms = math.sqrt(np.mean(res[keep] ** 2))
    se = np.sqrt(np.diag(np.linalg.pinv(A[rows].T @ A[rows]))[:2]) * rms
    tags = {int(t): sol[2 + 2 * i:4 + 2 * i] for i, t in enumerate(ids)}
    return sol[:2], se, tags, rms, keep


def analyze(raw, max_yaw_rate):
    bearing = as_series(raw.get(BEARING, []))
    rate = as_series(raw.get(YAW_RATE, []))
    tag_maps = {}
    for cam, (cx, cy) in CONFIGURED.items():
        obs = observations(raw.get(result_topic(cam), []))
        b = at(bearing, obs[:, 0], 15000)
        w = at(rate, obs[:, 0], 15000)
        ok = np.isfinite(b) & ~(np.abs(w) > max_yaw_rate) & (obs[:, 3] < 300)
        if ok.sum() < 30:
            print(f"camera {cam}: {ok.sum()} usable tag sightings, skipped")
            continue
        o, se, tags, rms, keep = fit(b[ok], obs[ok, 1], obs[ok, 2], obs[ok, 3])
        print(f"camera {cam}: {keep.sum()}/{len(keep)} sightings of {len(tags)} tags, "
              f"rms {rms:.2f} in")
        print(f"  suggested {{{cam}U, {o[0]:.3f}_in_, {o[1]:.3f}_in_}}  "
              f"(was {cx}, {cy}; +/- {se[0]:.2f}, {se[1]:.2f} in)")
        if max(se) > 0.5:
            print("  WARNING: offset poorly determined; spin more or add tags in more directions")
        tag_maps[cam] = tags
    shared = sorted(set.intersection(*(set(m) for m in tag_maps.values()))) if len(tag_maps) > 1 else []
    if shared:
        cams = sorted(tag_maps)
        print("tags seen by both cameras (x, y in, robot center at origin):")
        for t in shared:
            p = [tag_maps[c][t] for c in cams]
            cols = "  ".join(f"cam{c} ({v[0]:7.1f}, {v[1]:7.1f})" for c, v in zip(cams, p))
            gap = np.hypot(*(p[0] - p[1]))
            print(f"  tag {t:2d}  {cols}  gap {gap:.1f}" + ("  <-- check yaw/tag size" if gap > 2 else ""))


def simulate(rng, o, cam_yaw, tags, bearings, noise):
    rows = []
    for th in bearings:
        for tid, (tx, ty) in tags.items():
            ox, oy = rotate_cw(*o, th)
            vx, vy = tx - ox, ty - oy
            rel = math.degrees(math.atan2(vx, vy)) - th
            if abs((rel - cam_yaw + 180) % 360 - 180) < 35:
                rows.append((th, tid, rel + rng.normal(0, 0.2), math.hypot(vx, vy) + rng.normal(0, noise)))
    return np.array(rows).T


def selftest():
    s = np.array([[0.0, 1.0], [10.0, 2.0], [20.0, 3.0]])
    assert np.allclose(at(s, np.array([9.0, 26.0]), 5), [2.0, np.nan], equal_nan=True)
    obs = observations([(5e6, [4.0, 0.02, 7, 12.5, 80.0, 9, -3.0, 120.0])])
    assert np.allclose(obs, [[4e6, 7, 12.5, 80.0], [4e6, 9, -3.0, 120.0]])
    rng = np.random.default_rng(846)
    tags = {3: (90.0, 40.0), 7: (-60.0, 110.0), 11: (-100.0, -70.0), 20: (50.0, -120.0)}
    o = (6.25, -13.625)
    b, t, th, r = simulate(rng, o, 180.0, tags, np.arange(0.0, 720.0, 2.0), 0.3)
    r[:5] += 30
    fo, se, ftags, _, keep = fit(b, t, th, r)
    assert np.allclose(fo, o, atol=0.15), fo
    assert not keep[:5].any() and max(se) < 0.5
    assert all(np.allclose(ftags[i], tags[i], atol=0.5) for i in ftags), ftags
    fo2, *_ = fit(b + 17.0, t, th, r)
    assert np.allclose(fo2, fo, atol=1e-6), "gyro zero must not matter"
    print("selftest ok")


def main():
    ap = argparse.ArgumentParser(
        description="Fit AprilTag camera mount offsets by spinning in place among loose tags.",
        epilog=HELP, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--server", default="10.8.46.2", help="robot address")
    ap.add_argument("--seconds", type=float, default=0, help="stop after N s (default: Ctrl+C)")
    ap.add_argument("--load", help="refit a saved camfit_*.json instead of connecting")
    ap.add_argument("--max-yaw-rate", type=float, default=15.0,
        help="deg/s; drop sightings taken while turning faster (default 15)")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        selftest()
        return
    if args.load:
        with open(args.load) as f:
            raw = json.load(f)
    else:
        raw = capture(args.server, args.seconds)
    analyze(raw, args.max_yaw_rate)


if __name__ == "__main__":
    main()
