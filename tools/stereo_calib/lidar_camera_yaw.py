#!/usr/bin/env python3
"""Camera yaw relative to the lidar, from one flat target of known width.

Put a flat, textured target of known width (for example a laptop screen showing a
random pattern) upright in front of the rover, about 0.5 to 1 m away, so that both the
lidar and the stereo camera see all of it, with nothing else beside it at the same
distance. Run this while the robot's TF (bringup) and the stereo pipeline are running
and the rover stands still.

The target is found in the lidar scan and in the stereo cloud's slice at the lidar's
height, both expressed in the base frame through TF. For each sensor the script reports
where it puts the target: forward distance, sideways offset, angle and width. From
these it works out the camera yaw that makes the camera agree with the lidar, in two
independent ways:

  position: where the target's ends and centre are, seen from the camera. Not affected
            by stereo depth errors; affected by an error in the camera's mounting
            position (more so the closer the target is).
  surface:  the angle of the target's surface. Not affected by the mounting position;
            affected by stereo depth errors across the image.

Use the position yaw. Run it at two distances, for example 0.5 and 1 m: a position
yaw that changes with distance points to the camera's mounting position in the URDF,
and a surface angle difference that grows with distance points to the stereo
calibration. The stereo target can come out wider than the lidar one, because block
matching spreads the edges of near objects.

The lidar is the reference. Angles are in degrees, counterclockwise seen from above;
a target angle of 0 means it squarely faces the rover, positive means its left end is
closer.
"""
import argparse
import math
import time
import warnings

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
import tf2_ros

BIN = 0.25   # degrees: bearing bins of the stereo slice (about 2 pixels)


def rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def describe(b, r, step, origin):
    # one flat piece: a line through its points, and its two ends where the line meets
    # the outer edges of its first and last bearing bins
    br = np.radians(b)
    p = origin + np.stack([r * np.cos(br), r * np.sin(br)], axis=1)
    q = p[1:-1] if len(p) > 10 else p
    c = q.mean(axis=0)
    w = q - c
    nrm = np.linalg.eigh(w.T @ w)[1][:, 0]
    if nrm @ (c - origin) < 0:
        nrm = -nrm
    d = nrm @ c
    ends = []
    for be in (b[0] - step / 2, b[-1] + step / 2):
        u = np.array([math.cos(math.radians(be)), math.sin(math.radians(be))])
        ends.append(origin + u * ((d - nrm @ origin) / (nrm @ u)))
    return {'right': ends[0], 'left': ends[1], 'centre': (ends[0] + ends[1]) / 2,
            'width': float(np.hypot(*(ends[1] - ends[0]))),
            'angle': math.degrees(math.atan2(nrm[1], nrm[0])),
            'rms': float(np.sqrt(np.mean((w @ nrm) ** 2))), 'bins': len(b), 'pieces': 1, 'b': b, 'r': r}


def flat_pieces(bear, rng, step, origin):
    # bear: bin bearings in degrees, ascending; rng: median range per bin (nan = empty).
    # A piece is a run of neighbouring bins without a jump in range.
    out = []
    n = len(bear)
    i = 0
    while i < n:
        if not np.isfinite(rng[i]):
            i += 1
            continue
        j = i
        while j + 1 < n and np.isfinite(rng[j + 1]) and abs(rng[j + 1] - rng[j]) < max(0.03, 3 * rng[j] * math.radians(step)):
            j += 1
        if j - i + 1 >= 8:
            out.append(describe(bear[i:j + 1], rng[i:j + 1], step, origin))
        i = j + 1
    return out


def pick(pieces, step, origin, width, tol, near=None):
    # the target is one flat piece, or a run of neighbouring pieces on one line (a stripe
    # without texture, such as a screen's border, splits it in the stereo slice)
    cands = list(pieces)
    for i in range(len(pieces)):
        for j in range(i + 1, min(i + 6, len(pieces))):
            run = pieces[i:j + 1]
            m = describe(np.concatenate([t['b'] for t in run]), np.concatenate([t['r'] for t in run]), step, origin)
            nrm = np.array([math.cos(math.radians(m['angle'])), math.sin(math.radians(m['angle']))])
            if m['rms'] < 0.015 and all(abs(nrm @ (t['centre'] - m['centre'])) < 0.02 for t in run):
                m['pieces'] = len(run)
                cands.append(m)
    good = [s for s in cands if abs(s['width'] - width) <= tol and s['rms'] < 0.015]
    if near is not None:
        good = [s for s in good if np.hypot(*(s['centre'] - near)) < 0.15]
    return min(good, key=lambda s: abs(s['width'] - width)) if good else None


def show(name, s, pieces, origin):
    if s is None:
        seen = ', '.join('%.2f at %.2f' % (t['width'], np.hypot(*(t['centre'] - origin))) for t in pieces[:8])
        print('%s: none; flat pieces seen (width m at distance m): %s' % (name, seen if seen else 'none'))
    else:
        print('%s: found forward_m %.3f left_m %+.3f angle_deg %+.2f width_m %.3f fit_rms_m %.4f bins %d pieces %d' % (
            name, s['centre'][0], s['centre'][1], s['angle'], s['width'], s['rms'], s['bins'], s['pieces']))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--width', type=float, required=True, help='width of the target, metres')
    ap.add_argument('--max-range', type=float, default=1.2, help='look for the target within this range, metres')
    ap.add_argument('--duration', type=float, default=20.0, help='seconds of data')
    ap.add_argument('--wait', type=float, default=45.0, help='seconds to wait for the first cloud')
    ap.add_argument('--sector', type=float, default=40.0, help='look within this many degrees of straight ahead')
    ap.add_argument('--band', type=float, default=0.02, help='half height of the cloud slice around the lidar plane, metres')
    ap.add_argument('--scan-topic', default='/scan')
    ap.add_argument('--cloud-topic', default='/stereo/points2')
    ap.add_argument('--base-frame', default='base_link')
    ap.add_argument('--camera-link', default='stereo_left_link', help='the frame whose yaw in the base frame is the camera yaw')
    a = ap.parse_args()
    warnings.simplefilter('ignore', RuntimeWarning)
    np.seterr(all='ignore')
    rclpy.init()
    node = rclpy.create_node('lidar_camera_yaw')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    st = {'scans': [], 'clouds': [], 'last': 0.0, 'n': 0}
    rel = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

    def on_cloud(raw):
        st['n'] += 1
        now = time.monotonic()
        if now - st['last'] >= 0.5:
            st['last'] = now
            st['clouds'].append(raw)

    node.create_subscription(LaserScan, a.scan_topic, st['scans'].append, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, a.cloud_topic, on_cloud, rel, raw=True)
    t_end = time.monotonic() + a.wait
    while not st['clouds'] and time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not st['clouds']:
        print('first_cloud: none')
        return
    print('first_cloud: yes', flush=True)
    st['scans'].clear()
    st['clouds'].clear()
    st['n'] = 0
    t_end = time.monotonic() + a.duration
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    scans = st['scans']
    clouds = [deserialize_message(bytes(raw), PointCloud2) for raw in st['clouds']]
    print('received: scans %d clouds %d (used %d)' % (len(scans), st['n'], len(clouds)))
    if not scans or not clouds:
        print('lidar_target: none; no scan or no cloud in the window')
        return
    try:
        ts = buf.lookup_transform(a.base_frame, scans[0].header.frame_id, Time(), timeout=Duration(seconds=2.0)).transform
        tc = buf.lookup_transform(a.base_frame, clouds[0].header.frame_id, Time(), timeout=Duration(seconds=2.0)).transform
    except Exception as e:
        print('lidar_target: none; no transform (%s)' % type(e).__name__)
        return
    yaw_now = None
    try:
        tl = buf.lookup_transform(a.base_frame, a.camera_link, Time(), timeout=Duration(seconds=1.0)).transform
        rl = rot(tl.rotation)
        yaw_now = math.degrees(math.atan2(rl[1, 0], rl[0, 0]))
    except Exception:
        pass
    rs, o_s = rot(ts.rotation), np.array([ts.translation.x, ts.translation.y, ts.translation.z])
    rc, o_c = rot(tc.rotation), np.array([tc.translation.x, tc.translation.y, tc.translation.z])
    cam = o_c[:2]
    print('camera: position_in_base_m %+.4f %+.4f yaw_now_deg %s; lidar height_in_base_m %.4f' % (
        cam[0], cam[1], 'unknown' if yaw_now is None else '%+.2f' % yaw_now, o_s[2]))

    # lidar: the median range of each beam over the window, then flat pieces along the beams
    m0 = scans[0]
    n = len(m0.ranges)
    rr = np.array([s.ranges for s in scans if len(s.ranges) == n], dtype=np.float64)
    rr[~np.isfinite(rr)] = np.nan
    rr[(rr < m0.range_min) | (rr > min(m0.range_max, a.max_range))] = np.nan
    cnt = np.isfinite(rr).sum(axis=0)
    med = np.full(n, np.nan)
    for i in np.nonzero(cnt >= 0.5 * len(rr))[0]:
        col = rr[:, i]
        med[i] = np.median(col[np.isfinite(col)])
    ang = m0.angle_min + m0.angle_increment * np.arange(n)
    dirs = np.stack([np.cos(ang), np.sin(ang), np.zeros(n)], axis=1) @ rs.T
    bear = np.degrees(np.arctan2(dirs[:, 1], dirs[:, 0]))
    sel = np.nonzero(np.abs(bear) < a.sector)[0]
    sel = sel[np.argsort(bear[sel])]
    lstep = math.degrees(abs(m0.angle_increment))
    lp = flat_pieces(bear[sel], med[sel], lstep, o_s[:2])
    lid = pick(lp, lstep, o_s[:2], a.width, 0.04)
    show('lidar_target', lid, lp, o_s[:2])

    # stereo: the cloud's points at the lidar's height, in the base frame, binned by
    # bearing from the camera; the median range of each bin, then flat pieces
    pts = []
    valid = []
    for m in clouds:
        off = {f.name: f.offset for f in m.fields}
        b = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height * m.width, m.point_step)
        xyz = np.stack([b[:, off[k]:off[k] + 4].copy().view(np.float32).ravel() for k in ('x', 'y', 'z')], axis=1).astype(np.float64)
        ok = np.all(np.isfinite(xyz), axis=1)
        valid.append(100.0 * ok.sum() / len(ok))
        p = xyz[ok] @ rc.T + o_c
        pts.append(p[np.abs(p[:, 2] - o_s[2]) < a.band][:, :2])
    v = np.concatenate(pts) - cam
    r = np.hypot(v[:, 0], v[:, 1])
    bb = np.degrees(np.arctan2(v[:, 1], v[:, 0]))
    ok = (np.abs(bb) < a.sector) & (r > 0.15) & (r < a.max_range + 0.3)
    r, bb = r[ok], bb[ok]
    nb = int(round(2 * a.sector / BIN))
    idx = np.clip(np.floor((bb + a.sector) / BIN).astype(int), 0, nb - 1)
    order = np.argsort(idx, kind='stable')
    idx, r = idx[order], r[order]
    lo = np.searchsorted(idx, np.arange(nb))
    hi = np.searchsorted(idx, np.arange(nb), side='right')
    smed = np.full(nb, np.nan)
    for i in range(nb):
        if hi[i] - lo[i] >= 30:
            smed[i] = np.median(r[lo[i]:hi[i]])
    sp = flat_pieces(-a.sector + BIN * (np.arange(nb) + 0.5), smed, BIN, cam)
    ste = pick(sp, BIN, cam, a.width, 0.04, None if lid is None else lid['centre'])
    print('stereo_cloud: valid_percent %.1f points_in_slice %d' % (float(np.median(valid)), len(r)))
    show('stereo_target', ste, sp, cam)
    if lid is None or ste is None:
        return
    print('stereo_minus_lidar: forward_m %+.3f left_m %+.3f angle_deg %+.2f width_m %+.3f' % (
        ste['centre'][0] - lid['centre'][0], ste['centre'][1] - lid['centre'][1], ste['angle'] - lid['angle'], ste['width'] - lid['width']))

    # the rotation about the camera that best puts the stereo target's ends and centre
    # on the lidar's, and the difference of the two surface angles
    num = den = 0.0
    for k in ('right', 'centre', 'left'):
        s, t = ste[k] - cam, lid[k] - cam
        num += s[0] * t[1] - s[1] * t[0]
        den += s @ t
    d_pos = math.degrees(math.atan2(num, den))
    d_ang = lid['angle'] - ste['angle']
    cr, sr = math.cos(math.radians(d_pos)), math.sin(math.radians(d_pos))
    moved = cam + (ste['centre'] - cam) @ np.array([[cr, sr], [-sr, cr]])
    dist = float(np.hypot(*(lid['centre'] - cam)))
    per5 = math.degrees(math.atan2(0.005, dist))

    def absolute(d):
        return '' if yaw_now is None else ' camera_yaw_deg %+.2f' % (yaw_now + d)

    print('yaw_from_position: change_deg %+.2f%s' % (d_pos, absolute(d_pos)))
    print('yaw_from_surface_angle: change_deg %+.2f%s' % (d_ang, absolute(d_ang)))
    print('left_after_position_yaw: forward_m %+.3f left_m %+.3f angle_deg %+.2f' % (
        moved[0] - lid['centre'][0], moved[1] - lid['centre'][1], ste['angle'] + d_pos - lid['angle']))
    print('sensitivity: target %.2f m from the camera; 5 mm of sideways error in the camera position is %.2f deg of position yaw' % (dist, per5))
    gap = abs(d_pos - d_ang)
    if gap <= 1.0:
        verdict = 'the two methods agree'
    else:
        verdict = ('the two methods disagree: use the position yaw, and repeat at about twice the distance '
                   '(a position yaw that changes with distance points to the camera mounting position in the URDF; '
                   'a surface angle difference that grows with distance points to the stereo calibration)')
    if ste['width'] < lid['width'] - 0.02:
        verdict += '; the stereo target is more than 2 cm narrower than the lidar one, so one of its ends may be missing'
    print('conclusion: change_deg %+.2f%s (now %s); methods differ by %.2f deg: %s' % (
        d_pos, absolute(d_pos), 'unknown' if yaw_now is None else '%+.2f' % yaw_now, gap, verdict))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
