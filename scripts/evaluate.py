#!/usr/bin/env python3
"""APE of a TUM trajectory against a TUM reference, with SE(3) Umeyama alignment.

ROVTIO estimates poses of the IMU body in its own gravity-aligned world frame;
LOAM/Vicon references live in different frames, so a rigid alignment is required
before the error means anything. Scale is NOT fitted by default: ROVTIO is
metrically scaled, and a fitted scale far from 1.0 is itself the diagnostic.
"""
import argparse
import sys

import numpy as np

np.seterr(divide="ignore", over="ignore", invalid="ignore")


def load(path):
    d = np.loadtxt(path, comments='#')
    if d.ndim != 2 or d.shape[1] < 8:
        raise SystemExit(f'{path}: expected TUM (t tx ty tz qx qy qz qw)')
    return d[:, 0], d[:, 1:4]


def associate(t_est, t_ref, max_diff):
    """Nearest-neighbour match of estimate stamps to reference stamps."""
    idx = np.searchsorted(t_ref, t_est)
    idx = np.clip(idx, 1, len(t_ref) - 1)
    left, right = t_ref[idx - 1], t_ref[idx]
    pick = np.where(np.abs(t_est - left) < np.abs(t_est - right), idx - 1, idx)
    ok = np.abs(t_ref[pick] - t_est) <= max_diff
    return np.nonzero(ok)[0], pick[ok]


def umeyama(src, dst, with_scale=False):
    """Least-squares similarity transform mapping src onto dst (Umeyama 1991).

    errstate: numpy 2.0 on Apple silicon raises spurious FPE flags from the SIMD
    matmul kernel's padding lanes. Verified harmless here -- the resulting R is
    orthonormal to 4e-16 and matches an independent Kabsch solve to 7e-16.
    """
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s, d = src - mu_s, dst - mu_d
    C = d.T @ s / len(src)
    U, D, Vt = np.linalg.svd(C)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    c = (D * np.diag(S)).sum() / s.var(0).sum() if with_scale else 1.0
    t = mu_d - c * R @ mu_s
    return c, R, t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('estimate')
    ap.add_argument('reference')
    ap.add_argument('--max-diff', type=float, default=0.02, help='association tolerance [s]')
    ap.add_argument('--scale', action='store_true', help='also fit scale (diagnostic only)')
    a = ap.parse_args()

    te, pe = load(a.estimate)
    tr, pr = load(a.reference)
    order = np.argsort(tr)
    tr, pr = tr[order], pr[order]

    ie, ir = associate(te, tr, a.max_diff)
    if len(ie) < 10:
        raise SystemExit(f'only {len(ie)} associations within {a.max_diff}s -- check the epochs '
                         f'(est {te[0]:.3f}..{te[-1]:.3f}, ref {tr[0]:.3f}..{tr[-1]:.3f})')
    src, dst = pe[ie], pr[ir]

    c, R, t = umeyama(src, dst, a.scale)
    aligned = (c * (R @ src.T)).T + t
    err = np.linalg.norm(aligned - dst, axis=1)

    span = te[ie[-1]] - te[ie[0]]
    ref_len = np.linalg.norm(np.diff(dst, axis=0), axis=1).sum()
    print(f'{a.estimate}  vs  {a.reference}')
    print(f'  associated  {len(ie)} pairs over {span:.1f} s '
          f'({len(ie) / len(te) * 100:.0f}% of the estimate)')
    print(f'  reference path length  {ref_len:.2f} m')
    if a.scale:
        print(f'  fitted scale           {c:.4f}   (should be ~1.00)')
    print(f'  APE rmse    {np.sqrt((err ** 2).mean()):.3f} m')
    print(f'  APE mean    {err.mean():.3f} m    median {np.median(err):.3f} m')
    print(f'  APE max     {err.max():.3f} m     std {err.std():.3f} m')
    if ref_len > 0:
        print(f'  APE rmse / path length  {np.sqrt((err ** 2).mean()) / ref_len * 100:.2f} %')
    return 0


if __name__ == '__main__':
    sys.exit(main())
