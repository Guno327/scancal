#!/usr/bin/env python3
"""scancal — flatbed scanner dimensional calibration via a 3D-printed plate.

The reference is a fixed design: a 100 x 120 mm chamfered frame with an
internal rib truss. It fits virtually any printer bed, any flatbed scanner,
and 150 mm calipers. Because the design is fixed, no metadata travels with
it — `calibrate` knows the geometry.

Workflow:
  1. generate   write the reference STL
  2.            print it (dark filament, chamfer side down), measure its
                outer X and Y edge-to-edge with calipers
  3. calibrate  analyze one or more scans of the plate + your measurements,
                write a tailoring file describing the scanner's error
  4. correct    apply the tailoring file to any scan from the same scanner,
                producing a dimensionally accurate image for CAD import

Passing measurements to `calibrate` cancels the printer's error out of the
tailoring (scanner-only calibration, true dimensions). Omitting them
calibrates against nominal design dimensions: the tailoring then absorbs
printer error too, so corrected scans are pre-distorted — parts modeled on
them and printed on the same printer mate with the scanned object exactly
(closed-loop mode).

In measured mode `calibrate` additionally writes a *printer* tailoring file
(measured vs design dimensions = the printer's own XY scale error), enabling
the true-dimension workflow end to end:

  scan -> correct (scanner tailoring) -> model true mm -> export STL
       -> tailor-stl (printer tailoring) -> print comes out true mm

`tailor-stl` pre-scales STL vertices by the inverse printer error, per axis.
It corrects XY scale only — Z error and hole shrinkage are separate effects —
and assumes the part is printed in the same XY orientation as the plate.

Dependencies: numpy, opencv-python-headless
"""

import argparse
import datetime
import json
import math
import sys

import numpy as np

MM_PER_INCH = 25.4

# ------------------------------------------------------------ fixed design
PLATE_W = 100.0        # mm, X (short edge)
PLATE_H = 120.0        # mm, Y (long edge)
THICK = 3.0            # mm
CHAMFER = 0.5          # mm bottom-edge inset (swallows elephant foot)
CHAMFER_H = 1.0        # mm height the chamfer rises over
FRAME = 8.0            # mm outer frame width
RIB = 3.0              # mm truss rib width
CELLS_X, CELLS_Y = 2, 3

_iw, _ih = PLATE_W - 2 * FRAME, PLATE_H - 2 * FRAME
CELL_W = (_iw - (CELLS_X - 1) * RIB) / CELLS_X
CELL_H = (_ih - (CELLS_Y - 1) * RIB) / CELLS_Y

OPENING_CENTROIDS = [
    (FRAME + i * (CELL_W + RIB) + CELL_W / 2,
     FRAME + j * (CELL_H + RIB) + CELL_H / 2)
    for i in range(CELLS_X) for j in range(CELLS_Y)
]


# ---------------------------------------------------------------- generate

def _prism(poly, lo, hi, plane):
    """Closed prism from CCW 2D polygon extruded between lo..hi.
    plane='y': poly is (x,z), extruded along Y.
    plane='x': poly is (y,z), extruded along X.
    Returns list of triangles with outward normals."""
    def v(p, e):
        return (p[0], e, p[1]) if plane == "y" else (e, p[0], p[1])
    n = len(poly)
    tris = []
    for i in range(1, n - 1):  # end caps (triangle fan)
        tris.append((v(poly[0], lo), v(poly[i + 1], lo), v(poly[i], lo)))
        tris.append((v(poly[0], hi), v(poly[i], hi), v(poly[i + 1], hi)))
    for i in range(n):         # side walls
        a, b = poly[i], poly[(i + 1) % n]
        tris.append((v(a, lo), v(b, lo), v(b, hi)))
        tris.append((v(a, lo), v(b, hi), v(a, hi)))
    if plane == "x":           # axis swap mirrors handedness; restore it
        tris = [(a, c, b) for a, b, c in tris]
    return [(a, c, b) for a, b, c in tris]  # outward normals


def _box(x0, x1, y0, y1, z0, z1):
    return _prism([(x0, z0), (x1, z0), (x1, z1), (x0, z1)], y0, y1, "y")


def cmd_generate(args):
    w, h, t, c, ch = PLATE_W, PLATE_H, THICK, CHAMFER, CHAMFER_H
    # frame sides: cross-section chamfered on BOTH bottom corners (outer edge
    # for caliper/silhouette accuracy, inner edge so opening silhouettes are
    # elephant-foot-free too)
    lo_prof = [(c, 0), (FRAME - c, 0), (FRAME, ch), (FRAME, t), (0, t), (0, ch)]
    hi_prof = lambda d: [(d - FRAME + c, 0), (d - c, 0), (d, ch), (d, t),
                         (d - FRAME, t), (d - FRAME, ch)]
    # rib cross-section: chamfered on both bottom edges
    rib_prof = lambda p0: [(p0 + c, 0), (p0 + RIB - c, 0), (p0 + RIB, ch),
                           (p0 + RIB, t), (p0, t), (p0, ch)]
    tris = []
    tris += _prism(lo_prof, 0, h, "y")            # left  (x: 0..FRAME)
    tris += _prism(hi_prof(w), 0, h, "y")         # right (x: w-FRAME..w)
    tris += _prism(lo_prof, 0, w, "x")            # front (y: 0..FRAME)
    tris += _prism(hi_prof(h), 0, w, "x")         # back  (y: h-FRAME..h)
    for i in range(CELLS_X - 1):                  # ribs along Y
        x0 = FRAME + (i + 1) * CELL_W + i * RIB
        tris += _prism(rib_prof(x0), FRAME - 1, h - FRAME + 1, "y")
    for j in range(CELLS_Y - 1):                  # ribs along X
        y0 = FRAME + (j + 1) * CELL_H + j * RIB
        tris += _prism(rib_prof(y0), FRAME - 1, w - FRAME + 1, "x")

    with open(args.output, "w") as f:
        f.write("solid scancal_plate\n")
        for a, b, cc in tris:
            p, q, r = (np.array(x, float) for x in (a, b, cc))
            nrm = np.cross(q - p, r - p)
            nrm = nrm / (np.linalg.norm(nrm) or 1)
            f.write(f" facet normal {nrm[0]:g} {nrm[1]:g} {nrm[2]:g}\n"
                    f"  outer loop\n")
            for pt in (p, q, r):
                f.write(f"   vertex {pt[0]:g} {pt[1]:g} {pt[2]:g}\n")
            f.write("  endloop\n endfacet\n")
        f.write("endsolid scancal_plate\n")

    print(f"wrote {args.output}: {w:g} x {h:g} x {t:g} mm frame "
          f"({FRAME:g} mm frame, {RIB:g} mm ribs, {CELLS_X}x{CELLS_Y} "
          f"openings of {CELL_W:.1f} x {CELL_H:.1f} mm, {c:g} mm chamfer)")
    print("print in a dark color, chamfer side down (as modeled). Measure the "
          "outer edges with caliper jaws spanning the full thickness.")


# ---------------------------------------------------------------- detection

def detect_plate(gray):
    """Detect the plate silhouette; return 4 sub-pixel corners (tl,tr,br,bl)."""
    import cv2
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, bw = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        raise SystemExit("no plate found in scan")
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < 0.03 * gray.size:
        raise SystemExit("largest object is too small to be the plate; "
                         "check contrast (dark plate, light background)")
    peri = cv2.arcLength(c, True)
    quad = cv2.approxPolyDP(c, 0.02 * peri, True).reshape(-1, 2).astype(float)
    if len(quad) != 4:
        raise SystemExit(f"plate outline is not a clean quadrilateral "
                         f"({len(quad)} corners); check for shadows")

    pts = c.reshape(-1, 2).astype(float)
    lines = []
    for i in range(4):
        a, b = quad[i], quad[(i + 1) % 4]
        ab = b - a
        L = np.linalg.norm(ab)
        u = ab / L
        rel = pts - a
        along = rel @ u
        perp = rel @ np.array([-u[1], u[0]])
        band = (along > 0.1 * L) & (along < 0.9 * L)
        sel = pts[band & (np.abs(perp) < 25)]          # wide first gate
        if len(sel) < 20:
            raise SystemExit("too few edge points; scan resolution too low?")
        vx, vy, x0, y0 = cv2.fitLine(sel.astype(np.float32),
                                     cv2.DIST_HUBER, 0, 0.01, 0.01).flatten()
        # re-gate on residuals to the fitted line, then refit
        nvec = np.array([-vy, vx])
        r = (pts - (x0, y0)) @ nvec
        mad = max(1.4826 * np.median(np.abs(r[band & (np.abs(perp) < 25)])), 1.0)
        sel = pts[band & (np.abs(r) < 3 * mad)]
        if len(sel) < 20:
            raise SystemExit("too few edge points; scan resolution too low?")
        vx, vy, x0, y0 = cv2.fitLine(sel.astype(np.float32),
                                     cv2.DIST_HUBER, 0, 0.01, 0.01).flatten()
        lines.append((float(vx), float(vy), float(x0), float(y0)))

    corners = []
    for i in range(4):
        vx1, vy1, x1, y1 = lines[i - 1]
        vx2, vy2, x2, y2 = lines[i]
        A = np.array([[vx1, -vx2], [vy1, -vy2]])
        b = np.array([x2 - x1, y2 - y1])
        s = np.linalg.solve(A, b)
        corners.append((x1 + s[0] * vx1, y1 + s[0] * vy1))
    corners = np.array(corners)

    cen = corners.mean(axis=0)
    ang = np.arctan2(corners[:, 1] - cen[1], corners[:, 0] - cen[0])
    corners = corners[np.argsort(ang)]
    start = np.argmin(corners.sum(axis=1))
    return np.roll(corners, -start, axis=0)


def fit_plate(corners, w_mm, h_mm):
    ideal = np.array([(0, 0), (w_mm, 0), (w_mm, h_mm), (0, h_mm)], float)
    M = np.column_stack([ideal, np.ones(4)])
    Ax, *_ = np.linalg.lstsq(M, corners[:, 0], rcond=None)
    Ay, *_ = np.linalg.lstsq(M, corners[:, 1], rcond=None)
    A = np.array([[Ax[0], Ax[1]], [Ay[0], Ay[1]]])
    t = np.array([Ax[2], Ay[2]])
    resid = corners - (ideal @ A.T + t)
    return A, t, float(np.abs(resid).max())


def decompose(A, nominal):
    theta = math.atan2(A[1, 0], A[0, 0])
    ct, st = math.cos(-theta), math.sin(-theta)
    K = np.array([[ct, -st], [st, ct]]) @ A
    sx = K[0, 0]
    sy = math.hypot(K[0, 1], K[1, 1])
    skew = math.degrees(math.atan2(K[0, 1], K[1, 1]))
    return K / nominal, sx / nominal - 1, sy / nominal - 1, skew, math.degrees(theta)


def match_openings(gray, A, t, scale_meas, nominal):
    """Detect truss openings and match their centroids to design positions
    (predicted via the outline-fitted affine). Returns (ideal_mm, cents_px)
    arrays or None. Centroids are immune to the scanner's edge-shadow bias,
    so they are the primary geometry source for the calibration fit."""
    import cv2
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, dark = cv2.threshold(blur, 0, 255,
                            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    contours, hier = cv2.findContours(dark, cv2.RETR_CCOMP,
                                      cv2.CHAIN_APPROX_SIMPLE)
    if hier is None:
        return None
    hier = hier[0]
    plate_idx = int(np.argmax([cv2.contourArea(c) for c in contours]))
    cell_px = (CELL_W * nominal, CELL_H * nominal)
    min_a, max_a = 0.4 * cell_px[0] * cell_px[1], 1.6 * cell_px[0] * cell_px[1]
    cents = []
    for k, c in enumerate(contours):
        if hier[k][3] != plate_idx:
            continue
        a = cv2.contourArea(c)
        if not (min_a <= a <= max_a):
            continue
        m = cv2.moments(c)
        cents.append((m["m10"] / m["m00"], m["m01"] / m["m00"]))
    if len(cents) < 2:
        return None
    cents = np.array(cents)

    ideal = np.array(OPENING_CENTROIDS, float) * np.array(scale_meas)
    pred = ideal @ A.T + t
    tol = 0.35 * min(cell_px)
    pairs_i, pairs_c = [], []
    for i, p in enumerate(pred):
        d = np.linalg.norm(cents - p, axis=1)
        k = int(np.argmin(d))
        if d[k] < tol:
            pairs_i.append(ideal[i])
            pairs_c.append(cents[k])
    if len(pairs_i) < 4:
        return None
    return np.array(pairs_i), np.array(pairs_c)


# ---------------------------------------------------------------- calibrate

def cmd_calibrate(args):
    import cv2

    if args.x and args.y:
        w_mm = float(np.mean(args.x))
        h_mm = float(np.mean(args.y))
        mode = "measured"
        for name, m, d in (("X", w_mm, PLATE_W), ("Y", h_mm, PLATE_H)):
            if abs(m / d - 1) > 0.05:
                raise SystemExit(f"measured {name} ({m:.2f} mm) deviates >5% "
                                 f"from design ({d:g} mm); check axes and "
                                 f"measurements")
    elif args.x or args.y:
        raise SystemExit("give both --x and --y, or neither")
    else:
        w_mm, h_mm = PLATE_W, PLATE_H
        mode = "closed-loop"
        print("no measurements given: calibrating against nominal design "
              "dimensions (closed-loop mode — see README)", file=sys.stderr)

    nominal = args.dpi / MM_PER_INCH
    results = []
    for scan in args.scans:
        img = cv2.imread(scan, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise SystemExit(f"cannot read {scan}")
        corners = detect_plate(img)
        wpx = (np.linalg.norm(corners[1] - corners[0]) +
               np.linalg.norm(corners[2] - corners[3])) / 2
        hpx = (np.linalg.norm(corners[3] - corners[0]) +
               np.linalg.norm(corners[2] - corners[1])) / 2
        if wpx > hpx:
            raise SystemExit(f"{scan}: plate's long axis is along scan X; "
                             f"place the long edge along the scanner's long "
                             f"axis and rescan")
        A0, t0, corner_resid_px = fit_plate(corners, w_mm, h_mm)
        scale = (w_mm / PLATE_W, h_mm / PLATE_H)
        pairs = match_openings(img, A0, t0, scale, nominal)
        if pairs is None:
            print(f"{scan}: warning: opening centroids not found — falling "
                  f"back to outline fit (edge-shadow bias uncorrected)",
                  file=sys.stderr)
            A, t, fit_resid_um, n_open = A0, t0, None, 0
        else:
            ideal, cents = pairs
            X = np.hstack([ideal, np.ones((len(ideal), 1))])
            sol, _, _, _ = np.linalg.lstsq(X, cents, rcond=None)
            A, t = sol[:2].T, sol[2]
            resid = cents - (X @ sol)
            resid_um = np.linalg.norm(resid, axis=1) / nominal * 1000
            fit_resid_um = float(np.sqrt((resid_um ** 2).mean()))
            n_open = len(ideal)
        K_rel, ex, ey, skew, rot = decompose(A, nominal)
        results.append((K_rel, ex, ey, skew, rot))
        line = (f"{scan}: X {ex * 100:+.3f}%  Y {ey * 100:+.3f}%  "
                f"skew {skew:+.3f} deg  rot {rot:+.2f} deg")
        if fit_resid_um is not None:
            line += (f"\n    fit on {n_open} opening centroids, non-affine "
                     f"residual RMS {fit_resid_um:.0f} um")
            if fit_resid_um > 150:
                line += "  <- high: distortion beyond affine (or warped print)"
            # outline vs centroid Y scale: difference = edge shadow width
            _, _, ey0, _, _ = decompose(A0, nominal)
            shadow_um = (ey0 - ey) * h_mm * 1000 / 2
            if abs(shadow_um) > 100:
                line += (f"\n    outline Y wider than centroid fit by "
                         f"~{shadow_um:.0f} um/edge (illumination shadow — "
                         f"expected, corrected)")
        print(line)

    K_rel = np.mean([r[0] for r in results], axis=0)
    ex = float(np.mean([r[1] for r in results]))
    ey = float(np.mean([r[2] for r in results]))
    sx_spread = float(np.std([r[1] for r in results])) * 100
    sy_spread = float(np.std([r[2] for r in results])) * 100

    tailoring = {
        "scancal_version": 4,
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "scanner_note": args.note,
        "mode": mode,
        "calibration_dpi": args.dpi,
        "plate": {"design_mm": [PLATE_W, PLATE_H],
                  "measured_x_mm": args.x, "measured_y_mm": args.y},
        "scans_used": len(args.scans),
        "K_rel": K_rel.tolist(),
        "scale_error_pct_x": round(ex * 100, 4),
        "scale_error_pct_y": round(ey * 100, 4),
        "scale_spread_pct_x": round(sx_spread, 4),
        "scale_spread_pct_y": round(sy_spread, 4),
        "skew_deg": round(float(np.mean([r[3] for r in results])), 4),
    }
    with open(args.output, "w") as f:
        json.dump(tailoring, f, indent=2)

    print(f"\nwrote tailoring file: {args.output} ({mode} mode, "
          f"{len(args.scans)} scan(s))")

    if mode == "measured":
        pex = w_mm / PLATE_W - 1          # printer scale error, X
        pey = h_mm / PLATE_H - 1
        printer = {
            "scancal_version": 4,
            "type": "printer",
            "created": tailoring["created"],
            "printer_note": args.printer_note,
            "plate": tailoring["plate"],
            "scale_error_pct_x": round(pex * 100, 4),
            "scale_error_pct_y": round(pey * 100, 4),
        }
        with open(args.printer_output, "w") as f:
            json.dump(printer, f, indent=2)
        print(f"wrote printer tailoring: {args.printer_output}")
        print(f"  printer error: X {pex * 100:+.3f}%   Y {pey * 100:+.3f}%")
    print(f"  scale error:  X {ex * 100:+.3f}%   Y {ey * 100:+.3f}%")
    if len(args.scans) > 1:
        print(f"  scan spread:  X ±{sx_spread:.3f}%  Y ±{sy_spread:.3f}%  "
              f"(scanner repeatability)")
        if max(sx_spread, sy_spread) > 0.1:
            print("  warning: poor scan-to-scan repeatability; corrections "
                  "will only be accurate to this spread", file=sys.stderr)


# ---------------------------------------------------------------- correct

def cmd_correct(args):
    import cv2

    with open(args.tailoring) as f:
        tail = json.load(f)
    K_rel = np.array(tail["K_rel"], dtype=np.float64)

    src = cv2.imread(args.scan, cv2.IMREAD_UNCHANGED)
    if src is None:
        raise SystemExit(f"cannot read {args.scan}")
    dpi = args.dpi or tail["calibration_dpi"]

    C = np.linalg.inv(K_rel)
    h, w = src.shape[:2]
    corners = np.array([[0, 0], [w, 0], [0, h], [w, h]], dtype=np.float64) @ C.T
    shift = corners.min(axis=0)
    size = np.ceil(corners.max(axis=0) - shift).astype(int)
    M = np.hstack([C, -shift[:, None]])
    border = 255 if src.ndim == 2 else (255,) * src.shape[2]
    out = cv2.warpAffine(src, M, tuple(size), flags=cv2.INTER_LANCZOS4,
                         borderValue=border)

    if args.dxf:
        _outline_dxf(out, args.dxf, dpi, args.min_hole)
        if tail.get("mode") == "closed-loop":
            print("note: closed-loop tailoring — dimensions are pre-distorted "
                  "for the calibration printer, not true mm")
        return

    cv2.imwrite(args.output, out)

    print(f"wrote corrected image: {args.output} ({size[0]}x{size[1]} px)")
    print(f"import into CAD at exactly {dpi:g} DPI = "
          f"{MM_PER_INCH / dpi:.6f} mm/px")
    if tail.get("mode") == "closed-loop":
        print("note: closed-loop tailoring — dimensions are pre-distorted "
              "for the calibration printer, not true mm")


# ---------------------------------------------------------------- outline/dxf

def _refine_subpixel(gray, poly):
    """Shift each contour point to the gradient extremum along the local
    normal (parabolic subpixel fit), searching +/-5 px around the coarse
    threshold position. Reduces threshold/halo bias."""
    import cv2
    g = cv2.GaussianBlur(gray, (0, 0), 1.2).astype(np.float64)
    h, w = g.shape
    n = len(poly)
    tang = np.roll(poly, -2, axis=0) - np.roll(poly, 2, axis=0)
    tang /= np.maximum(np.linalg.norm(tang, axis=1, keepdims=True), 1e-9)
    nrm = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
    offs = np.arange(-5, 6)
    samp = poly[:, None, :] + nrm[:, None, :] * offs[None, :, None]
    xs = np.clip(samp[..., 0], 0, w - 1)
    ys = np.clip(samp[..., 1], 0, h - 1)
    x0 = xs.astype(int); y0 = ys.astype(int)
    x1 = np.minimum(x0 + 1, w - 1); y1 = np.minimum(y0 + 1, h - 1)
    fx = xs - x0; fy = ys - y0
    prof = (g[y0, x0] * (1 - fx) * (1 - fy) + g[y0, x1] * fx * (1 - fy) +
            g[y1, x0] * (1 - fx) * fy + g[y1, x1] * fx * fy)
    grad = np.abs(np.gradient(prof, axis=1))
    k = grad[:, 1:-1].argmax(axis=1) + 1
    i = np.arange(n)
    gm, gl, gr = grad[i, k], grad[i, k - 1], grad[i, k + 1]
    denom = gl - 2 * gm + gr
    with np.errstate(divide="ignore", invalid="ignore"):
        frac = np.where(np.abs(denom) > 1e-9, 0.5 * (gl - gr) / denom, 0.0)
    frac = np.clip(frac, -0.5, 0.5)
    return poly + nrm * (offs[k] + frac)[:, None]


def _smooth_closed(poly, k=7):
    kern = np.ones(k) / k
    pad = k // 2
    ext = np.vstack([poly[-pad:], poly, poly[:pad]])
    return np.stack([np.convolve(ext[:, j], kern, "valid") for j in (0, 1)],
                    axis=1)


def _fit_circle(pts, tol=0.08):
    """RANSAC circle fit at fixed tolerance (mm). Occlusion deforms hole
    contours with chord cuts rather than sparse outliers, so consensus on
    the true arc beats least squares on everything. ok requires a majority
    of points on the arc and >=150 deg coverage, so slots and rectangles
    (whose rounded ends cover <180 deg and a minority of points) don't
    pass."""
    def kasa(p):
        A = np.column_stack([2 * p[:, 0], 2 * p[:, 1], np.ones(len(p))])
        b = (p ** 2).sum(axis=1)
        (cx, cy, c), _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        return cx, cy, np.sqrt(c + cx * cx + cy * cy)

    n = len(pts)
    rng = np.random.default_rng(0)
    best, best_inl = None, None
    for _ in range(200):
        i = rng.choice(n, 3, replace=False)
        try:
            cx, cy, r = kasa(pts[i])
        except np.linalg.LinAlgError:
            continue
        if not np.isfinite(r) or r > 25:
            continue
        d = np.abs(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - r)
        inl = d < tol
        if best_inl is None or inl.sum() > best_inl.sum():
            best, best_inl = (cx, cy, r), inl
    if best is None or best_inl.sum() < 10:
        return 0, 0, 0, False
    for _ in range(3):
        cx, cy, r = kasa(pts[best_inl])
        d = np.abs(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - r)
        best_inl = d < tol
    inl = best_inl
    if inl.sum() < 10:
        return 0, 0, 0, False
    rms = float(np.sqrt((d[inl] ** 2).mean()))
    ang = np.sort(np.arctan2(pts[inl, 1] - cy, pts[inl, 0] - cx))
    gaps = np.diff(np.concatenate([ang, [ang[0] + 2 * np.pi]]))
    coverage = 2 * np.pi - gaps.max()
    ok = (rms <= 0.05 and 0.3 <= r <= 25 and
          ((inl.mean() >= 0.55 and coverage >= np.radians(150)) or
           (inl.mean() >= 0.35 and coverage >= np.radians(210))))
    return float(cx), float(cy), float(r), ok


def _outline_dxf(img, path, dpi, min_hole_mm):
    import cv2
    gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mmpx = MM_PER_INCH / dpi
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, bw = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    contours, hier = cv2.findContours(bw, cv2.RETR_CCOMP,
                                      cv2.CHAIN_APPROX_NONE)
    if hier is None:
        raise SystemExit("no part found in image")
    hier = hier[0]
    part = int(np.argmax([cv2.contourArea(c) for c in contours]))
    min_hole_px_a = np.pi * (min_hole_mm / 2 / mmpx) ** 2
    keep = [part] + [k for k in range(len(contours))
                     if hier[k][3] == part
                     and cv2.contourArea(contours[k]) >= min_hole_px_a]
    polys = []
    circles = []
    h_img = gray.shape[0]
    for k in keep:
        p = contours[k].reshape(-1, 2).astype(np.float64)
        if len(p) < 12:
            continue
        p = _refine_subpixel(gray, p)
        p = _smooth_closed(p)
        mm_all = np.column_stack([p[:, 0] * mmpx, (h_img - p[:, 1]) * mmpx])
        if k != part:
            cx, cy, r, ok = _fit_circle(mm_all)
            if ok:
                circles.append((cx, cy, r))
                continue
        p = cv2.approxPolyDP(p.astype(np.float32), 0.75, True).reshape(-1, 2)
        mm = np.column_stack([p[:, 0] * mmpx, (h_img - p[:, 1]) * mmpx])
        polys.append(mm)

    with open(path, "w") as f:
        f.write("0\nSECTION\n2\nENTITIES\n")
        for mm in polys:
            f.write("0\nPOLYLINE\n8\n0\n66\n1\n70\n1\n")
            for x, y in mm:
                f.write(f"0\nVERTEX\n8\n0\n10\n{x:.4f}\n20\n{y:.4f}\n")
            f.write("0\nSEQEND\n")
        for cx, cy, r in circles:
            f.write(f"0\nCIRCLE\n8\n0\n10\n{cx:.4f}\n20\n{cy:.4f}\n40\n{r:.4f}\n")
        f.write("0\nENDSEC\n0\nEOF\n")

    outer = polys[0]
    print(f"wrote outline: {path} — 1 outer contour, {len(circles)} fitted "
          f"circle(s), {len(polys) - 1} non-circular contour(s), features "
          f">= {min_hole_mm:g} mm, in true mm")
    if circles:
        print("  circle diameters:",
              sorted(round(2 * r, 2) for _, _, r in circles))
    print(f"  outer bbox: {np.ptp(outer[:, 0]):.2f} x {np.ptp(outer[:, 1]):.2f} mm")
    print("  holes/interior features are the trustworthy part; verify "
          "critical OUTER dimensions with calipers (edge shadow/halo)")


# ---------------------------------------------------------------- tailor-stl

def _read_stl(path):
    """Return (n, 3, 3) vertex array and whether the input was binary."""
    import struct
    with open(path, "rb") as f:
        data = f.read()
    if len(data) >= 84:
        n = struct.unpack_from("<I", data, 80)[0]
        if len(data) == 84 + 50 * n:                       # binary
            arr = np.frombuffer(data[84:], dtype=np.uint8)
            arr = arr.reshape(n, 50)[:, :48].copy().view("<f4").reshape(n, 4, 3)
            return arr[:, 1:, :].astype(np.float64), True
    tris = []
    cur = []
    for line in data.decode("ascii", "replace").splitlines():   # ascii
        parts = line.split()
        if parts[:1] == ["vertex"]:
            cur.append([float(v) for v in parts[1:4]])
            if len(cur) == 3:
                tris.append(cur)
                cur = []
    if not tris:
        raise SystemExit(f"{path}: not a valid STL")
    return np.array(tris, np.float64), False


def _write_stl(path, tris, binary):
    import struct
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    nrm = np.cross(b - a, c - a)
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-30)
    if binary:
        with open(path, "wb") as f:
            f.write(b"scancal tailor-stl".ljust(80, b"\0"))
            f.write(struct.pack("<I", len(tris)))
            block = np.zeros((len(tris), 50), np.uint8)
            block[:, :48] = np.hstack([nrm, tris.reshape(-1, 9)]) \
                .astype("<f4").view(np.uint8).reshape(len(tris), 48)
            f.write(block.tobytes())
    else:
        with open(path, "w") as f:
            f.write("solid tailored\n")
            for k in range(len(tris)):
                f.write(f" facet normal {nrm[k,0]:g} {nrm[k,1]:g} {nrm[k,2]:g}\n"
                        f"  outer loop\n")
                for v in tris[k]:
                    f.write(f"   vertex {v[0]:g} {v[1]:g} {v[2]:g}\n")
                f.write("  endloop\n endfacet\n")
            f.write("endsolid tailored\n")


def cmd_tailor_stl(args):
    with open(args.tailoring) as f:
        tail = json.load(f)
    if tail.get("type") != "printer":
        raise SystemExit(f"{args.tailoring} is not a printer tailoring file "
                         f"(use the one calibrate writes via --printer-output)")
    ex = tail["scale_error_pct_x"] / 100
    ey = tail["scale_error_pct_y"] / 100

    tris, binary = _read_stl(args.stl)
    tris[:, :, 0] /= 1 + ex        # inverse pre-compensation, X
    tris[:, :, 1] /= 1 + ey        # Y; Z untouched
    _write_stl(args.output, tris, binary)

    lo, hi = tris.min(axis=(0, 1)), tris.max(axis=(0, 1))
    print(f"wrote {args.output} ({'binary' if binary else 'ascii'}, "
          f"{len(tris)} facets)")
    print(f"  applied printer correction: X x{1/(1+ex):.5f}  Y x{1/(1+ey):.5f}")
    print(f"  tailored bounding box: {hi[0]-lo[0]:.3f} x {hi[1]-lo[1]:.3f} x "
          f"{hi[2]-lo[2]:.3f} mm")
    print("  note: XY only; print in the same XY orientation as the "
          "calibration plate")


# ---------------------------------------------------------------- cli

def main():
    ap = argparse.ArgumentParser(prog="scancal", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    g = sub.add_parser("generate", help="write the reference plate STL "
                                        "(fixed 100x120 mm design)")
    g.add_argument("-o", "--output", default="target.stl")
    g.set_defaults(func=cmd_generate)

    c = sub.add_parser("calibrate", help="analyze plate scan(s), write tailoring")
    c.add_argument("scans", nargs="+",
                   help="one or more scans of the plate (more scans average "
                        "out scanner jitter)")
    c.add_argument("--dpi", type=float, required=True, help="scanner DPI")
    c.add_argument("--x", type=float, nargs="+",
                   help="measured X (short edge, ~100 mm) dimension(s); "
                        "repeat to average multiple measurements")
    c.add_argument("--y", type=float, nargs="+",
                   help="measured Y (long edge, ~120 mm) dimension(s)")
    c.add_argument("--note", default="", help="free-text scanner identifier")
    c.add_argument("-o", "--output", default="tailoring.json")
    c.add_argument("--printer-output", default="printer-tailoring.json",
                   help="printer tailoring file (measured mode only)")
    c.add_argument("--printer-note", default="",
                   help="free-text printer identifier")
    c.set_defaults(func=cmd_calibrate)

    r = sub.add_parser("correct", help="apply tailoring file to a scan")
    r.add_argument("scan")
    r.add_argument("-t", "--tailoring", required=True)
    r.add_argument("--dpi", type=float,
                   help="DPI of this scan (default: calibration DPI)")
    r.add_argument("-o", "--output", default="corrected.png")
    r.add_argument("--dxf", metavar="OUT.dxf",
                   help="extract part outline + holes and write a DXF in "
                        "true mm instead of the corrected image")
    r.add_argument("--min-hole", type=float, default=1.0, metavar="MM",
                   help="ignore holes smaller than this diameter (default 1)")
    r.set_defaults(func=cmd_correct)

    s = sub.add_parser("tailor-stl", help="pre-scale an STL by the inverse "
                                          "printer error (XY only)")
    s.add_argument("stl")
    s.add_argument("-t", "--tailoring", required=True,
                   help="printer tailoring file from calibrate")
    s.add_argument("-o", "--output", default="tailored.stl")
    s.set_defaults(func=cmd_tailor_stl)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
