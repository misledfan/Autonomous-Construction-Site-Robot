"""
vision_utils.py - pure OpenCV/NumPy functions (no ROS), so they can be unit-tested
on saved images. Used by sheet_detector.py and zone_verifier.py.

Colour space: OpenCV HSV (H 0-179, S 0-255, V 0-255).
"""
import math
from collections import Counter

import cv2
import numpy as np

# ------------------------------------------------------------------ defaults
# Measured on photos of the lab markers (phone + TB3 C270 camera).
# Pink markers sit at H ~163-168, the brown MDF wall at H ~12-16 (S ~100),
# so red is restricted to H<=7 / H>=173 with high saturation.
DEFAULT_RANGES = {
    'red':    [[0, 140, 60, 7, 255, 255], [173, 140, 60, 179, 255, 255]],
    'green':  [[40, 80, 40, 90, 255, 255]],
    'blue':   [[95, 80, 40, 130, 255, 255]],
    'pink':   [[145, 80, 60, 172, 255, 255]],
    'yellow': [[20, 80, 80, 40, 255, 255]],
}


def resize_to_width(img, width):
    h, w = img.shape[:2]
    if w == width:
        return img
    scale = width / float(w)
    return cv2.resize(img, (width, max(1, int(round(h * scale)))), interpolation=cv2.INTER_AREA)


def colour_mask(hsv, ranges, open_k=3, close_k=5):
    """Binary mask (uint8 0/255) for a colour given as a list of [hL,sL,vL,hH,sH,vH]."""
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for r in ranges:
        mask |= cv2.inRange(hsv, np.array(r[:3], np.uint8), np.array(r[3:], np.uint8))
    if open_k > 1:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((open_k, open_k), np.uint8))
    if close_k > 1:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((close_k, close_k), np.uint8))
    return mask


# ------------------------------------------------------------ sheet detection
def detect_sheet(bgr, ranges, min_area_frac=0.08, min_solidity=0.80,
                 aspect_min=1.0, aspect_max=2.0):
    """Classify one frame as 'RED', 'GREEN' or 'NONE'.

    A colour counts as a presented A4 sheet only if its largest blob
      * covers >= min_area_frac of the frame (sheet held close -> big), and
      * is sheet-shaped: solid (area / convex hull area) and roughly rectangular
        with an aspect ratio near A4 (1.41) in either orientation.
    Returns (label, info_dict)."""
    hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)
    frame_area = float(bgr.shape[0] * bgr.shape[1])
    best = ('NONE', 0.0)
    info = {}
    for label, key in (('RED', 'red'), ('GREEN', 'green')):
        mask = colour_mask(hsv, ranges[key], open_k=5, close_k=7)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            info[label] = {'area_frac': 0.0}
            continue
        c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(c)
        hull_area = max(cv2.contourArea(cv2.convexHull(c)), 1.0)
        (_, _), (rw, rh), _ = cv2.minAreaRect(c)
        aspect = max(rw, rh) / max(min(rw, rh), 1.0)
        frac = area / frame_area
        solidity = area / hull_area
        # If the sheet fills/overflows the frame the aspect is meaningless -> accept.
        big = frac > 0.6
        ok = frac >= min_area_frac and solidity >= min_solidity and \
            (big or aspect_min <= aspect <= aspect_max)
        info[label] = {'area_frac': round(frac, 3), 'solidity': round(solidity, 2),
                       'aspect': round(aspect, 2), 'ok': ok}
        if ok and frac > best[1]:
            best = (label, frac)
    return best[0], info


class Debouncer:
    """'Consistently detected': switch state only when >= k of the last n frames agree."""

    def __init__(self, n=4, k=3, initial='NONE'):
        self.n, self.k = n, k
        self.history = []
        self.state = initial

    def update(self, label):
        self.history.append(label)
        self.history = self.history[-self.n:]
        for cand in ('RED', 'GREEN'):
            if self.history.count(cand) >= self.k and self.state != cand:
                self.state = cand
                return True
        return False


# ------------------------------------------------------------- zone analysis
def marker_mask(mask, min_area_px=150, max_area_frac=0.45, hsv=None, min_paper=0.5,
                edge_rules=True):
    """Keep only blobs that can be printed markers on a white A4 sheet:
      * not touching the BOTTOM edge (floor, e.g. the blue lab floor),
      * not spanning from the left edge to the right edge (curtain / floor stripes),
      * not huge (> 45 % of the frame) and not tiny (< min_area_px),
      * (if hsv is given) surrounded mostly by WHITE PAPER: in a thin ring around the
        blob at least `min_paper` of the pixels must be light and unsaturated.
        The markers are printed on white sheets; the navy lab wall, the brown MDF and
        the floor are not -> they are rejected even when their colour is similar."""
    h, w = mask.shape[:2]
    out = np.zeros_like(mask)
    paper = None
    if hsv is not None:
        paper = ((hsv[..., 1] < 80) & (hsv[..., 2] > 110)).astype(np.uint8)
        k = max(5, w // 40) | 1
        kernel = np.ones((k, k), np.uint8)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        a = cv2.contourArea(c)
        x, y, bw, bh = cv2.boundingRect(c)
        if a < min_area_px or a > max_area_frac * h * w:
            continue
        if edge_rules and (y + bh >= h - 2 or (x <= 2 and x + bw >= w - 2)):
            continue
        blob = np.zeros_like(mask)
        cv2.drawContours(blob, [c], -1, 255, thickness=cv2.FILLED)
        if paper is not None:
            ring = cv2.subtract(cv2.dilate(blob, kernel), blob) > 0
            ring &= mask == 0                     # ignore neighbouring same-colour pixels
            n = int(ring.sum())
            if n == 0 or paper[ring].sum() / float(n) < min_paper:
                continue
        out |= blob
    return cv2.bitwise_and(mask, out) if out.any() else out


def classify_shape(contour):
    """Name a contour: triangle / square / rectangle / pentagon / circle / other."""
    area = cv2.contourArea(contour)
    peri = cv2.arcLength(contour, True)
    if peri <= 0:
        return 'other'
    circularity = 4.0 * math.pi * area / (peri * peri)
    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
    v = len(approx)
    if v == 3:
        return 'triangle'
    if v == 4:
        (_, _), (w, h), _ = cv2.minAreaRect(contour)
        ar = max(w, h) / max(min(w, h), 1.0)
        return 'square' if ar <= 1.25 else 'rectangle'
    if v == 5 and circularity < 0.85:
        return 'pentagon'
    if circularity >= 0.75:
        return 'circle'
    return 'other'


def find_shapes(mask, min_area_px=150, border_px=2):
    """External contours of one colour mask, ignoring tiny blobs and blobs cut by the
    image border (a partly visible marker cannot be classified reliably)."""
    h, w = mask.shape[:2]
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    shapes = []
    for c in cnts:
        if cv2.contourArea(c) < min_area_px:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        if x <= border_px or y <= border_px or x + bw >= w - border_px or y + bh >= h - border_px:
            continue
        shapes.append((classify_shape(c), c))
    return shapes


def find_object_regions(mask, min_area_px=120, max_area_frac=0.5):
    """Objects printed ON the zone colour (e.g. bulbs on the yellow disc) appear as
    holes in the colour mask. Fill each external blob, subtract the mask -> holes."""
    filled = np.zeros_like(mask)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(filled, cnts, -1, 255, thickness=cv2.FILLED)
    holes = cv2.subtract(filled, mask)
    holes = cv2.morphologyEx(holes, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    hcnts, _ = cv2.findContours(holes, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    total = float(mask.shape[0] * mask.shape[1])
    boxes = []
    for c in hcnts:
        a = cv2.contourArea(c)
        if a < min_area_px or a > max_area_frac * total:
            continue
        boxes.append(cv2.boundingRect(c))
    return boxes


def crop_box(img, box, pad_frac=0.15):
    x, y, w, h = box
    p = int(round(pad_frac * max(w, h)))
    H, W = img.shape[:2]
    x0, y0 = max(0, x - p), max(0, y - p)
    x1, y1 = min(W, x + w + p), min(H, y + h + p)
    return img[y0:y1, x0:x1]


def analyse_zone_frame(bgr, ranges, zone_colours, min_zone_frac=0.01,
                       min_shape_area_px=150, classifier=None, min_obj_score=0.7):
    """Analyse ONE frame.
    zone_colours: list of colour names that identify zones, e.g. ['blue','pink','yellow'].
    Returns dict with winner colour, dominance share, shape counts (winner colour only)
    and object counts (CNN on holes in the winner colour)."""
    hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (3, 3), 0), cv2.COLOR_BGR2HSV)
    frame_area = float(bgr.shape[0] * bgr.shape[1])
    masks, areas = {}, {}
    for col in zone_colours:
        masks[col] = marker_mask(colour_mask(hsv, ranges[col]), min_shape_area_px, hsv=hsv)
        areas[col] = float(cv2.countNonZero(masks[col]))
    total = sum(areas.values())
    winner = max(areas, key=areas.get) if areas else None
    if winner is None or areas[winner] / frame_area < min_zone_frac:
        return {'colour': 'unknown', 'dominance': 0.0, 'shapes': {}, 'objects': {},
                'areas': areas}
    shapes = Counter(name for name, _ in find_shapes(masks[winner], min_shape_area_px))
    objects = Counter()
    if classifier is not None:
        for box in find_object_regions(masks[winner]):
            crop = crop_box(bgr, box)
            label, score = classifier.classify(crop)
            if label != 'background' and score >= min_obj_score:
                objects[label] += 1
    return {'colour': winner, 'dominance': round(areas[winner] / max(total, 1.0), 3),
            'shapes': dict(shapes), 'objects': dict(objects), 'areas': areas}


def combine_frames(results):
    """Majority vote across frames: zone colour by mode, then counts by mode among the
    frames that agree on that colour. Returns (summary_dict, frames_agreeing)."""
    if not results:
        return {'colour': 'unknown', 'dominance': 0.0, 'shapes': {}, 'objects': {}}, 0
    colour = Counter(r['colour'] for r in results).most_common(1)[0][0]
    same = [r for r in results if r['colour'] == colour]

    def key(r):
        return (tuple(sorted(r['shapes'].items())), tuple(sorted(r['objects'].items())))
    best_key, agree = Counter(key(r) for r in same).most_common(1)[0]
    dominance = float(np.median([r['dominance'] for r in same]))
    return {'colour': colour, 'dominance': round(dominance, 3),
            'shapes': dict(best_key[0]), 'objects': dict(best_key[1])}, agree


# ---------------------------------------------------------------- CNN wrapper
class OnnxClassifier:
    """Runs the exported CNN (models/object_cnn.onnx) with cv2.dnn - no PyTorch needed.
    Input: grayscale crop resized to size x size, scaled to [0,1]. Output: softmax."""

    def __init__(self, model_path, labels, size=40):
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.labels = list(labels)
        self.size = size

    def classify(self, crop_bgr):
        if crop_bgr is None or crop_bgr.size == 0:
            return 'background', 0.0
        gray = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2GRAY) if crop_bgr.ndim == 3 else crop_bgr
        x = cv2.resize(gray, (self.size, self.size), interpolation=cv2.INTER_AREA)
        x = x.astype(np.float32)[None, None] / 255.0
        self.net.setInput(x)
        out = self.net.forward().reshape(-1)
        i = int(np.argmax(out))
        return self.labels[i], float(out[i])


# ------------------------------------------------------------- annotation
def annotate(bgr, ranges, colour, classifier=None, min_shape_area_px=150, min_obj_score=0.7):
    """Draw detected shapes (zone colour only) and CNN objects on a copy of the frame."""
    out = bgr.copy()
    if colour not in ranges:
        return out
    hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (3, 3), 0), cv2.COLOR_BGR2HSV)
    mask = marker_mask(colour_mask(hsv, ranges[colour]), min_shape_area_px, hsv=hsv)
    for name, c in find_shapes(mask, min_shape_area_px):
        cv2.drawContours(out, [c], -1, (0, 255, 0), 2)
        x, y, _, _ = cv2.boundingRect(c)
        cv2.putText(out, name, (x, max(10, y - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    if classifier is not None:
        for box in find_object_regions(mask):
            label, score = classifier.classify(crop_box(bgr, box))
            if label != 'background' and score >= min_obj_score:
                x, y, w, h = box
                cv2.rectangle(out, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(out, f'{label} {score:.2f}', (x, y + h + 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
    return out


def load_vision_config(path):
    """Read config/vision.yaml (plain YAML, shared by nodes and offline tools)."""
    import yaml
    with open(path, 'r') as f:
        cfg = yaml.safe_load(f) or {}
    ranges = dict(DEFAULT_RANGES)
    ranges.update(cfg.get('hsv_ranges', {}))
    cfg['hsv_ranges'] = ranges
    return cfg


# ------------------------------------------------------------- alignment
def zone_target(bgr, ranges, zone_colours, min_frac=0.003, only=None):
    """Where is the zone marker in the image?  Returns (colour, x_error, area_frac, cut):
    x_error in [-1, 1] (negative = marker left of centre), area_frac = share of the
    frame covered by the dominant zone colour (grows as the robot gets closer).
    cut = True if the marker touches the image border (part of it is outside the view).
    colour is None if no zone colour is visible."""
    hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (3, 3), 0), cv2.COLOR_BGR2HSV)
    h, w = bgr.shape[:2]
    best = (None, 0, None)
    min_px = max(40, int(0.003 * h * w))
    for col in ([only] if only in ranges else zone_colours):
        m = marker_mask(colour_mask(hsv, ranges[col]), min_px, hsv=hsv)
        a = cv2.countNonZero(m)
        if a > best[1]:
            best = (col, a, m)
    col, area, mask = best
    if col is None or area < min_frac * h * w:
        return None, 0.0, 0.0, False
    mom = cv2.moments(mask, binaryImage=True)
    cx = mom['m10'] / max(mom['m00'], 1.0)
    # "cut off": a paper-backed blob of this colour touches ANY image edge (incl. the
    # bottom, which the marker filter itself ignores because of the floor)
    loose = marker_mask(colour_mask(hsv, ranges[col]), min_px, hsv=hsv, edge_rules=False,
                        min_paper=0.2)
    b = 2
    cut = bool(loose[:b].any() or loose[-b:].any() or loose[:, :b].any() or loose[:, -b:].any())
    return col, (cx - w / 2.0) / (w / 2.0), area / float(h * w), cut
