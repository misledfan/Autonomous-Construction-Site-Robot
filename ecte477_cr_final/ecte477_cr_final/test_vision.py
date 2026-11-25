#!/usr/bin/env python3
"""
test_vision.py - run the SAME vision code the nodes use on saved images (no robot needed).

  ros2 run ecte477_cr_final test_vision ~/cr_snapshots/*.png
Prints sheet detection (RED/GREEN/NONE) and zone analysis for each image and writes
<name>_annotated.png next to it. Good evidence for the report.
"""
import os
import sys

import cv2

from ecte477_cr_final import vision_utils as vu
from ecte477_cr_final.common import share_path


def main():
    if len(sys.argv) < 2:
        sys.exit('usage: test_vision IMAGE [IMAGE ...]')
    cfg = vu.load_vision_config(share_path('config', 'vision.yaml'))
    ranges, z, s = cfg['hsv_ranges'], cfg.get('zone', {}), cfg.get('sheet', {})
    model = share_path('models', z.get('cnn_model', 'object_cnn.onnx'))
    labels = [line.strip() for line in open(os.path.splitext(model)[0] + '_labels.txt') if line.strip()]
    clf = vu.OnnxClassifier(model, labels, int(z.get('cnn_input', 40)))
    for p in sys.argv[1:]:
        img = cv2.imread(os.path.expanduser(p))
        if img is None:
            print(p, ': cannot read')
            continue
        sheet, info = vu.detect_sheet(vu.resize_to_width(img, int(s.get('resize_width', 160))),
                                      ranges, float(s.get('min_area_frac', 0.08)),
                                      float(s.get('min_solidity', 0.80)))
        frame = vu.resize_to_width(img, int(z.get('resize_width', 320)))
        r = vu.analyse_zone_frame(frame, ranges, cfg.get('zone_colours', ['blue', 'pink', 'yellow']),
                                  float(z.get('min_zone_frac', 0.01)),
                                  int(z.get('min_shape_area_px', 150)), clf,
                                  float(z.get('cnn_min_score', 0.7)))
        print(f'{p}\n  sheet: {sheet} {info}\n  zone colour: {r["colour"]} '
              f'(dominance {r["dominance"]})  shapes: {r["shapes"]}  objects: {r["objects"]}')
        out = os.path.splitext(p)[0] + '_annotated.png'
        cv2.imwrite(out, vu.annotate(frame, ranges, r['colour'], clf))
        print(f'  annotated -> {out}')


if __name__ == '__main__':
    main()
