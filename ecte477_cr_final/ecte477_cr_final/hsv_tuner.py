#!/usr/bin/env python3
"""
hsv_tuner.py - tune an HSV range on saved images (no ROS needed).

  ros2 run ecte477_cr_final hsv_tuner ~/cr_snapshots/snap_x.png [more.png ...] --colour red
Trackbars set H/S/V low/high; the window shows the mask. Keys:
  n = next image, p = print the YAML line for config/vision.yaml, q = quit.
Note: red needs TWO ranges (H 0-7 and 173-179) because red wraps around H=0/180.
"""
import argparse
import os

import cv2
import numpy as np

from ecte477_cr_final import vision_utils as vu


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('images', nargs='+')
    ap.add_argument('--colour', default='pink')
    a = ap.parse_args()
    start = vu.DEFAULT_RANGES.get(a.colour, [[0, 80, 60, 179, 255, 255]])[0]
    imgs = [vu.resize_to_width(cv2.imread(os.path.expanduser(p)), 480) for p in a.images]
    win = 'hsv_tuner'
    cv2.namedWindow(win)
    names = ['H low', 'S low', 'V low', 'H high', 'S high', 'V high']
    for n, v, mx in zip(names, start, [179, 255, 255, 179, 255, 255]):
        cv2.createTrackbar(n, win, int(v), mx, lambda _: None)
    i = 0
    while True:
        vals = [cv2.getTrackbarPos(n, win) for n in names]
        hsv = cv2.cvtColor(imgs[i], cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(vals[:3], np.uint8), np.array(vals[3:], np.uint8))
        view = np.hstack([imgs[i], cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
        cv2.imshow(win, view)
        k = cv2.waitKey(50) & 0xFF
        if k == ord('q'):
            break
        if k == ord('n'):
            i = (i + 1) % len(imgs)
        if k == ord('p'):
            print(f'  {a.colour}: [{vals}]')
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
