"""Unit tests for the vision code on real photos of the lab zone markers.
Run:  cd ~/ros2_ws/src/ecte477_cr_final && python3 -m pytest -q test/test_vision_utils.py"""
import os

import cv2
import numpy as np

from ecte477_cr_final import vision_utils as vu

HERE = os.path.dirname(__file__)
ROOT = os.path.dirname(HERE)
CFG = vu.load_vision_config(os.path.join(ROOT, 'config', 'vision.yaml'))
R = CFG['hsv_ranges']
COLS = CFG['zone_colours']
CLF = vu.OnnxClassifier(os.path.join(ROOT, 'models', 'object_cnn.onnx'),
                        ['background', 'light_bulb'])


def img(name):
    return cv2.imread(os.path.join(HERE, 'data', name))


def test_blue_zone_counts_only_blue_squares():
    r = vu.analyse_zone_frame(img('zone_blue.jpg'), R, COLS, classifier=CLF)
    assert r['colour'] == 'blue'
    assert r['shapes'] == {'square': 4}          # the 2 pink squares + 2 pink circles ignored


def test_pink_zone_shapes():
    r = vu.analyse_zone_frame(img('zone_pink.jpg'), R, COLS, classifier=CLF)
    assert r['colour'] == 'pink'
    assert r['shapes'] == {'circle': 3, 'triangle': 1, 'square': 1}


def test_yellow_zone_two_bulbs():
    r = vu.analyse_zone_frame(img('zone_yellow.jpg'), R, COLS, classifier=CLF)
    assert r['colour'] == 'yellow'
    assert r['objects'] == {'light_bulb': 2}


def test_pink_marker_is_not_a_red_sheet():
    pink = cv2.resize(img('zone_pink.jpg')[60:200, 40:200], (160, 120))
    assert vu.detect_sheet(pink, R)[0] == 'NONE'


def test_red_and_green_sheets():
    for colour, expect in (((35, 25, 190), 'RED'), ((60, 160, 40), 'GREEN')):
        frame = cv2.resize(img('zone_pink.jpg'), (160, 120))
        box = cv2.boxPoints(((80, 60), (90, 64), 15)).astype(np.int32)
        cv2.fillPoly(frame, [box], colour)
        assert vu.detect_sheet(frame, R)[0] == expect


def test_debouncer_needs_3_of_4():
    d = vu.Debouncer(4, 3)
    assert [d.update(x) for x in ['RED', 'NONE', 'RED']] == [False, False, False]
    assert d.update('RED') and d.state == 'RED'
    assert not d.update('NONE') and d.state == 'RED'     # latched until GREEN


def test_navy_lab_wall_is_not_a_zone():
    # real robot-camera frame facing the navy lab wall (no marker in view)
    navy = img('robot_navy.png')
    assert vu.analyse_zone_frame(navy, R, COLS)['colour'] == 'unknown'
