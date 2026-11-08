"""Floor positions of the rebuilt walls, from photos/New_Walls_Overhead{1,2}.png.

For each overhead photo: a floor homography fitted to the floor tape's corners
(CAD 'Guideline (zone 1)'), then the stand feet and pavers clicked by hand in the
photo mapped onto the floor. Only points on the floor are valid (the photos are
wide-angle from ~2 m up: anything tall lands far from its foot). Run from the
repo root: python agents/logs/2026-11-07-walls-rebuilt/measure.py
"""
import cv2
import numpy as np

# Tape corners (course frame, m): A the launch line's north end, then along the line.
TAPE = {'A': (-2.515, 2.585), 'launch': (-2.515, 0.889), 'B': (-1.372, 2.585), 'C': (-1.372, 0.972),
        'D': (-0.254, 0.972), 'E': (-0.254, 2.585)}
PHOTOS = {
    'New_Walls_Overhead1.png': {      # 3000x2250, taken with the phone turned: north is down
        'tape': {'A': (1890, 1225), 'B': (1469, 1213), 'C': (1340, 718), 'D': (890, 765), 'E': (1100, 1198)},
        'points': {'M3 paver south': (740, 775), 'M3 paver north': (710, 975),
                   'NW post foot east': (1465, 1250), 'NW post foot west': (1930, 1270),
                   'tape east end': (675, 1197)}},
    'New_Walls_Overhead2.png': {      # 1320x999, north up
        'tape': {'A': (493, 428), 'launch': (490, 680), 'B': (697, 425), 'C': (735, 650), 'D': (937, 622),
                 'E': (852, 427)},
        'points': {'M3 paver south': (1005, 620), 'M3 paver north': (1025, 530),
                   'NW post foot east': (685, 435), 'NW post foot west': (475, 420),
                   'tape east end': (1045, 422), 'net NW floor corner (check: -3.05, 3.05)': (412, 385)}},
}

for name, photo in PHOTOS.items():
    keys = list(photo['tape'])
    world = np.array([TAPE[k] for k in keys], float)
    pixels = np.array([photo['tape'][k] for k in keys], float)
    H, _ = cv2.findHomography(world, pixels, 0)
    fit = cv2.perspectiveTransform(world[None], H)[0]
    print(f'{name}: tape fit residuals (px) {np.round(np.linalg.norm(fit - pixels, axis=1), 1).tolist()}')
    inverse = np.linalg.inv(H)
    for label, uv in photo['points'].items():
        xy = cv2.perspectiveTransform(np.array([[uv]], float), inverse)[0, 0]
        print(f'  {label:42s} ({xy[0]:+.2f}, {xy[1]:+.2f})')
