"""frames: candidate frames for the device depth, from a host stereo match and the cameras' CameraInfo.

ROS convention: R rotates raw-camera points into the rectified frame (X_rect = R @ X_raw); P projects rectified
points, P[0][3] = -fx' * baseline on whichever camera the driver treats as the second one (on this driver that is
the LEFT camera: i_reverse_stereo_socket_order).
"""
import numpy as np

import geometry as g


def _k(info):
    k = info['k']
    return (k[0], k[4], k[2], k[5])


def _kp(info):
    p = info['p']
    return (p[0], p[5], p[2], p[6])


def _r(info):
    return np.asarray(info['r'], float).reshape(3, 3)


def baseline(info_l, info_r):
    fx = info_r['p'][0] or info_l['p'][0]
    b = max(abs(info_l['p'][3]), abs(info_r['p'][3])) / fx
    if b < 1e-6:
        raise ValueError('neither camera_info carries a baseline in P[0][3]')
    return b


def build_candidates(host_left, host_right, info_l, info_r, info_dev, host_size, dev_size):
    """Host depth (rectified left/right frames, host_size) moved to the device depth's size, in each frame it could be."""
    kl, kr = _kp(info_l), _kp(info_r)
    return {
        'claimed: raw left (its camera_info)': g.reproject_depth(host_left, kl, _r(info_l).T, np.zeros(3),
                                                                 _k(info_dev), dev_size),
        'rectified left': g.reproject_depth(host_left, kl, np.eye(3), np.zeros(3),
                                            g.scale_k(kl, host_size, dev_size), dev_size),
        'rectified right': g.reproject_depth(host_right, kr, np.eye(3), np.zeros(3),
                                             g.scale_k(kr, host_size, dev_size), dev_size),
    }
