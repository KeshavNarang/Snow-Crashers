"""driver_params: build the depth-enabled OAK driver configuration safely.

The candidate is the RUNNING driver's own parameter dump (`ros2 param dump /oak`) plus a small overlay, so nothing
the VIO depends on is retyped by hand. check_candidate() refuses anything that changes or drops a non-stereo key,
leaves the geometry-critical stereo settings implicit, or combines sizes the driver would publish inconsistently.

Facts these rules come from (depthai-ros v3.3.0-humble, the installed version):
  * pipeline types with a StereoDepth node: DEPTH, RGBD (pipeline_generator.cpp, matched case-insensitively);
  * stereo.i_aligned defaults to true and stereo.i_board_socket_id to CAM_A (0), the RGB camera, which the DEPTH
    pipeline does not run: depth camera_info/frame would describe the wrong camera (stereo_param_handler.cpp);
  * the stereo node feeds StereoDepth from the cameras at the stereo size with ImgResizeMode::CROP, so a stereo size
    with another aspect than the cameras' would crop the field of view (stereo.cpp);
  * rectified outputs take their camera_info size from left/right i_width/i_height (stereo.cpp setupRectQueue).
"""
import yaml

PRESETS = ('FAST_ACCURACY', 'DEFAULT', 'FACE', 'HIGH_DETAIL', 'ROBOTICS')
SOCKETS = {'left': 1, 'right': 2}                     # dai::CameraBoardSocket CAM_B, CAM_C
DEPTH_PIPELINES = ('DEPTH', 'RGBD')
REQUIRED = ('stereo.i_publish_topic', 'stereo.i_aligned', 'stereo.i_board_socket_id', 'stereo.i_width',
            'stereo.i_height')
PIPELINE_KEY = 'pipeline_gen.i_pipeline_type'


def flatten(d, prefix=''):
    out = {}
    for k, v in d.items():
        key = f'{prefix}{k}'
        if isinstance(v, dict):
            out.update(flatten(v, key + '.'))
        else:
            out[key] = v
    return out


def unflatten(flat):
    out = {}
    for key, v in flat.items():
        parts = key.split('.')
        cur = out
        for p in parts[:-1]:
            cur = cur.setdefault(p, {})
        cur[parts[-1]] = v
    return out


def parse_dump(text):
    """Flat {dotted.name: value} from a `ros2 param dump` of exactly one node."""
    data = yaml.safe_load(text)
    if not isinstance(data, dict) or len(data) != 1:
        raise ValueError('expected the parameter dump of exactly one node')
    (node, body), = data.items()
    if not isinstance(body, dict) or not isinstance(body.get('ros__parameters'), dict):
        raise ValueError(f'{node!r} has no ros__parameters')
    return flatten(body['ros__parameters'])


def to_param_file(flat, node='/oak'):
    return yaml.safe_dump({node: {'ros__parameters': unflatten(flat)}}, default_flow_style=False, sort_keys=True)


def depth_overlay(size=(640, 400), fps=30.0, align='left', preset='DEFAULT', extended=False, subpixel=True,
                  rect=False, pipeline='depth'):
    """The stereo settings to add. align: 'left' / 'right' (depth reprojected to that camera) or 'none' (StereoDepth's
    own reference; camera_info then nominally left, to be checked by eval)."""
    w, h = (int(size[0]), int(size[1]))
    if w <= 0 or h <= 0 or w * 10 != h * 16:
        raise ValueError(f'stereo size {w}x{h} is not 16:10 like the OV9282 cameras')
    if align not in ('left', 'right', 'none'):
        raise ValueError(f'align must be left, right or none, not {align!r}')
    if preset not in PRESETS:
        raise ValueError(f'unknown depth preset {preset!r}; one of {PRESETS}')
    if not fps or fps <= 0:
        raise ValueError('fps must be positive')
    return {PIPELINE_KEY: pipeline,
            'stereo.i_publish_topic': True,
            'stereo.i_aligned': align != 'none',
            'stereo.i_board_socket_id': SOCKETS.get(align, 1),
            'stereo.i_width': w, 'stereo.i_height': h, 'stereo.i_fps': float(fps),
            'stereo.i_depth_preset': preset,
            'stereo.i_extended_disp': bool(extended), 'stereo.i_subpixel': bool(subpixel),
            'stereo.i_left_rect_publish_topic': bool(rect), 'stereo.i_right_rect_publish_topic': bool(rect)}


def build_candidate(dump, overlay):
    cand = dict(dump)
    cand.update(overlay)
    return cand


def _allowed(key):
    return key == PIPELINE_KEY or key.startswith('stereo.')


def check_candidate(dump, cand):
    """Problems that make `cand` unsafe or inconsistent, as readable strings. [] = acceptable."""
    problems = []
    for k, v in dump.items():
        if k not in cand:
            problems.append(f'{k}: dropped from the running configuration')
        elif cand[k] != v and not _allowed(k):
            problems.append(f'{k}: changed from {v!r} to {cand[k]!r}; only {PIPELINE_KEY} and stereo.* may change')
    for k in cand:
        if k not in dump and not _allowed(k):
            problems.append(f'{k}: new key outside stereo.*')
    if str(cand.get(PIPELINE_KEY, '')).upper() not in DEPTH_PIPELINES:
        problems.append(f'{PIPELINE_KEY}={cand.get(PIPELINE_KEY)!r}: this pipeline has no stereo depth node '
                        f'(need one of {DEPTH_PIPELINES})')
    for k in REQUIRED:
        if k not in cand:
            problems.append(f'{k}: must be set explicitly (its default is wrong for this camera)')
    if cand.get('stereo.i_board_socket_id') == 0 and str(cand.get(PIPELINE_KEY, '')).upper() == 'DEPTH':
        problems.append('stereo.i_board_socket_id=0 is the RGB camera, which the DEPTH pipeline does not run')
    sw, sh = cand.get('stereo.i_width'), cand.get('stereo.i_height')
    lw, lh = cand.get('left.i_width'), cand.get('left.i_height')
    if None not in (sw, sh, lw, lh):
        if sw * lh != sh * lw:
            problems.append(f'stereo size {sw}x{sh} has a different aspect from the cameras {lw}x{lh} '
                            '(CROP resize would cut the field of view)')
        if (cand.get('stereo.i_left_rect_publish_topic') or cand.get('stereo.i_right_rect_publish_topic')) \
                and (sw, sh) != (lw, lh):
            problems.append(f'rect outputs at the stereo size {sw}x{sh} would carry camera_info sized {lw}x{lh}')
    return problems
