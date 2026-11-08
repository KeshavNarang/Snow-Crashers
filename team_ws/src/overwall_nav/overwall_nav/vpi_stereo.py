"""GPU stereo matching with NVIDIA VPI, as a drop-in for OpenCV's StereoSGBM in ``StereoRig``.

The Jetson's GPU sits mostly idle while ``stereo_depth`` runs SGBM on the CPU, sharing six cores
with cuVSLAM. VPI's stereo disparity estimator (semi-global matching on a census cost) runs on
the GPU (``cuda``) or on Orin's optical-flow engine (``ofa``, where the hardware has one).

``VpiMatcher.compute(left, right)`` takes two rectified mono8 images and returns disparity as
int16 in 1/16 px, exactly like ``cv2.StereoSGBM.compute``, so the rest of ``StereoRig.points``
(texture and saturation filters, 3D points) is unchanged. Invalid pixels come back <= 0.

VPI is in JetPack (``python3.10-vpi3`` on the host; the skybeta container has ``libnvvpi.so.3``
but not the Python module). ``available()`` says whether it can be imported here.
"""
import numpy as np


def available():
    try:
        import vpi  # noqa: F401
    except ImportError:
        return False
    return True


class VpiMatcher:

    def __init__(self, num_disparities, backend='cuda', window=5, p1=3, p2=48, uniqueness=-1.0,
                 conf_threshold=32767, quality=6, num_passes=3):
        import vpi
        self.vpi = vpi
        self.kind = backend
        self.backend = {'cuda': vpi.Backend.CUDA, 'ofa': vpi.Backend.OFA}[backend]
        if backend == 'ofa' and num_disparities not in (128, 256):
            raise ValueError('the OFA backend only searches 128 or 256 disparities')
        self.maxdisp = int(num_disparities)
        self.args = dict(window=int(window), maxdisp=self.maxdisp, mindisp=0, p1=int(p1), p2=int(p2),
                         confthreshold=int(conf_threshold), quality=int(quality))
        if backend == 'cuda':
            self.args['uniqueness'] = float(uniqueness)
        else:
            self.args['numpasses'] = int(num_passes)
        self.stream = vpi.Stream()

    def compute(self, left, right):
        vpi = self.vpi
        with self.stream, vpi.Backend.CUDA:
            lv = vpi.asimage(np.ascontiguousarray(left)).convert(vpi.Format.Y16_ER)
            rv = vpi.asimage(np.ascontiguousarray(right)).convert(vpi.Format.Y16_ER)
        if self.kind == 'ofa':                      # OFA reads block-linear images
            with self.stream, vpi.Backend.VIC:
                lv = lv.convert(vpi.Format.Y16_ER_BL)
                rv = rv.convert(vpi.Format.Y16_ER_BL)
        with self.stream, self.backend:
            disparity = vpi.stereodisp(lv, rv, **self.args)
        if disparity.format == vpi.Format.S16_BL:
            with self.stream, vpi.Backend.VIC:
                disparity = disparity.convert(vpi.Format.S16)
        self.stream.sync()
        q105 = np.array(disparity.cpu(), dtype=np.int16, copy=True)   # Q10.5: 1/32 px
        return q105 // 2                                                # -> 1/16 px, like SGBM
