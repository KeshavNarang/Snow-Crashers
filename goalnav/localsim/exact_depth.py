"""Exact depth on the course's planar boxes, for the frames seen on the pad.

The voxel renderer (``localsim.world``) places a wall anywhere within about one
2.5 cm voxel and repeats each ray over an 8 x 8 pixel block. That is fine for
the obstacle guard but biases the launch alignment (``drone_ref.launch_align``)
by about 0.7 degree and 2 cm. Here rays that hit near a low-polygon group
(walls, nets, floor, roof) are re-intersected exactly with its triangles, and
the block image is upsampled by interpolating inverse depth (exact on planes;
across depth edges the nearest block is kept, as stereo shows them).
"""
import numpy as np

# Groups with at most this many triangles are boxes or plates: intersect exactly.
MAX_TRIANGLES = 64


class ExactFaces:
    def __init__(self, mesh, skip=()):
        vertices = np.asarray(mesh.vertices, dtype=float)
        tris = [t for name, group in mesh.groups.items()
                if 0 < len(group) <= MAX_TRIANGLES and not name.startswith(tuple(skip)) for t in group]
        tri = vertices[np.asarray(tris)]
        self.a, self.e1, self.e2 = tri[:, 0], tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]

    def intersect(self, origin, directions, chunk=2048):
        """Nearest exact hit along each unit ray (inf if none) and |cos| of
        the incidence angle there."""
        best = np.full(len(directions), np.inf)
        cosine = np.ones(len(directions))
        s = origin - self.a                                       # (T, 3)
        q = np.cross(s, self.e1)                                  # (T, 3)
        normal = np.cross(self.e1, self.e2)
        normal /= np.linalg.norm(normal, axis=1)[:, None]
        for start in range(0, len(directions), chunk):
            d = directions[start:start + chunk]
            p = np.cross(d[:, None, :], self.e2[None])            # (R, T, 3)
            det = np.einsum('rtk,tk->rt', p, self.e1)
            with np.errstate(divide='ignore', invalid='ignore'):
                inv = 1. / det
                u = np.einsum('rtk,tk->rt', p, s) * inv
                v = np.einsum('rk,tk->rt', d, q) * inv
                t = np.einsum('tk,tk->t', self.e2, q)[None] * inv
            ok = (np.abs(det) > 1e-12) & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-6)
            t = np.where(ok, t, np.inf)
            nearest = t.argmin(axis=1)
            rows = np.arange(len(d))
            best[start:start + chunk] = t[rows, nearest]
            cosine[start:start + chunk] = np.abs(np.einsum('rk,rk->r', d, normal[nearest]))
        return best, cosine

    def refine(self, origin, directions, hit, tolerance):
        """Replace voxel hits that lie within tolerance of an exact face (measured
        along its normal: at grazing incidence voxels protrude far along the ray)."""
        exact, cosine = self.intersect(origin, directions)
        with np.errstate(invalid='ignore'):
            explained = (exact - hit) * cosine <= tolerance
        return np.where(explained | ~np.isfinite(hit), exact, hit)


def upsample(block_mm, stride, width, height, edge=0.05):
    """Block depth (mm, 0 = none) -> full image, inverse depth interpolated
    between block centres unless the four neighbours differ by more than edge."""
    rows, cols = block_mm.shape
    inverse = np.where(block_mm > 0, 1. / np.maximum(block_mm, 1), np.nan)
    # Ray k of render_depth samples pixel stride / 2 + k * stride.
    v = np.clip((np.arange(height) - stride / 2) / stride, 0, rows - 1)
    u = np.clip((np.arange(width) - stride / 2) / stride, 0, cols - 1)
    v0, u0 = np.minimum(v.astype(int), rows - 2), np.minimum(u.astype(int), cols - 2)
    fv, fu = (v - v0)[:, None], (u - u0)[None, :]
    corners = [inverse[np.ix_(v0 + i, u0 + j)] for i in (0, 1) for j in (0, 1)]
    blend = (corners[0] * (1 - fv) * (1 - fu) + corners[1] * (1 - fv) * fu
             + corners[2] * fv * (1 - fu) + corners[3] * fv * fu)
    stack = np.nan_to_num(np.stack(corners), nan=np.inf)
    smooth = np.all(np.isfinite(stack), axis=0) & (stack.max(axis=0) <= stack.min(axis=0) * (1 + edge))
    nearest = np.repeat(np.repeat(block_mm, stride, axis=0), stride, axis=1)[:height, :width]
    with np.errstate(divide='ignore', invalid='ignore'):
        out = np.where(smooth, np.round(1. / blend), nearest)
    return np.nan_to_num(out, nan=0.).astype(np.uint16)
