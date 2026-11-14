"""The object's pose from one depth image of the wrist camera, with no help from the simulation.

Pure numpy + scipy (no Isaac Sim), so it runs inside Isaac Sim and offline on saved captures
(perception_eval.py).

Frames: poses are 4x4 matrices acting on column vectors. The camera frame is OpenCV's: x right,
y down, z forward (USD cameras look down -z with y up; wrist_camera.py converts).

locate() does it in three steps:
  1. Depth pixels -> points in the world, from the camera's intrinsics and its pose (the arm's
     forward kinematics times the camera mount).
  2. Keep the pick area, fit the table plane (RANSAC) and keep what stands on it, and keep the
     largest connected cluster: the object's visible surface.
  3. Register the object's model (points sampled from its mesh) to it: start from orientations
     spread over all of SO(3), each placed where the model's camera-facing side lands on the
     observed points and its lowest point on the table; rank them coarsely, refine the best few
     hundred with a short ICP (point-to-plane), and the best few of those with a full one. Each
     candidate is scored by how many observed points the model explains, minus model points the
     camera would have seen in front of what it actually saw (free space) and model points sunk
     into the table.

The coarse rank alone is a poor judge: in the first version, which fully refined only its 16
best, starts near the true pose ranked 100th to 450th, and the jar came out upside down in most
noisy images. The scores of the refined candidates pick the right one.

No object is assumed to be symmetric. For a near-symmetric one (the jar about its axis, the
lemon) the orientation about the symmetry is arbitrary, which grasping does not mind.
"""

from dataclasses import dataclass

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import KDTree

# --- tuning (metres unless stated) --------------------------------------------------------------
ROI_RADIUS = 0.18          # the pick area: this far (horizontally) around where the object should be
ROI_MAX_Z = 0.30           # ignore everything higher (the gripper's fingers are in view)
PLANE_THRESHOLD = 0.004    # table plane inliers
MIN_HEIGHT = 0.006         # object points: at least this far above the table plane
CLUSTER_RADIUS = 0.006     # neighbouring points closer than this belong to the same cluster
MIN_POINTS = 50            # fewer object points than this: no estimate
SCENE_VOXEL = 0.003        # observed points are thinned to one per voxel this size
N_ROTATIONS = 4000         # starting orientations, spread over all of SO(3) (about 16 degrees apart)
COARSE_POINTS = 150        # observed points used to rank the starting poses
REFINE = 300               # best-ranked starting poses refined with a short ICP ...
REFINE_ITERATIONS = 6
REFINE_POINTS = 300        # ... on this many observed points
TOP_K = 8                  # best of those after scoring, refined with the full ICP
ICP_ITERATIONS = 40
ICP_MAX_DIST = (0.02, 0.01, 0.005)  # correspondence cut-off, tightened over the iterations
INLIER_DIST = 0.004        # an observed point is explained if the model passes this close
FREE_SPACE_MARGIN = 0.01   # a model point is in seen-free space if the camera saw this much further
TABLE_MARGIN = 0.005       # a model point this far below the table plane is sunk into it


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float  # principal point in continuous pixel coordinates (pixel u spans [u, u + 1))
    cy: float
    width: int
    height: int


@dataclass
class ObjectModel:
    """Points on the object's surface and their outward normals, in its body frame."""
    points: np.ndarray
    normals: np.ndarray

    def __post_init__(self):
        self.tree = KDTree(self.points)
        self.centre = self.points.mean(axis=0)

    @staticmethod
    def from_mesh(vertices: np.ndarray, triangles: np.ndarray, spacing: float = 0.0025, seed: int = 0) -> "ObjectModel":
        """Sample the mesh's surface evenly: many area-weighted random points, then one per voxel."""
        v = np.asarray(vertices, dtype=float)
        t = np.asarray(triangles, dtype=int)
        v0, v1, v2 = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
        cross = np.cross(v1 - v0, v2 - v0)
        area = 0.5 * np.linalg.norm(cross, axis=1)
        keep = area > 1e-12
        v0, v1, v2, cross, area = v0[keep], v1[keep], v2[keep], cross[keep], area[keep]
        rng = np.random.default_rng(seed)
        n = int(min(200_000, max(20_000, 8 * area.sum() / spacing ** 2)))
        idx = rng.choice(len(area), n, p=area / area.sum())
        r1, r2 = rng.random(n), rng.random(n)
        s = np.sqrt(r1)
        pts = (1 - s)[:, None] * v0[idx] + (s * (1 - r2))[:, None] * v1[idx] + (s * r2)[:, None] * v2[idx]
        normals = cross[idx] / np.linalg.norm(cross[idx], axis=1, keepdims=True)
        _, first = np.unique(np.floor(pts / spacing).astype(np.int64), axis=0, return_index=True)
        pts, normals = pts[first], normals[first]
        # the mesh's winding gives outward normals; if most point inwards it is wound the other way
        if np.mean(np.einsum("ij,ij->i", normals, pts - pts.mean(axis=0)) > 0) < 0.5:
            normals = -normals
        return ObjectModel(pts, normals)


@dataclass
class PoseEstimate:
    pose: np.ndarray | None  # the object's body frame in the world
    score: float = float("nan")
    fitness: float = float("nan")    # share of observed points the model explains
    rmse: float = float("nan")       # of the explained points, metres
    violation: float = float("nan")  # share of the model's visible points in seen-free space
    sunk: float = float("nan")       # share of the model's points below the table
    n_points: int = 0                # observed object points
    detail: str = ""


# --- geometry helpers -----------------------------------------------------------------------------
def transform(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    return pts @ T[:3, :3].T + T[:3, 3]


def invert(T: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


def rodrigues(w: np.ndarray) -> np.ndarray:
    theta = np.linalg.norm(w)
    if theta < 1e-12:
        return np.eye(3)
    k = w / theta
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * K @ K


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """(n, 4) unit quaternions (w, x, y, z) -> (n, 3, 3) rotation matrices."""
    w, x, y, z = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1),
    ], -2)


def rotation_grid(n: int) -> np.ndarray:
    """n rotations spread evenly over SO(3): the super-Fibonacci spiral (Alexa, CVPR 2022)."""
    phi, psi = np.sqrt(2.0), 1.533751168755204288118041
    i = np.arange(n) + 0.5
    s = i / n
    r, big_r = np.sqrt(s), np.sqrt(1.0 - s)
    alpha, beta = 2 * np.pi * i / phi, 2 * np.pi * i / psi
    q = np.stack([big_r * np.cos(beta), r * np.sin(alpha), r * np.cos(alpha), big_r * np.sin(beta)], 1)
    return quat_to_matrix(q)


def angle_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    r = Ra[:3, :3].T @ Rb[:3, :3]
    return float(np.degrees(np.arccos(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0))))


# --- step 1 and 2: points, table, object cluster --------------------------------------------------
def backproject(depth: np.ndarray, intr: Intrinsics, z_range=(0.2, 2.0)) -> np.ndarray:
    """Depth image (distance to the image plane, metres) -> points in the camera frame."""
    d = np.asarray(depth, dtype=float).reshape(intr.height, intr.width)
    v, u = np.nonzero(np.isfinite(d) & (d > z_range[0]) & (d < z_range[1]))
    z = d[v, u]
    return np.stack([(u + 0.5 - intr.cx) / intr.fx * z, (v + 0.5 - intr.cy) / intr.fy * z, z], 1)


def fit_plane(points: np.ndarray, threshold: float = PLANE_THRESHOLD, iterations: int = 300,
              seed: int = 0) -> tuple[np.ndarray, float]:
    """RANSAC plane n.p + d = 0, refined by least squares on its inliers; n points up (+z)."""
    rng = np.random.default_rng(seed)
    sample = points if len(points) <= 20_000 else points[rng.choice(len(points), 20_000, replace=False)]
    best, best_count = None, -1
    for _ in range(iterations):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        count = int(np.count_nonzero(np.abs(sample @ n - n @ a) < threshold))
        if count > best_count:
            best, best_count = (n, -n @ a), count
    if best is None:
        raise ValueError("no plane found")
    n, d = best
    inliers = points[np.abs(points @ n + d) < threshold]
    c = inliers.mean(axis=0)
    n = np.linalg.svd(inliers - c, full_matrices=False)[2][2]
    n = n if n[2] > 0 else -n
    return n, float(-n @ c)


def largest_cluster(points: np.ndarray, radius: float = CLUSTER_RADIUS) -> np.ndarray:
    """Boolean mask of the biggest group of points connected by gaps smaller than radius."""
    pairs = KDTree(points).query_pairs(radius, output_type="ndarray")
    n = len(points)
    graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(n, n))
    _, labels = connected_components(graph, directed=False)
    return labels == np.argmax(np.bincount(labels))


def voxel_downsample(points: np.ndarray, voxel: float) -> np.ndarray:
    """One point per voxel: the mean of the points in it."""
    _, inverse, counts = np.unique(np.floor(points / voxel).astype(np.int64), axis=0,
                                   return_inverse=True, return_counts=True)
    inverse = inverse.reshape(-1)
    sums = np.zeros((len(counts), 3))
    np.add.at(sums, inverse, points)
    return sums / counts[:, None]


def segment_object(points_world: np.ndarray, roi_centre_xy) -> tuple[np.ndarray, tuple[np.ndarray, float]]:
    """The object's visible surface points and the table plane, from all depth points."""
    near = (np.linalg.norm(points_world[:, :2] - np.asarray(roi_centre_xy), axis=1) < ROI_RADIUS) \
        & (points_world[:, 2] < ROI_MAX_Z) & (points_world[:, 2] > -0.1)
    roi = points_world[near]
    if len(roi) < 3:
        raise ValueError("nothing seen in the pick area")
    plane = fit_plane(roi)
    above = roi[roi @ plane[0] + plane[1] > MIN_HEIGHT]
    if len(above) < MIN_POINTS:
        return above, plane
    return above[largest_cluster(above)], plane


# --- step 3: registration -------------------------------------------------------------------------
def icp(scene: np.ndarray, model: ObjectModel, T: np.ndarray, iterations: int = ICP_ITERATIONS,
        max_dist=ICP_MAX_DIST) -> np.ndarray:
    """Refine the body pose T so the scene points lie on the model's surface (point-to-plane, with
    a small point-to-point term for directions a flat patch cannot pin down)."""
    A = invert(T)  # world -> body
    for it in range(iterations):
        md = max_dist[min(it * len(max_dist) // iterations, len(max_dist) - 1)]
        p = transform(A, scene)
        d, j = model.tree.query(p, distance_upper_bound=md)
        ok = np.isfinite(d)
        if ok.sum() < 10:
            break
        p, q, n = p[ok], model.points[j[ok]], model.normals[j[ok]]
        # point-to-plane rows: r = n.(p - q) + (p x n).w + n.v
        J = np.hstack([np.cross(p, n), n])
        r = np.einsum("ij,ij->i", n, p - q)
        H = J.T @ J
        g = J.T @ r
        # point-to-point rows, weight 0.1: r = (p - q) + w x p + v
        w2 = 0.01
        pp = np.einsum("ij,ik->jk", p, p)
        H[:3, :3] += w2 * (np.trace(pp) * np.eye(3) - pp)
        s = p.sum(axis=0)
        S = np.array([[0, -s[2], s[1]], [s[2], 0, -s[0]], [-s[1], s[0], 0]])
        H[:3, 3:] += w2 * S
        H[3:, :3] += w2 * S.T
        H[3:, 3:] += w2 * len(p) * np.eye(3)
        e = p - q
        g[:3] += w2 * np.cross(p, e).sum(axis=0)
        g[3:] += w2 * e.sum(axis=0)
        x = np.linalg.solve(H + 1e-9 * np.eye(6), -g)
        step = np.eye(4)
        step[:3, :3] = rodrigues(x[:3])
        step[:3, 3] = x[3:]
        A = step @ A
        if np.linalg.norm(x[:3]) < 1e-5 and np.linalg.norm(x[3:]) < 1e-6:
            break
    return invert(A)


def free_space_violation(model: ObjectModel, T: np.ndarray, depth: np.ndarray, intr: Intrinsics,
                         T_world_cam: np.ndarray) -> float:
    """Share of the model's points, posed at T, that the camera should have seen in front of what
    it saw: projected into the depth image, they are nearer than the observed depth by more than
    FREE_SPACE_MARGIN. Points behind the observed surface (the object's far side) are fine."""
    pc = transform(invert(T_world_cam) @ T, model.points)
    pc = pc[pc[:, 2] > 0.05]
    u = np.floor(pc[:, 0] / pc[:, 2] * intr.fx + intr.cx).astype(int)
    v = np.floor(pc[:, 1] / pc[:, 2] * intr.fy + intr.cy).astype(int)
    inside = (u >= 0) & (u < intr.width) & (v >= 0) & (v < intr.height)
    d = np.asarray(depth, dtype=float).reshape(intr.height, intr.width)[v[inside], u[inside]]
    z = pc[inside, 2]
    seen = np.isfinite(d) & (d > 0)
    if not seen.any():
        return 1.0
    return float(np.count_nonzero(seen & (d > z + FREE_SPACE_MARGIN)) / np.count_nonzero(seen))


def explain(scene: np.ndarray, model: ObjectModel, T: np.ndarray) -> tuple[float, float]:
    """Share of scene points within INLIER_DIST of the posed model, and their RMS distance."""
    d, _ = model.tree.query(transform(invert(T), scene))
    inl = d < INLIER_DIST
    return float(inl.mean()), float(np.sqrt(np.mean(d[inl] ** 2))) if inl.any() else float("nan")


def initial_poses(scene: np.ndarray, model: ObjectModel, cam_pos: np.ndarray, rotations: np.ndarray,
                  plane: tuple[np.ndarray, float], chunk: int = 250) -> tuple[np.ndarray, np.ndarray]:
    """A starting pose for each rotation, and a coarse score for it (lower is better).

    Translation: across the table, the centroid of the model's camera-facing points goes onto the
    scene's centroid; up from the table, the model's lowest point goes onto the table (the object
    rests on it). Score: mean distance (capped at 1 cm) from the scene points to the model so
    placed. Returns (poses (n, 4, 4), scores (n,))."""
    n, d = plane
    centroid = scene.mean(axis=0)
    poses = np.tile(np.eye(4), (len(rotations), 1, 1))
    scores = np.empty(len(rotations))
    for a in range(0, len(rotations), chunk):
        R = rotations[a:a + chunk]
        pw = np.einsum("rij,nj->rni", R, model.points)  # rotated model points
        nw = np.einsum("rij,nj->rni", R, model.normals)
        t = centroid - np.einsum("rij,j->ri", R, model.centre)
        for _ in range(2):
            facing = np.einsum("rni,rni->rn", nw, cam_pos - (pw + t[:, None, :])) > 0
            cnt = np.maximum(facing.sum(axis=1), 1)
            t = centroid - (pw * facing[..., None]).sum(axis=1) / cnt[:, None]
        lowest = (pw @ n).min(axis=1) + t @ n + d  # height of the lowest model point above the table
        t -= lowest[:, None] * n
        poses[a:a + chunk, :3, :3] = R
        poses[a:a + chunk, :3, 3] = t
        body = np.einsum("rji,rnj->rni", R, scene[None, :, :] - t[:, None, :])  # scene in each body frame
        dist, _ = model.tree.query(body.reshape(-1, 3), distance_upper_bound=0.01, workers=-1)
        scores[a:a + chunk] = np.minimum(dist, 0.01).reshape(len(R), -1).mean(axis=1)
    return poses, scores


def subsample(points: np.ndarray, n: int, seed: int = 0) -> np.ndarray:
    return points if len(points) <= n else points[np.random.default_rng(seed).choice(len(points), n, replace=False)]


def score_pose(T: np.ndarray, scene: np.ndarray, model: ObjectModel, depth: np.ndarray, intr: Intrinsics,
               T_world_cam: np.ndarray, plane: tuple[np.ndarray, float]) -> PoseEstimate:
    """How well the model posed at T accounts for the image: the share of observed points it
    explains, minus the share of its points in seen-free space and sunk into the table."""
    fitness, rmse = explain(scene, model, T)
    violation = free_space_violation(model, T, depth, intr, T_world_cam)
    sunk = float(np.mean(transform(T, model.points) @ plane[0] + plane[1] < -TABLE_MARGIN))
    return PoseEstimate(T, fitness - violation - sunk, fitness, rmse, violation, sunk)


def register(object_points: np.ndarray, model: ObjectModel, depth: np.ndarray, intr: Intrinsics,
             T_world_cam: np.ndarray, plane: tuple[np.ndarray, float]) -> PoseEstimate:
    """Search over orientations in three rounds, each on fewer, better candidates: a coarse rank of
    N_ROTATIONS starting poses on a few points; a short ICP and the full score for the best REFINE;
    the full ICP for the best TOP_K of those."""
    scene = voxel_downsample(object_points, SCENE_VOXEL)
    if len(scene) < MIN_POINTS // 3:
        return PoseEstimate(None, n_points=len(object_points), detail=f"only {len(object_points)} object points")
    poses, coarse = initial_poses(subsample(scene, COARSE_POINTS), model, T_world_cam[:3, 3],
                                  rotation_grid(N_ROTATIONS), plane)
    few = subsample(scene, REFINE_POINTS)
    refined = [score_pose(icp(few, model, poses[k], REFINE_ITERATIONS, ICP_MAX_DIST[:2]), few, model, depth, intr,
                          T_world_cam, plane) for k in np.argsort(coarse)[:REFINE]]
    best = PoseEstimate(None, score=-np.inf)
    for cand in sorted(refined, key=lambda e: -e.score)[:TOP_K]:
        assert cand.pose is not None
        est = score_pose(icp(scene, model, cand.pose), scene, model, depth, intr, T_world_cam, plane)
        if est.score > best.score:
            best = est
    best.n_points = len(object_points)
    return best


def locate(depth: np.ndarray, intr: Intrinsics, T_world_cam: np.ndarray, model: ObjectModel,
           roi_centre_xy) -> PoseEstimate:
    """The object's pose from a depth image taken from T_world_cam (OpenCV camera frame)."""
    pts = transform(T_world_cam, backproject(depth, intr))
    try:
        obj, plane = segment_object(pts, roi_centre_xy)
    except ValueError as e:
        return PoseEstimate(None, detail=str(e))
    if len(obj) < MIN_POINTS:
        return PoseEstimate(None, n_points=len(obj), detail=f"only {len(obj)} object points")
    return register(obj, model, depth, intr, T_world_cam, plane)


# --- evaluation (needs the true pose) -------------------------------------------------------------
def pose_errors(model: ObjectModel, T_est: np.ndarray, T_true: np.ndarray) -> dict:
    """centre_mm: distance between the model's centroid as estimated and as true; rot_deg; add_mm:
    mean distance of corresponding model points; adds_mm: mean distance of each estimated model
    point to the nearest true one (symmetric objects: an equivalent pose scores near zero)."""
    pe, pt = transform(T_est, model.points), transform(T_true, model.points)
    return {
        "centre_mm": 1000 * float(np.linalg.norm(T_est[:3, :3] @ model.centre + T_est[:3, 3]
                                                 - T_true[:3, :3] @ model.centre - T_true[:3, 3])),
        "rot_deg": angle_deg(T_est, T_true),
        "add_mm": 1000 * float(np.mean(np.linalg.norm(pe - pt, axis=1))),
        "adds_mm": 1000 * float(np.mean(KDTree(pt).query(pe)[0])),
    }
