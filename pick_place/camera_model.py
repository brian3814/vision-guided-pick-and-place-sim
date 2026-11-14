"""The wrist camera's model: optics, where it sits on the gripper, and how its depth is made.
Pure numpy (no Isaac Sim), so offline tools use the same numbers; wrist_camera.py builds it in the
stage.

The camera is modelled on an Intel RealSense D455: the optics of Isaac Sim's D455 asset (1.93 mm
focal length on a 3.896 mm wide sensor, a 90 x 64 degree field of view) at 640 x 400 pixels, and
its 95 mm stereo baseline for the depth.

Frames: the camera prim is a USD camera (looks down its -z, y up). Perception uses OpenCV's camera
frame (x right, y down, z forward); camera_pose() returns that one.

Depth (simulate_depth): an active-stereo camera applied to the renderer's perfect depth. Why not
Isaac Sim's own stereo model (SingleViewDepthCameraSensor): it models passive stereo,
which needs surface texture. On the cell's plain table and smooth objects it returned depth for
1 to 2% of the pixels (texture edges only). A D455 projects an infrared dot pattern and measures
such surfaces densely. So the depth here is perfect depth with the effects of stereo matching
added:

- Disparity noise. Depth comes from disparity d = f B / z, so a disparity error of s pixels is a
  depth error of z^2 s / (f B): it grows with the square of the distance. The noise is a white part
  plus a part correlated over a block of pixels (a stereo matcher's window).
- Stereo occlusion. Next to the left edge of a nearer surface, the background is seen by the left
  imager (the depth reference) but hidden from the right one: no depth there.
- Edges. Matching windows that straddle a depth step give unreliable depth; the camera's filters
  drop it. Pixels whose 3 x 3 neighbourhood spans more than EDGE_STEP_M are left empty.
- Range. Disparities above MAX_DISPARITY_PX (too near) are not searched.

The noise levels are assumptions, not a calibration: about 1 mm at 0.45 m with the defaults.
"""

import warnings

import numpy as np

import perception

WIDTH, HEIGHT = 640, 400
FOCAL_MM, H_APERTURE_MM = 1.93, 3.896
# In the gripper (= wrist_3_link = tool0) frame: 2.5 cm outside the 2F-140's body on its +x side
# (its fingers close along y), 3 cm out from the flange.
MOUNT_XYZ = (0.0625, 0.0, 0.03)
HOUSING_SIZE = (0.124, 0.029, 0.026)  # along the camera's x (the stereo baseline), y, and depth
USD_TO_CV = np.diag([1.0, -1.0, -1.0, 1.0])
# Where the camera looks from before a pick: this far from the pick spot towards the robot's base,
# this high above the table, so it sees the object's top and the side facing the robot.
LOOK_BACK_M, LOOK_HEIGHT_M = 0.22, 0.45


def mount() -> np.ndarray:
    """The USD camera's pose in the gripper frame, as built: looking along +z (the approach axis),
    its x axis (the stereo baseline) along the gripper's y (the closing axis)."""
    m = np.eye(4)
    m[:3, :3] = np.column_stack([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    m[:3, 3] = MOUNT_XYZ
    return m


def intrinsics() -> perception.Intrinsics:
    f = FOCAL_MM / H_APERTURE_MM * WIDTH
    return perception.Intrinsics(f, f, WIDTH / 2, HEIGHT / 2, WIDTH, HEIGHT)


def camera_pose(tool_pose: np.ndarray, mount_pose: np.ndarray | None = None) -> np.ndarray:
    """The camera's pose (OpenCV frame) in the world, for the gripper frame at tool_pose; mount_pose
    is the USD camera in the gripper frame (default: as built; or a hand-eye calibration's)."""
    return tool_pose @ (mount() if mount_pose is None else mount_pose) @ USD_TO_CV


def look_tool_pose(target, back: float = LOOK_BACK_M, height: float = LOOK_HEIGHT_M) -> np.ndarray:
    """The gripper pose that puts the camera `back` m from the target, towards the robot's base,
    and `height` m above the table, looking at the target."""
    target = np.asarray(target, dtype=float)
    toward_base = -np.array([target[0], target[1], 0.0])
    toward_base /= np.linalg.norm(toward_base)
    cam = np.array([target[0], target[1], 0.0]) + back * toward_base
    cam[2] = height
    z = cam - target  # a USD camera's +z points back from what it looks at
    z /= np.linalg.norm(z)
    x = np.cross([0.0, 0.0, 1.0], z)
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    T[:3, 3] = cam
    return T @ np.linalg.inv(mount())


def tool_spheres() -> list[tuple[np.ndarray, float]]:
    """Spheres covering the housing, in the gripper frame, for the planner."""
    m = mount()
    centre = (m @ np.array([0.0, 0.0, HOUSING_SIZE[2] / 2, 1.0]))[:3]  # the box sits behind the lens
    r = 0.5 * float(np.hypot(HOUSING_SIZE[1], HOUSING_SIZE[2]))
    half = HOUSING_SIZE[0] / 2 - r
    return [(centre + o * m[:3, 0], r) for o in np.linspace(-half, half, 5)]


# --- depth --------------------------------------------------------------------------------------
BASELINE_M = 0.095
WHITE_PX = 0.05        # per-pixel disparity noise, pixels (standard deviation)
CORRELATED_PX = 0.15   # disparity noise shared over blocks of BLOCK_PX pixels
BLOCK_PX = 8
EDGE_STEP_M = 0.02
MAX_DISPARITY_PX = 126.0
DROPOUT = 0.005        # share of pixels lost at random


def simulate_depth(depth: np.ndarray, fx: float, rng: np.random.Generator, scale: float = 1.0,
             baseline_m: float = BASELINE_M) -> np.ndarray:
    """Perfect depth (H, W), metres to the image plane -> depth as the stereo camera would give it
    (NaN where it has none). scale multiplies both noise levels."""
    z = np.asarray(depth, dtype=float).copy()
    h, w = z.shape
    valid = np.isfinite(z) & (z > 0)
    fb = fx * baseline_m
    disparity = np.where(valid, fb / np.where(valid, z, 1.0), 0.0)

    # stereo occlusion: pixel u is hidden from the right imager if a pixel to its right lands at
    # or left of it there (u' - d(u') <= u - d(u))
    right = np.arange(w)[None, :] - disparity
    right[~valid] = np.inf
    nearest_from_right = np.minimum.accumulate(right[:, ::-1], axis=1)[:, ::-1]
    shifted = np.full_like(right, np.inf)
    shifted[:, :-1] = nearest_from_right[:, 1:]
    occluded = shifted < right - 0.5

    # edges
    padded = np.pad(np.where(valid, z, np.nan), 1, mode="edge")
    windows = np.stack([padded[i:i + h, j:j + w] for i in range(3) for j in range(3)])
    with warnings.catch_warnings():  # all-NaN windows (no depth around): not an edge, dropped anyway
        warnings.simplefilter("ignore", RuntimeWarning)
        step = np.nanmax(windows, axis=0) - np.nanmin(windows, axis=0)
    edge = ~(step <= EDGE_STEP_M)

    # disparity noise
    bh, bw = -(-h // BLOCK_PX), -(-w // BLOCK_PX)
    blocks = rng.normal(0.0, CORRELATED_PX * scale, (bh, bw))
    correlated = np.kron(blocks, np.ones((BLOCK_PX, BLOCK_PX)))[:h, :w]
    noisy = disparity + correlated + rng.normal(0.0, WHITE_PX * scale, (h, w))

    keep = valid & ~occluded & ~edge & (disparity <= MAX_DISPARITY_PX) & (noisy > 0) & (rng.random((h, w)) >= DROPOUT)
    out = np.full((h, w), np.nan)
    out[keep] = fb / noisy[keep]
    return out
