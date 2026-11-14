"""The wrist camera in the stage, and the object's model from its mesh. Runs inside Isaac Sim;
import it after SimulationApp has started. The camera's numbers (optics, mount, depth) are in
camera_model.py.
"""

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

import camera_model as cm
import cell
import perception

CAMERA = f"{cell.WRIST}/WristCamera"
HOUSING = f"{cell.WRIST}/WristCameraHousing"


def to_gf(m: np.ndarray) -> Gf.Matrix4d:
    return Gf.Matrix4d(m.T.tolist())


def to_np(m: Gf.Matrix4d) -> np.ndarray:
    return np.array(m, dtype=float).T


class WristCamera:
    """The camera prim on the wrist (at camera_model.mount()), its housing (a plain box of the
    D455's size, visual only), and a render product giving RGB and the renderer's depth (distance
    to the image plane).

    The images come through plain Replicator annotators on the CPU. With Isaac Sim's CameraSensor
    wrapper attached instead (even if never read), creating the second cuMotion optimizer crashed
    Isaac Sim in 3 of 3 runs; with these, 6 of 6 plans went through."""

    def __init__(self, stage: Usd.Stage):
        import omni.replicator.core as rep
        from isaacsim.sensors.experimental.rtx import RtxCamera

        RtxCamera(CAMERA)  # the camera prim, with Omniverse's sensor schema
        prim = stage.GetPrimAtPath(CAMERA)
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(to_gf(cm.mount()))
        usd_cam = UsdGeom.Camera(prim)
        usd_cam.CreateFocalLengthAttr(cm.FOCAL_MM)
        usd_cam.CreateHorizontalApertureAttr(cm.H_APERTURE_MM)
        usd_cam.CreateVerticalApertureAttr(cm.H_APERTURE_MM * cm.HEIGHT / cm.WIDTH)  # square pixels
        usd_cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 10.0))

        housing = UsdGeom.Cube.Define(stage, HOUSING)  # no collision
        housing.CreateSizeAttr(1.0)
        box = cm.mount()
        box[:3, 3] = (box @ np.array([0.0, 0.0, cm.HOUSING_SIZE[2] / 2, 1.0]))[:3]  # behind the lens
        housing.AddTransformOp().Set(to_gf(box @ np.diag([*cm.HOUSING_SIZE, 1.0])))
        housing.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.12, 0.14)])

        self.render_product = rep.create.render_product(CAMERA, (cm.WIDTH, cm.HEIGHT))
        # Render at the sensor's own resolution: no DLSS upscaling from a lower one.
        rp = stage.GetPrimAtPath(self.render_product.path)
        if "OmniRtxPostDebugSettingsAPI_1" not in rp.GetAppliedSchemas():
            rp.ApplyAPI("OmniRtxPostDebugSettingsAPI_1")
        rp.GetAttribute("omni:rtx:post:aa:op").Set("none")
        self.annotators = {}
        for name in ("rgb", "distance_to_image_plane"):
            annotator = rep.AnnotatorRegistry.get_annotator(name, device="cpu")
            annotator.attach(self.render_product)
            self.annotators[name] = annotator

    def read(self) -> tuple[np.ndarray, np.ndarray] | None:
        """(RGB (H, W, 3) uint8, perfect depth (H, W) metres), or None before the first frame."""
        rgb = np.asarray(self.annotators["rgb"].get_data())
        depth = np.asarray(self.annotators["distance_to_image_plane"].get_data())
        if rgb.size != cm.HEIGHT * cm.WIDTH * 4 or depth.size != cm.HEIGHT * cm.WIDTH:
            return None
        return (rgb.reshape(cm.HEIGHT, cm.WIDTH, 4)[..., :3].astype(np.uint8),
                depth.astype(float).reshape(cm.HEIGHT, cm.WIDTH))


def fan_triangles(counts: np.ndarray, indices: np.ndarray) -> np.ndarray:
    """USD polygon faces -> triangles (fans)."""
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
    out = []
    for c in np.unique(counts):
        if c < 3:
            continue
        s = starts[counts == c]
        out += [np.stack([indices[s], indices[s + k], indices[s + k + 1]], 1) for k in range(1, c - 1)]
    return np.vstack(out) if out else np.zeros((0, 3), dtype=int)


def object_model(body: Usd.Prim) -> tuple[perception.ObjectModel, list[str]]:
    """Points sampled from the object's visual meshes (not its colliders), in its body frame, and
    the meshes used."""
    xc = UsdGeom.XformCache()
    body_world = to_np(xc.GetLocalToWorldTransform(body))
    if not np.allclose(np.linalg.norm(body_world[:3, :3], axis=0), 1.0, atol=1e-6):
        raise RuntimeError(f"{body.GetPath()} is scaled; its physics pose would not match its mesh")
    to_body = perception.invert(body_world)
    verts, tris, used, offset = [], [], [], 0
    for p in Usd.PrimRange(body):
        if not p.IsA(UsdGeom.Mesh) or p.HasAPI(UsdPhysics.CollisionAPI) or "collider" in str(p.GetPath()).lower():
            continue
        im = UsdGeom.Imageable(p)
        if im.ComputePurpose() not in (UsdGeom.Tokens.default_, UsdGeom.Tokens.render) \
                or im.ComputeVisibility() == UsdGeom.Tokens.invisible:
            continue
        mesh = UsdGeom.Mesh(p)
        pts = mesh.GetPointsAttr().Get()
        if not pts:
            continue
        pts = np.asarray(pts, dtype=float)
        t = fan_triangles(np.asarray(mesh.GetFaceVertexCountsAttr().Get(), dtype=int),
                          np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=int))
        verts.append(perception.transform(to_body @ to_np(xc.GetLocalToWorldTransform(p)), pts))
        tris.append(t + offset)
        offset += len(pts)
        used.append(f"{p.GetPath()} ({len(pts)} points, {len(t)} triangles)")
    if not verts:
        raise RuntimeError(f"no visual mesh under {body.GetPath()}")
    return perception.ObjectModel.from_mesh(np.vstack(verts), np.vstack(tris)), used
