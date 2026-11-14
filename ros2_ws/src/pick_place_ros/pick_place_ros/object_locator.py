"""object_locator: finds the object in the wrist camera's depth image (step 5 of omniverse_hookup).

The same perception as step 4 (pick_place/perception.py), as a ROS 2 node. Isaac Sim plays the
robot and the camera; this node only sees what a real camera and robot would give it:

  subscribes  /wrist_camera/depth        sensor_msgs/Image, 32FC1 metres (NaN: no depth)
              /wrist_camera/camera_info  sensor_msgs/CameraInfo
              /tf, /tf_static            the arm's links (from Isaac Sim) and the camera mount
                                         (static, from the hand-eye calibration)
              /cell/object               std_msgs/String, latched: which object to look for
  publishes   /object_pose               geometry_msgs/PoseStamped, in the world frame, stamped
                                         with the image's stamp
              /object_pose/info          std_msgs/String, JSON: found, score, fitness, points,
                                         detail, compute time; one per image, found or not
              /object_pose/model         sensor_msgs/PointCloud2: the object's model at the pose
              /wrist_camera/object_points  sensor_msgs/PointCloud2: the points taken as the object

TF is kept in this node's own buffer, fed from /tf and /tf_static, rather than a
TransformListener: the cell restarts the simulation, and with it the clock, every cycle, and a
plain buffer would then ignore every new transform as older than the last cycle's. When time
jumps back, the buffer is cleared (the static mount is put back).
"""

import json
import pathlib
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def matrix_from_transform(t) -> np.ndarray:
    q, p = t.rotation, t.translation
    w, x, y, z = q.w, q.x, q.y, q.z
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def quaternion(R: np.ndarray) -> tuple[float, float, float, float]:
    """Rotation matrix -> (w, x, y, z)."""
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    q = np.array([w, x, y, z])
    q /= np.linalg.norm(q)
    return tuple(float(v) for v in q)


def yaml_frames(buffer) -> dict:
    """The TF frames the buffer knows: {child: {'parent': ...}}."""
    import yaml

    return yaml.safe_load(buffer.all_frames_as_yaml()) or {}


def cloud(points: np.ndarray, header) -> PointCloud2:
    msg = PointCloud2()
    msg.header = header
    msg.height, msg.width = 1, len(points)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")]
    msg.is_bigendian, msg.point_step, msg.row_step, msg.is_dense = False, 12, 12 * len(points), True
    msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
    return msg


class ObjectLocator(Node):
    def __init__(self):
        super().__init__("object_locator")
        self.declare_parameter("project_dir", "")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("roi_centre", [0.75, -0.35])
        project = pathlib.Path(self.get_parameter("project_dir").value)
        if not (project / "pick_place" / "perception.py").exists():
            raise RuntimeError(f"project_dir {project!s} has no pick_place/perception.py")
        sys.path.insert(0, str(project / "pick_place"))
        import perception  # the step 4 perception, unchanged

        self.P = perception
        self.models_dir = project / "results" / "perception" / "models"
        self.models: dict = {}
        self.world = self.get_parameter("world_frame").value
        self.roi = tuple(self.get_parameter("roi_centre").value)
        self.object = None
        self.camera_info = None

        self.buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30))
        self.static_tf: list = []
        self.newest_tf = None
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        self.create_subscription(TFMessage, "/tf_static", self.on_tf_static, LATCHED)
        self.create_subscription(String, "/cell/object", self.on_object, LATCHED)
        self.create_subscription(CameraInfo, "/wrist_camera/camera_info", self.on_camera_info, 5)
        self.create_subscription(Image, "/wrist_camera/depth", self.on_depth, 2)
        self.pub_pose = self.create_publisher(PoseStamped, "/object_pose", 10)
        self.pub_info = self.create_publisher(String, "/object_pose/info", 10)
        self.pub_model = self.create_publisher(PointCloud2, "/object_pose/model", 2)
        self.pub_points = self.create_publisher(PointCloud2, "/wrist_camera/object_points", 2)
        self.get_logger().info(f"waiting for depth images; perception from {project / 'pick_place'}")

    # --- inputs ---------------------------------------------------------------------------
    def on_tf(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            stamp = Time.from_msg(t.header.stamp).nanoseconds * 1e-9
            if self.newest_tf is not None and stamp < self.newest_tf - 0.5:
                self.get_logger().info(f"time jumped back ({self.newest_tf:.2f} s -> {stamp:.2f} s): TF buffer cleared")
                self.buffer.clear()
                for s in self.static_tf:
                    self.buffer.set_transform_static(s, "static")
                self.newest_tf = None
            self.newest_tf = stamp if self.newest_tf is None else max(self.newest_tf, stamp)
            self.buffer.set_transform(t, "isaac_sim")

    def on_tf_static(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            self.static_tf.append(t)
            self.buffer.set_transform_static(t, "static")

    def on_object(self, msg: String) -> None:
        self.object = msg.data
        self.get_logger().info(f"looking for {self.object}")

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    # --- the work -------------------------------------------------------------------------
    def model(self, name: str):
        if name not in self.models:
            m = np.load(self.models_dir / f"{name}.npz")
            self.models[name] = self.P.ObjectModel(m["points"], m["normals"])
        return self.models[name]

    def on_depth(self, msg: Image) -> None:
        t0 = time.perf_counter()
        info = {"stamp": [msg.header.stamp.sec, msg.header.stamp.nanosec], "found": False}
        try:
            if self.object is None:
                raise LookupError("no object named on /cell/object yet")
            if self.camera_info is None:
                raise LookupError("no camera_info yet")
            if msg.encoding != "32FC1":
                raise LookupError(f"depth encoding {msg.encoding}, expected 32FC1")
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).astype(float)
            k = self.camera_info.k
            # ROS puts pixel centres at integer coordinates; perception.py at half-integers
            intr = self.P.Intrinsics(k[0], k[4], k[2] + 0.5, k[5] + 0.5, msg.width, msg.height)
            # The arm stands still while the camera looks, so the newest transform will do; how
            # old it is against the image goes into the info.
            try:
                tf = self.buffer.lookup_transform(self.world, msg.header.frame_id, Time())
            except Exception as e:  # noqa: BLE001  tf2's lookup, connectivity, extrapolation errors
                frames = sorted(f"{v['parent']}->{k}" for k, v in yaml_frames(self.buffer).items())
                raise LookupError(f"{e}; frames known: {', '.join(frames) or 'none'}") from None
            info["tf_age_s"] = round((Time.from_msg(msg.header.stamp) - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9, 4)
            T_wc = matrix_from_transform(tf.transform)
            est = self.P.locate(depth, intr, T_wc, self.model(self.object), self.roi)
            info.update(found=est.pose is not None, points=int(est.n_points), detail=est.detail,
                        score=float(est.score), fitness=float(est.fitness), violation=float(est.violation))
            if est.pose is not None:
                pose = PoseStamped()
                pose.header.stamp, pose.header.frame_id = msg.header.stamp, self.world
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (float(v) for v in est.pose[:3, 3])
                w, x, y, z = quaternion(est.pose[:3, :3])
                pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z = w, x, y, z
                self.pub_pose.publish(pose)
                model_points = self.P.transform(est.pose, self.model(self.object).points)
                self.pub_model.publish(cloud(model_points, pose.header))
            pts = self.P.transform(T_wc, self.P.backproject(depth, intr))
            obj, _ = self.P.segment_object(pts, self.roi)
            header = PoseStamped().header
            header.stamp, header.frame_id = msg.header.stamp, self.world
            self.pub_points.publish(cloud(obj, header))
        except Exception as e:  # noqa: BLE001  - any failure is reported as not found
            info["detail"] = f"{type(e).__name__}: {e}"
        info["wall_s"] = round(time.perf_counter() - t0, 3)
        self.pub_info.publish(String(data=json.dumps(info)))
        self.get_logger().info(f"{self.object}: " + (f"found from {info.get('points')} points in {info['wall_s']:.1f} s"
                                                     if info["found"] else f"not found: {info.get('detail')}"))


def main() -> None:
    rclpy.init()
    node = ObjectLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
