"""ROS 2 for the cell (step 5). Runs inside Isaac Sim; import it after SimulationApp has started.

Isaac Sim plays the robot: through its ROS 2 bridge (OmniGraph) it publishes the simulation clock,
the arm and gripper's joint states and their TF tree every physics step, and when the cycle looks,
the wrist camera's depth, colour and intrinsics. The object's pose comes back from the
object_locator node (ros2_ws/src/pick_place_ros), which finds it in that depth image with the
camera's pose from TF.

    topics (Isaac Sim publishes)            topics (the locator publishes)
    /clock          rosgraph_msgs/Clock      /object_pose        geometry_msgs/PoseStamped
    /joint_states   sensor_msgs/JointState   /object_pose/info   std_msgs/String (JSON: found,
    /tf             tf2_msgs/TFMessage                            fitness, points, detail, stamp)
    /wrist_camera/depth        sensor_msgs/Image (32FC1, metres, NaN where the camera has no depth)
    /wrist_camera/color        sensor_msgs/Image (rgb8)
    /wrist_camera/camera_info  sensor_msgs/CameraInfo
    /debug/object_true_pose    geometry_msgs/PoseStamped (the simulation's truth, for comparison only)

Launch Isaac Sim with RMW_IMPLEMENTATION=rmw_fastrtps_cpp in the environment: its default on
Windows is Zenoh, and ROS 2 Jazzy for Windows talks Fast DDS. ROS_DOMAIN_ID must match (default 0).
"""

import json
import time

import numpy as np

import camera_model
import perception

WORLD_FRAME = "world"
CAMERA_FRAME = "wrist_camera_optical"
GRAPH = "/ROS2Bridge"


def articulation_root(robot_prim: str) -> str:
    """The prim under robot_prim carrying the articulation root. The TF and joint-state publishers
    must target it: given the robot's outer Xform they publish only that one frame."""
    import omni.usd
    from pxr import Usd, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    for p in Usd.PrimRange(stage.GetPrimAtPath(robot_prim)):
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(p.GetPath())
    raise RuntimeError(f"no articulation root under {robot_prim}")


def build_graph(robot_prim: str) -> str:
    """The bridge's action graph: clock, joint states and the robot's TF tree, every tick.
    Returns the articulation root the publishers target."""
    import omni.graph.core as og
    import usdrt.Sdf

    root = articulation_root(robot_prim)
    og.Controller.edit(
        {"graph_path": GRAPH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("JointStates", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("TF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("JointStates.inputs:targetPrim", [usdrt.Sdf.Path(root)]),
                ("TF.inputs:targetPrims", [usdrt.Sdf.Path(root)]),
            ],
            og.Controller.Keys.CONNECT: [
                ("Tick.outputs:tick", "Clock.inputs:execIn"),
                ("Tick.outputs:tick", "JointStates.inputs:execIn"),
                ("Tick.outputs:tick", "TF.inputs:execIn"),
                ("SimTime.outputs:simulationTime", "Clock.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "JointStates.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "TF.inputs:timeStamp"),
            ],
        },
    )
    return root


def _stamp(t: float):
    from builtin_interfaces.msg import Time

    sec = int(np.floor(t))
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


def _pose_msg(T: np.ndarray, stamp, frame: str):
    from geometry_msgs.msg import PoseStamped

    q = _quat(T[:3, :3])
    msg = PoseStamped()
    msg.header.stamp, msg.header.frame_id = stamp, frame
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in T[:3, 3])
    msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = (float(v) for v in q)
    return msg


def _quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> unit quaternion (w, x, y, z)."""
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def pose_from_msg(p) -> np.ndarray:
    w, x, y, z = p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


class RosLink:
    """Isaac Sim's own ROS 2 node (the bridge's bundled rclpy): publishes the camera when asked
    and waits for the locator's answer."""

    def __init__(self, timeout_s: float = 60.0):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import String

        self.timeout_s = timeout_s
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("isaac_cell", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.pub_depth = self.node.create_publisher(Image, "/wrist_camera/depth", 2)
        self.pub_color = self.node.create_publisher(Image, "/wrist_camera/color", 2)
        self.pub_info = self.node.create_publisher(CameraInfo, "/wrist_camera/camera_info", 2)
        self.pub_truth = self.node.create_publisher(PoseStamped, "/debug/object_true_pose", 2)
        self.poses: dict[tuple, np.ndarray] = {}
        self.infos: dict[tuple, dict] = {}
        self.node.create_subscription(PoseStamped, "/object_pose", self._on_pose, 10)
        self.node.create_subscription(String, "/object_pose/info", self._on_info, 10)
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_object = self.node.create_publisher(String, "/cell/object", latched)
        self.Image, self.CameraInfo, self.String = Image, CameraInfo, String

    def publish_object(self, name: str) -> None:
        """Tell the locator which object to look for (latched: a locator started later gets it too)."""
        self.pub_object.publish(self.String(data=name))

    def wait_for_locator(self, timeout_s: float = 60.0) -> bool:
        t0 = time.perf_counter()
        while not self.locator_seen():
            if time.perf_counter() - t0 > timeout_s:
                return False
            self.spin(0.2)
        self.spin(1.0)  # let it receive /cell/object and the TF tree
        return True

    def _on_pose(self, msg) -> None:
        self.poses[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = pose_from_msg(msg.pose)

    def _on_info(self, msg) -> None:
        info = json.loads(msg.data)
        self.infos[(info["stamp"][0], info["stamp"][1])] = info

    def spin(self, seconds: float = 0.0) -> None:
        end = time.perf_counter() + seconds
        while True:
            self.executor.spin_once(timeout_sec=0.01)
            if time.perf_counter() >= end:
                return

    def locator_seen(self) -> bool:
        """Whether an object_locator node is subscribed to the depth topic."""
        return self.pub_depth.get_subscription_count() > 0

    def publish_camera(self, rgb: np.ndarray, depth: np.ndarray, intr: perception.Intrinsics, sim_time: float):
        stamp = _stamp(sim_time)
        info = self.CameraInfo()
        info.header.stamp, info.header.frame_id = stamp, CAMERA_FRAME
        info.width, info.height = intr.width, intr.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        # ROS puts pixel centres at integer coordinates; camera_model's principal point is in
        # continuous coordinates (pixel u spans [u, u + 1)).
        cx, cy = intr.cx - 0.5, intr.cy - 0.5
        info.k = [intr.fx, 0.0, cx, 0.0, intr.fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [intr.fx, 0.0, cx, 0.0, 0.0, intr.fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(info)
        for pub, data, encoding, step in ((self.pub_color, rgb.astype(np.uint8), "rgb8", intr.width * 3),
                                          (self.pub_depth, depth.astype(np.float32), "32FC1", intr.width * 4)):
            img = self.Image()
            img.header.stamp, img.header.frame_id = stamp, CAMERA_FRAME
            img.height, img.width, img.encoding, img.is_bigendian, img.step = intr.height, intr.width, encoding, 0, step
            img.data = np.ascontiguousarray(data).tobytes()
            pub.publish(img)
        return stamp

    def locate(self, rgb: np.ndarray, depth: np.ndarray, sim_time: float, true_pose: np.ndarray | None = None
               ) -> tuple[perception.PoseEstimate, float]:
        """Publish the image and wait for the locator's answer to it. Returns the estimate (pose
        None if not found or no answer in time) and the wall time waited."""
        stamp = self.publish_camera(rgb, depth, camera_model.intrinsics(), sim_time)
        if true_pose is not None:
            self.pub_truth.publish(_pose_msg(true_pose, stamp, WORLD_FRAME))
        key = (stamp.sec, stamp.nanosec)
        t0 = time.perf_counter()
        while key not in self.infos and time.perf_counter() - t0 < self.timeout_s:
            self.executor.spin_once(timeout_sec=0.02)
        waited = time.perf_counter() - t0
        info = self.infos.pop(key, None)
        if info is None:
            return perception.PoseEstimate(None, detail=f"no answer from object_locator in {self.timeout_s:.0f} s"), waited
        if info["found"]:
            while key not in self.poses and time.perf_counter() - t0 < self.timeout_s:
                self.executor.spin_once(timeout_sec=0.02)
        pose = self.poses.pop(key, None) if info["found"] else None
        return perception.PoseEstimate(pose, score=info.get("score", float("nan")), fitness=info.get("fitness", float("nan")),
                                       violation=info.get("violation", float("nan")), n_points=info.get("points", 0),
                                       detail=info.get("detail", "")), waited

    def close(self) -> None:
        self.executor.shutdown()
        self.node.destroy_node()
        import rclpy

        rclpy.shutdown(context=self.context)
