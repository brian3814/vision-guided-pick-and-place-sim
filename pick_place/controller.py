"""One pick-and-place cycle as a phase machine, advanced once per physics step. Runs inside Isaac
Sim; import it only after SimulationApp has started.

While it waits for the planning process, the cycle yields PLANNING instead of a step: its driver
(cell_session.run_cycle) then keeps Kit updating with the simulation paused, so the viewport and the
web page's stream go on, and the arm is still where the plan starts when the answer comes.

Phases: settle -> [look] -> to pre-grasp -> approach -> close -> lift -> transport -> lower ->
release -> retract -> home -> settle. Every phase has a measured gate; a failed gate ends the cycle
with an outcome naming it, and the rest of the cycle is not run (no release on a timeout).

Where the object is: by default the simulation says (its true pose, as in step 3). With a Vision
(step 4), the arm first moves to a look pose and the wrist camera's depth image gives the pose
(perception.py); grasps, the planner's obstacle and the place pose then come from that estimate
alone. The true pose is still read, but only to judge the gates and score the result.
Blind (an `assumed` pose, the baseline for the camera): no camera and no true pose; the robot takes
the object to be where it would be without the start pose's turn and shift, and grasps, the
obstacle and the place pose come from that.

Outcomes:
  placed             the object came to rest at the place spot: within PLACE_TOLERANCE_M of where
                     it should be, tilted less than MAX_TILT_DEG
  misplaced          it was released but came to rest elsewhere or tipped over
  not_found          the camera found no object (or gave no image)
  unreachable        no robust grasp could be reached and its whole cycle planned
  plan_failed        the motion to the look pose or home could not be planned
  tracking           the arm did not reach a planned pose within HOLD_MAX_STEPS (detail: phase)
  missed             the fingers closed to where the pads meet: nothing between them
  dropped_lift       the object did not rise with the gripper
  dropped_transport  the object left the gripper while it was carried
"""

import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import omni.physx

import camera_model
import cell
import gm_config
import perception
import planner as planner_mod

PRE_GRASP_M = 0.10        # approach and retreat this far along the gripper's approach axis
LIFT_M = 0.15             # lift and carry height above the pick and place poses
PLACE_CLEARANCE_M = 0.003  # release the object this far above where it rests
SETTLE_STEPS = 30
HOLD_MAX_STEPS = 90       # after a trajectory ends, steps allowed to reach its final pose
TOOL_TOLERANCE_M = 0.003
TOOL_TOLERANCE_DEG = 1.0
MAX_TRACKING_ERROR_M = 0.02  # stop if the tool falls this far behind the planned path
SPEED = 0.5                # trajectories are played at this fraction of the optimizer's timing
DEBUG_STEPS = int(__import__("os").environ.get("CELL_DEBUG_STEPS", "0"))
LOWER_TOLERANCE_M = 0.01  # the object may touch down a little early when lowered
RELEASE_STEPS = 45
FINAL_SETTLE_STEPS = 60
LIFT_MIN_RISE_FRACTION = 0.5
MAX_GRASP_TRIES = 10       # grasps ruled out (a later move cannot be planned) before giving up
CARRY_TOLERANCE_M = 0.02
PLACE_TOLERANCE_M = 0.02
MAX_TILT_DEG = 10.0
LOOK_WAIT_STEPS = 15       # at the look pose: the arm settles, and frames rendered since arrive
LOOK_TARGET_Z = 0.04       # the camera aims this high above the pick spot
PLANNING = "planning"      # yielded by Cycle.run() while it waits for a plan: an update, but no physics step
PLANNING_FRAME_S = 0.1     # while it waits, an update at most this often: with Kit drawing frames flat out,
                           # cuMotion in the planning process took half as long again (4 cycles: 54 s, not 35)


# --- pose helpers (4x4 numpy matrices acting on column vectors) ----------------------------
def quat_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def make_pose(position, rotation: np.ndarray) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = rotation
    m[:3, 3] = position
    return m


def translation(v) -> np.ndarray:
    return make_pose(v, np.eye(3))


def rot_z(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return make_pose([0, 0, 0], np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]))


def angle_between(a: np.ndarray, b: np.ndarray) -> float:
    """Angle in degrees of the rotation between two poses' orientations."""
    r = a[:3, :3].T @ b[:3, :3]
    return float(np.degrees(np.arccos(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0))))


def backed(pose: np.ndarray, d: float) -> np.ndarray:
    """The pose moved back d along its own approach axis (+z of the gripper frame)."""
    out = pose.copy()
    out[:3, 3] -= pose[:3, 2] * d
    return out


def raised(pose: np.ndarray, dz: float) -> np.ndarray:
    out = pose.copy()
    out[2, 3] += dz
    return out


def body_pose(path: str) -> np.ndarray:
    r = omni.physx.get_physx_interface().get_rigidbody_transformation(path)
    if not r or not r.get("ret_val"):
        raise RuntimeError(f"no physics pose for {path}")
    x, y, z, w = r["rotation"]
    return make_pose(r["position"], quat_matrix(w, x, y, z))


@dataclass
class Vision:
    """What the cycle needs to find the object with the wrist camera instead of being told."""
    camera: Any                     # wrist_camera.WristCamera (imports pxr, so not imported here)
    model: perception.ObjectModel   # points on the object's surface, in its body frame
    mount: np.ndarray               # the camera on the gripper as the robot believes it (hand-eye calibration)
    rest_body: np.ndarray           # the object's body pose at rest in the grasp matrix's world, which
                                    # the stored grasps refer to
    noise: float | None             # depth noise (camera_model.simulate_depth scale); None: perfect depth
    rng: np.random.Generator
    roi_centre: tuple = cell.PICK_XY  # the pick area the camera looks at
    intr: perception.Intrinsics = field(default_factory=camera_model.intrinsics)
    link: Any = None                # ros_link.RosLink (step 5): ask the object_locator node over ROS 2
                                    # instead of running the perception in this process


class Cycle:
    """One pick at the object's current pose and place at the place spot.

    grasps: robust grasps as gripper poses in the cell for the object at `object_start` ("pose"),
    and in the grasp matrix's world ("matrix_pose", used with a Vision).
    place_shift: where the object should end up, as a translation of its start pose: for scoring,
    and for control too unless a Vision gives the object's pose.
    slip_correction: aim the lower by where the object sits in the hand (needs its true pose, so
    never with a Vision).
    look_first: without a Vision, still move to the look pose before choosing a grasp, so the
    grasp choice starts where it does with the camera (for comparisons).
    assumed: blind, without a Vision: (body pose, place shift) the robot takes for granted instead
    of the true ones, those of the object's nominal start pose. Grasps, the planner's obstacle and
    the place pose come from them; no slip correction (it needs the true pose)."""

    def __init__(self, robot, c: cell.Cell, plan: planner_mod.Planner, grasps: list[dict],
                 object_start: np.ndarray, place_shift: np.ndarray, dt: float, speed: float = SPEED,
                 on_phase=None, vision: Vision | None = None, slip_correction: bool = True,
                 look_first: bool = False, assumed: tuple[np.ndarray, np.ndarray] | None = None):
        assert vision is None or assumed is None
        self.robot, self.cell, self.planner, self.grasps = robot, c, plan, grasps
        self.on_phase = on_phase  # called with each phase's name when it ends (e.g. to take a picture)
        self.object_start, self.place_shift, self.dt, self.speed = object_start, place_shift, dt, speed
        self.vision, self.slip_correction = vision, slip_correction and vision is None and assumed is None
        self.look_first, self.assumed = look_first, assumed
        # what the robot moves the object by (from its estimate with a Vision, the nominal one blind)
        self.ctrl_shift = place_shift if assumed is None else assumed[1]
        self.estimate: np.ndarray | None = None
        g = c.g
        names = robot.dof_names
        self.arm_idx = [names.index(j) for j in cell.ARM_JOINTS]
        self.finger_idx = [names.index(j) for j in (c.finger_joint, *c.coupled_joints)]
        to_sim = np.radians if g["joint_type"] == "revolute" else (lambda v: v)
        self.to_usd = np.degrees if g["joint_type"] == "revolute" else (lambda v: v)
        self.finger_open, self.finger_close = float(to_sim(g["open_target"])), float(to_sim(g["close_target"]))
        self.result: dict = {"outcome": None, "detail": None, "plan_wall_s": 0.0, "max_tool_error_m": 0.0}
        self.steps_taken = 0
        self.log_lines: list[str] = []

    # --- robot I/O -------------------------------------------------------------------------
    def q(self) -> np.ndarray:
        return np.asarray(self.robot.get_dof_positions().numpy(), dtype=float).reshape(-1)[self.arm_idx]

    def command(self, q: np.ndarray, qd: np.ndarray | None = None) -> None:
        """Joint position targets, and velocity targets (zero if not given): with a zero velocity
        target the drives' damping would pull against the motion."""
        self.robot.set_dof_position_targets(np.asarray(q, dtype=np.float32).reshape(1, -1), dof_indices=self.arm_idx)
        qd = np.zeros(len(self.arm_idx)) if qd is None else qd
        self.robot.set_dof_velocity_targets(np.asarray(qd, dtype=np.float32).reshape(1, -1), dof_indices=self.arm_idx)

    def set_finger(self, target: float) -> None:
        self.robot.set_dof_position_targets(np.full((1, len(self.finger_idx)), target, dtype=np.float32),
                                            dof_indices=self.finger_idx)

    def finger_fraction(self) -> float:
        pos = float(self.to_usd(np.asarray(self.robot.get_dof_positions().numpy()).reshape(-1)[self.finger_idx[0]]))
        g = self.cell.g
        return (pos - g["open_target"]) / (g["pads_touch_position"] - g["open_target"])

    def tool(self) -> np.ndarray:
        return body_pose(cell.WRIST)

    def obj(self) -> np.ndarray:
        return body_pose(str(self.cell.object_body.GetPath()))

    def log(self, msg: str) -> None:
        line = f"[cell] t={self.steps_taken * self.dt:6.2f}s {msg}"
        self.log_lines.append(line)
        print(line, flush=True)

    # --- building blocks ---------------------------------------------------------------------
    def wait(self, n: int):
        for _ in range(n):
            self.steps_taken += 1
            yield

    def plan(self, name: str, fn, q: np.ndarray | None = None):
        """fn's plan (fn: a planner call, given the arm's joints: q, or where the arm is now), or
        None. Yields PLANNING until the planning process answers, every PLANNING_FRAME_S."""
        job = planner_mod.Pending(fn, self.q() if q is None else q)
        while not job.wait(PLANNING_FRAME_S):
            yield PLANNING
        p = job.result()
        self.result["plan_wall_s"] += job.wall_s
        if p is None:
            self.log(f"{name}: no plan")
        return p

    def follow(self, p: planner_mod.Plan, target: np.ndarray, name: str, tolerance_m: float = TOOL_TOLERANCE_M):
        """Stream the trajectory as joint position targets, then hold its end until the tool is
        at the target pose. Sets the outcome to tracking if it never gets there."""
        # The optimizer's trajectories run at the URDF's joint limits, faster than the simulated
        # UR10's drives (force-limited) can follow; played at full speed the wrist lags and the
        # gripper leaves the planned, collision-free path. So they are played slowed down.
        n = int(np.ceil(p.duration / (self.dt * self.speed)))
        self.result["motion_s"] = self.result.get("motion_s", 0.0) + n * self.dt
        for k in range(1, n + 1):
            t = min(k * self.dt * self.speed, p.duration)
            pos, vel = p.at(t)
            self.command(pos, vel * self.speed if k * self.dt * self.speed < p.duration else None)
            yield from self.wait(1)
            err = float(np.linalg.norm(self.tool()[:3, 3] - p_tool_at(self.planner, p, t)[:3, 3]))
            self.result["max_tool_error_m"] = max(self.result["max_tool_error_m"], err)
            if DEBUG_STEPS and k <= DEBUG_STEPS:
                qa, qc = self.q(), p.at(t)[0]
                allq = np.asarray(self.robot.get_dof_positions().numpy()).reshape(-1)
                allv = np.asarray(self.robot.get_dof_velocities().numpy()).reshape(-1)
                forces = np.asarray(self.robot.get_dof_projected_joint_forces().numpy()).reshape(-1)
                self.log(f"  {name} k={k} wrist_1 cmd {qc[3]:.4f} act {qa[3]:.4f} v {allv[self.arm_idx[3]]:+.3f} "
                         f"F {forces[self.arm_idx[3]]:+.1f} | other err {np.max(np.abs(np.delete(qa - qc, 3))):.4f} | "
                         f"gripper q {np.round(allq[6:], 3)} v {np.round(allv[6:], 2)}")
            if err > MAX_TRACKING_ERROR_M:
                self.fail("tracking", f"{name}: tool {err * 1000:.0f} mm behind the planned path")
                self.log(f"  joints {np.round(self.q(), 3)} vs commanded {np.round(p.at(t)[0], 3)}")
                allv = np.asarray(self.robot.get_dof_velocities().numpy()).reshape(-1)[self.arm_idx]
                forces = np.asarray(self.robot.get_dof_projected_joint_forces().numpy()).reshape(-1)[self.arm_idx]
                self.log(f"  velocities {np.round(allv, 2)} vs commanded {np.round(p.at(t)[1] * self.speed, 2)}; "
                         f"joint torques {np.round(forces, 1)}")
                if DEBUG_STEPS:
                    self.log(f"  contacts {contacts()}")
                return
        final = p.final()
        tool, pos_err = self.tool(), float("inf")
        for _ in range(HOLD_MAX_STEPS):
            self.command(final)
            tool = self.tool()
            pos_err = float(np.linalg.norm(tool[:3, 3] - target[:3, 3]))
            if pos_err < tolerance_m and angle_between(tool, target) < TOOL_TOLERANCE_DEG:
                self.log(f"{name}: done ({p.duration:.2f}s planned, tool {pos_err * 1000:.1f} mm off)")
                if self.on_phase:
                    self.on_phase(name)
                return
            yield from self.wait(1)
        self.fail("tracking", f"{name}: tool {pos_err * 1000:.1f} mm, {angle_between(tool, target):.1f} deg off")
        self.log(f"  joints {np.round(self.q(), 3)} vs target {np.round(final, 3)}")

    def moves(self, grasp: np.ndarray) -> list[tuple[str, np.ndarray, bool]]:
        """The straight moves after the pre-grasp pose, as planned before the arm moves (no slip
        yet): name, tool target, and whether the table is an obstacle for it."""
        lifted = raised(grasp, LIFT_M)
        place = translation(self.ctrl_shift) @ raised(grasp, PLACE_CLEARANCE_M)
        above = raised(place, LIFT_M - PLACE_CLEARANCE_M)
        return [("approach", grasp, False), ("lift", lifted, False), ("transport", above, True),
                ("lower", place, False), ("retract", backed(place, PRE_GRASP_M), False)]

    def plan_moves(self, q: np.ndarray, grasp: np.ndarray):
        """Plan each straight move from the end of the previous one, starting at q. Returns the
        name of the first that cannot be planned (None if all can) and the plans by name; yields
        PLANNING meanwhile, as plan() does.

        These plans are the ones executed. Re-planning a move from where the arm actually is (a
        fraction of a millimetre away) sometimes failed where this plan had succeeded."""
        self.planner.set_enabled("object", False)
        plans = {}
        for name, target, table in self.moves(grasp):
            self.planner.set_enabled("table", table)
            p = yield from self.plan(f"plan {name}", lambda q0, target=target: self.planner.plan_line(q0, target), q)
            if p is None:
                return name, plans
            plans[name] = p
            q = p.final()
        return None, plans

    def go_look(self):
        """Move to the look pose and wait there (LOOK_WAIT_STEPS)."""
        r, pl = self.result, self.planner
        target = camera_model.look_tool_pose([*cell.PICK_XY, LOOK_TARGET_Z], self.cell.gripper_name)
        pl.set_enabled("table", True)
        pl.set_enabled("object", False)
        p = yield from self.plan("to look pose", lambda q: pl.plan_pose(q, target))
        if p is None:
            return self.fail("plan_failed", "look pose")
        yield from self.follow(p, target, "to look pose")
        if r["outcome"]:
            return
        yield from self.wait(LOOK_WAIT_STEPS)

    def look(self):
        """Move to the look pose, take a depth image, and find the object's pose in it."""
        v, r, pl = self.vision, self.result, self.planner
        assert v is not None
        start = self.steps_taken
        yield from self.go_look()
        if r["outcome"]:
            return
        frame = v.camera.read()
        if frame is None:
            return self.fail("not_found", "no camera image")
        depth = frame[1] if v.noise is None else camera_model.simulate_depth(frame[1], v.intr.fx, v.rng, v.noise)
        if v.link is not None:
            # Step 5: publish the image; the object_locator node gets the camera's pose from TF
            # (the arm's links from this simulation, the mount from the calibration) and answers.
            # The simulation waits meanwhile, as it does for the in-process perception.
            import omni.timeline

            sim_time = omni.timeline.get_timeline_interface().get_current_time()
            est, waited = v.link.locate(frame[0], depth, sim_time, true_pose=self.obj())
            r["perception_wall_s"] = round(waited, 2)
        else:
            # the camera's pose from the joint readings and the calibrated mount, as a real robot knows it
            T_wc = camera_model.camera_pose(pl.tool_pose(self.q()), v.mount)
            t0 = time.perf_counter()
            est = perception.locate(depth, v.intr, T_wc, v.model, v.roi_centre)
            r["perception_wall_s"] = round(time.perf_counter() - t0, 2)
        r["look_s"] = round((self.steps_taken - start) * self.dt, 2)
        r["est_points"] = est.n_points
        if est.pose is None:
            return self.fail("not_found", est.detail)
        err = perception.pose_errors(v.model, est.pose, self.obj())  # the true pose: scoring only
        r.update(est_centre_mm=round(err["centre_mm"], 2), est_rot_deg=round(err["rot_deg"], 2),
                 est_adds_mm=round(err["adds_mm"], 2), est_add_mm=round(err["add_mm"], 2),
                 est_fitness=round(est.fitness, 3), est_violation=round(est.violation, 4))
        self.log(f"object found from {est.n_points} points ({r['perception_wall_s']:.1f}s): centre {err['centre_mm']:.1f} mm, "
                 f"{err['rot_deg']:.1f} deg, ADD-S {err['adds_mm']:.1f} mm from the true pose")
        self.estimate = est.pose

    def fail(self, outcome: str, detail: str) -> None:
        self.result["outcome"], self.result["detail"] = outcome, detail
        self.log(f"FAILED {outcome}: {detail}")

    # --- the cycle ---------------------------------------------------------------------------
    def run(self):
        r, pl = self.result, self.planner
        self.set_finger(self.finger_open)
        yield from self.wait(SETTLE_STEPS)
        start_step = self.steps_taken
        # The gripper mount and every target assume the wrist body sits where cuMotion puts tool0.
        tool, fk = self.tool(), pl.tool_pose(self.q())
        off, turn = float(np.linalg.norm(tool[:3, 3] - fk[:3, 3])), angle_between(tool, fk)
        if off > 0.002 or turn > 0.5:
            raise RuntimeError(f"wrist frame is {off * 1000:.1f} mm, {turn:.2f} deg from cuMotion's tool0")
        obj0 = self.obj()  # the true pose; with a Vision only for the gates and the score
        if self.vision is None:
            if self.look_first:  # start the grasp choice where the camera mode does
                yield from self.go_look()
                if r["outcome"]:
                    return
            # The grasps refer to the object's start pose; follow it if it moved while settling.
            # Blind: put them where the object is assumed to be instead.
            seen = obj0 if self.assumed is None else self.assumed[0]
            correction = seen @ np.linalg.inv(self.object_start)
            grasps = [correction @ g["pose"] for g in self.grasps]
        else:
            yield from self.look()
            if r["outcome"]:
                return
            seen = self.estimate
            to_seen = seen @ np.linalg.inv(self.vision.rest_body)  # grasp-matrix world -> the object as seen
            grasps = [to_seen @ g["matrix_pose"] for g in self.grasps]
            # Move the object by what takes its centroid, as seen, to where the centroid belongs on the
            # place spot. By the centroid, not the body origin: for a near-symmetric object the
            # estimated orientation is arbitrary, and with it where the origin is (the lemon's sits
            # 37 mm below its centroid, so a 43 degree turn put it 25 mm off). Only the estimate's yaw
            # turns the centroid's offset from the pick point, which is small for those objects.
            c = np.append(self.vision.model.centre, 1.0)
            seen_c, rest_c = (seen @ c)[:3], (self.vision.rest_body @ c)[:3]  # rest_c: offset from the pick point
            yaw = np.arctan2(to_seen[1, 0], to_seen[0, 0])
            offset = rot_z(yaw)[:3, :3] @ np.array([rest_c[0], rest_c[1], 0.0])
            self.ctrl_shift = np.array([cell.PLACE_XY[0] + offset[0] - seen_c[0], cell.PLACE_XY[1] + offset[1] - seen_c[1], 0.0])
        centre, size = self.cell.object_box()  # the object is an obstacle on the way to the pre-grasp pose
        pl.add_cuboid("object", size=[s + 0.02 for s in size], pose=seen @ translation(list(centre)))

        # 1. Choose a grasp: the planner picks the robust grasp whose pre-grasp pose it reaches
        # best; then every straight move of the cycle is planned from there before the arm moves.
        # If one cannot be planned, that grasp is ruled out and the planner picks again.
        pre = [backed(g, PRE_GRASP_M) for g in grasps]
        ruled_out: dict[int, str] = {}
        while True:
            candidates = [i for i in range(len(grasps)) if i not in ruled_out]
            pl.set_enabled("table", True)
            pl.set_enabled("object", True)
            p = None
            if candidates:
                p = yield from self.plan("to pre-grasp", lambda q: pl.plan_goalset(q, [pre[i] for i in candidates]))
            if p is None or len(ruled_out) >= MAX_GRASP_TRIES:
                r["rejected_grasps"] = len(ruled_out)
                if p is not None:
                    detail = f"gave up after ruling out {len(ruled_out)} grasps"
                elif ruled_out:
                    detail = (f"{len(ruled_out)} of {len(grasps)} robust grasps ruled out by a later move "
                              f"({', '.join(sorted(set(ruled_out.values())))}); no motion to any other")
                else:
                    detail = f"no motion to any of {len(grasps)} robust grasps"
                return self.fail("unreachable", detail)
            chosen = candidates[p.target_index]
            blocked, plans = yield from self.plan_moves(p.final(), grasps[chosen])
            if blocked is None:
                break
            ruled_out[chosen] = blocked
            self.log(f"grasp {self.grasps[chosen]['index']}: its {blocked} cannot be planned; choosing another")
        choice, grasp = self.grasps[chosen], grasps[chosen]
        r["grasp_index"], r["approach_z"], r["rejected_grasps"] = choice["index"], choice["approach_z"], len(ruled_out)
        self.log(f"grasp {choice['index']} (approach z {choice['approach_z']:.2f}) of {len(grasps)}")
        yield from self.follow(p, pre[chosen], "to pre-grasp")
        if r["outcome"]:
            return

        # 2. straight in along the approach axis; the table and object are not obstacles now:
        # the grasp matrix checked that the fingertips clear the table at the grasp.
        yield from self.follow(plans["approach"], grasp, "approach")
        if r["outcome"]:
            return

        # 3. close; first note how far the approach pushed the object (scoring only)
        r["push_m"] = round(float(np.linalg.norm(self.obj()[:3, 3] - obj0[:3, 3])), 4)
        if r["push_m"] > 0.005:
            self.log(f"the approach pushed the object {r['push_m'] * 1000:.0f} mm")
        self.set_finger(self.finger_close)
        yield from self.wait(gm_config.CLOSE_STEPS)
        r["finger_fraction"] = round(self.finger_fraction(), 4)
        if r["finger_fraction"] >= gm_config.MISSED_FRACTION:
            return self.fail("missed", f"fingers closed to {r['finger_fraction']:.2f} of the way")
        obj_in_tool = np.linalg.inv(self.tool()) @ self.obj()
        self.log(f"closed ({r['finger_fraction']:.2f} of the way); the object is "
                 f"{np.linalg.norm(self.obj()[:3, 3] - obj0[:3, 3]) * 1000:.0f} mm from its start")
        if self.on_phase:
            self.on_phase("close")

        # 4. lift straight up
        moves = dict((name, target) for name, target, _ in self.moves(grasp))
        yield from self.follow(plans["lift"], moves["lift"], "lift")
        if r["outcome"]:
            return
        rise = self.obj()[2, 3] - obj0[2, 3]
        if rise < LIFT_MIN_RISE_FRACTION * LIFT_M:
            return self.fail("dropped_lift", f"object rose {rise * 1000:.0f} mm")
        pick_done = self.steps_taken

        # 5. carry to above the place spot, in a straight line at the same height
        place = moves["lower"]
        yield from self.follow(plans["transport"], moves["transport"], "transport")
        if r["outcome"]:
            return
        carried = self.tool() @ obj_in_tool
        drift = float(np.linalg.norm(self.obj()[:3, 3] - carried[:3, 3]))
        if drift > CARRY_TOLERANCE_M:
            return self.fail("dropped_transport", f"object {drift * 1000:.0f} mm from where the grip held it")

        # 6. lower; aim so the object, not the tool, ends PLACE_CLEARANCE_M above its rest pose,
        # which absorbs any slip in the fingers since the grasp closed. But never lower than the
        # place pose itself: if the object slid towards the palm, aiming lower would put the
        # fingertips into the table (they cleared it by only a few mm at the grasp); the object
        # is then released that much higher.
        slip_now = np.linalg.inv(self.tool()) @ self.obj()
        r["slip_m"] = round(float(np.linalg.norm(slip_now[:3, 3] - obj_in_tool[:3, 3])), 4)
        r["slip_deg"] = round(angle_between(slip_now, obj_in_tool), 2)
        expected = translation(self.place_shift) @ obj0  # where the object should end up (scoring)
        if self.slip_correction:
            landing = place @ slip_now
            aim = translation(raised(expected, PLACE_CLEARANCE_M)[:3, 3] - landing[:3, 3]) @ place
            aim[2, 3] = max(aim[2, 3], place[2, 3])
            pl.set_enabled("table", False)
            p = yield from self.plan("lower", lambda q: pl.plan_line(q, aim))
            if p is None:  # fall back to the pre-planned move to the nominal place pose
                p, aim = plans["lower"], place
        else:  # the robot cannot see the object in its hand: lower as planned
            p, aim = plans["lower"], place
        yield from self.follow(p, aim, "lower", tolerance_m=LOWER_TOLERANCE_M)
        if r["outcome"]:
            return

        # 7. release, back out along the approach axis, go home
        off = raised(expected, PLACE_CLEARANCE_M)[:3, 3] - self.obj()[:3, 3]
        self.log(f"lowered: the object is {np.round(off * 1000, 0)} mm from where it should be released")
        self.set_finger(self.finger_open)
        yield from self.wait(RELEASE_STEPS)
        self.log(f"released: the object is {np.linalg.norm(self.obj()[:3, 3] - expected[:3, 3]) * 1000:.0f} mm from the target")
        out = backed(aim, PRE_GRASP_M)
        p = yield from self.plan("retract", lambda q: pl.plan_line(q, out))
        if p is None:  # fall back to the pre-planned move (it starts from the nominal place pose)
            p, out = plans["retract"], moves["retract"]
        yield from self.follow(p, out, "retract")
        if r["outcome"]:
            return
        # Home, around the object where the robot believes it put it (before, the planner's object
        # box stayed at the pick spot, and one placed mug ended up 109 mm away).
        pl.set_enabled("table", True)
        pl.add_cuboid("object", size=[s + 0.02 for s in size], pose=translation(self.ctrl_shift) @ seen @ translation(list(centre)))
        pl.set_enabled("object", True)
        p = yield from self.plan("home", lambda q: pl.plan_joints(q, cell.HOME_Q))
        if p is None:  # e.g. the gripper starts inside the padded box
            pl.set_enabled("object", False)
            p = yield from self.plan("home without the placed object", lambda q: pl.plan_joints(q, cell.HOME_Q))
        if p is None:
            return self.fail("plan_failed", "home")
        yield from self.follow(p, pl.tool_pose(cell.HOME_Q), "home")
        if r["outcome"]:
            return
        r["cycle_s"] = round((self.steps_taken - start_step) * self.dt, 2)
        r["pick_s"] = round((pick_done - start_step) * self.dt, 2)
        yield from self.wait(FINAL_SETTLE_STEPS)

        final = self.obj()
        r["place_error_m"] = round(float(np.linalg.norm(final[:3, 3] - expected[:3, 3])), 4)
        r["tilt_deg"] = round(float(np.degrees(np.arccos(np.clip(final[:3, 2] @ expected[:3, 2], -1, 1)))), 2)
        ok = r["place_error_m"] < PLACE_TOLERANCE_M and r["tilt_deg"] < MAX_TILT_DEG
        r["outcome"] = "placed" if ok else "misplaced"
        self.log(f"{r['outcome']}: {r['place_error_m'] * 1000:.1f} mm from the target, tilted {r['tilt_deg']:.1f} deg, "
                 f"cycle {r['cycle_s']:.1f}s")


def contacts() -> list[str]:
    """Body pairs in contact this step (debugging; needs PhysxContactReportAPI on the bodies)."""
    from omni.physx import get_physx_simulation_interface
    from pxr import PhysicsSchemaTools

    headers, _ = get_physx_simulation_interface().get_contact_report()
    out = set()
    for h in headers:
        a = str(PhysicsSchemaTools.intToSdfPath(h.actor0)).split("/")[-1]
        b = str(PhysicsSchemaTools.intToSdfPath(h.actor1)).split("/")[-1]
        out.add(f"{a}-{b}")
    return sorted(out)


def p_tool_at(pl: planner_mod.Planner, p: planner_mod.Plan, t: float) -> np.ndarray:
    return pl.tool_pose(p.at(t)[0])
