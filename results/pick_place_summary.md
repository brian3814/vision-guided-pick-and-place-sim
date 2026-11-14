# Pick-and-place cell results

A UR10 with the gripper on its flange picks the object off the table and puts it on the place spot,
using the object's robust grasps from the grasp matrix (held under every test condition). Each cycle
starts with the object turned and shifted (seeded: the same start poses in every run with the same
seed); the planner picks whichever robust grasp it can reach. *Pose from*: the simulation tells the
robot where the object is (step 3), or the wrist camera finds it (step 4), or nothing does (*blind*:
the robot takes it to be at its nominal pose, not turned or shifted; the baseline for the camera).
Latest finished run per gripper x object x pose source (a shorter run does not replace a longer one).

Columns: *grasps used* is how many different robust grasps the planner chose; *ruled out* counts
grasps it chose first but dropped because a later move of the cycle could not be planned; *side
grasps* approached at less than 30 degrees below horizontal (approach z above -0.5); *slip* is how far
the object moved in the fingers between closing and lowering. Medians are over placed cycles.

| Gripper | Object | Pose from | Robust grasps | Placed | Failures | Grasps used | Ruled out | Side grasps | Cycle s | Place error mm | Slip mm | Slip deg |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2F-140 | beer_stein_K58BL2P1CR | simulation | 34 | 7/10 | dropped_transport 2, unreachable 1 | 4 | 3 | 3 | 13.4 | 0.9 | 2.0 | 0.9 |
| 2F-140 | jar_2CKZZHE3PV | simulation | 45 | 10/10 | – | 4 | 2 | 6 | 16.3 | 1.2 | 0.1 | 0.1 |
| 2F-140 | lemon_8SUCD6125H | simulation | 49 | 8/10 | unreachable 2 | 4 | 4 | 6 | 16.3 | 0.2 | 0.0 | 0.1 |
| 2F-140 | mug_A07RDPW843 | simulation | 28 | 9/10 | unreachable 1 | 5 | 0 | 0 | 16.1 | 0.4 | 0.1 | 0.1 |
| 2F-140 | toy_block_6MPL7NVFVU | simulation | 31 | 9/10 | misplaced 1 | 5 | 2 | 2 | 10.0 | 0.4 | 2.3 | 1.7 |
| 2F-140 | beer_stein_K58BL2P1CR | camera | 34 | 7/10 | dropped_transport 2, tracking 1 | 4 | 6 | 0 | 16.4 | 7.1 | 2.7 | 1.7 |
| 2F-140 | jar_2CKZZHE3PV | camera | 45 | 10/10 | – | 4 | 0 | 0 | 16.2 | 1.9 | 0.0 | 0.0 |
| 2F-140 | lemon_8SUCD6125H | camera | 49 | 10/10 | – | 5 | 9 | 1 | 15.9 | 3.3 | 0.2 | 0.3 |
| 2F-140 | mug_A07RDPW843 | camera | 28 | 10/10 | – | 6 | 3 | 0 | 15.0 | 1.4 | 0.2 | 0.1 |
| 2F-140 | toy_block_6MPL7NVFVU | camera | 31 | 10/10 | – | 3 | 3 | 0 | 13.0 | 8.1 | 3.9 | 2.6 |
| 2F-140 | beer_stein_K58BL2P1CR | camera via ROS 2 | 34 | 7/10 | dropped_transport 3 | 4 | 6 | 0 | 16.5 | 5.7 | 0.4 | 0.2 |
| 2F-140 | jar_2CKZZHE3PV | camera via ROS 2 | 45 | 10/10 | – | 4 | 1 | 0 | 16.1 | 1.6 | 0.1 | 0.1 |
| 2F-140 | lemon_8SUCD6125H | camera via ROS 2 | 49 | 10/10 | – | 5 | 7 | 1 | 15.8 | 3.6 | 0.7 | 0.3 |
| 2F-140 | mug_A07RDPW843 | camera via ROS 2 | 28 | 10/10 | – | 6 | 2 | 0 | 15.0 | 1.2 | 0.2 | 0.1 |
| 2F-140 | toy_block_6MPL7NVFVU | camera via ROS 2 | 31 | 10/10 | – | 3 | 3 | 0 | 13.2 | 7.9 | 2.4 | 1.9 |
| 2F-140 | beer_stein_K58BL2P1CR | nowhere: blind, nominal pose | 34 | 4/10 | dropped_lift 2, dropped_transport 3, misplaced 1 | 1 | 0 | 0 | 17.1 | 8.8 | 1.6 | 1.7 |
| 2F-140 | jar_2CKZZHE3PV | nowhere: blind, nominal pose | 45 | 6/10 | misplaced 4 | 1 | 0 | 0 | 16.1 | 4.8 | 0.1 | 0.1 |
| 2F-140 | lemon_8SUCD6125H | nowhere: blind, nominal pose | 49 | 6/10 | dropped_lift 1, misplaced 3 | 1 | 10 | 0 | 15.9 | 7.8 | 0.2 | 0.4 |
| 2F-140 | mug_A07RDPW843 | nowhere: blind, nominal pose | 28 | 3/10 | dropped_transport 1, misplaced 3, tracking 3 | 1 | 0 | 0 | 15.3 | 4.1 | 0.0 | 0.1 |
| 2F-140 | toy_block_6MPL7NVFVU | nowhere: blind, nominal pose | 31 | 4/10 | dropped_lift 1, misplaced 5 | 2 | 0 | 0 | 11.2 | 7.5 | 1.9 | 3.0 |
| 2F-140 | beer_stein_K58BL2P1CR | simulation, from the look pose, no slip correction | 34 | 7/10 | dropped_transport 3 | 3 | 6 | 0 | 16.5 | 3.5 | 3.9 | 4.2 |
| 2F-140 | jar_2CKZZHE3PV | simulation, from the look pose, no slip correction | 45 | 10/10 | – | 3 | 0 | 0 | 16.1 | 1.6 | 0.1 | 0.1 |
| 2F-140 | lemon_8SUCD6125H | simulation, from the look pose, no slip correction | 49 | 10/10 | – | 5 | 5 | 1 | 15.9 | 1.6 | 0.2 | 0.1 |
| 2F-140 | mug_A07RDPW843 | simulation, from the look pose, no slip correction | 28 | 10/10 | – | 5 | 0 | 0 | 15.5 | 1.9 | 0.1 | 0.1 |
| 2F-140 | toy_block_6MPL7NVFVU | simulation, from the look pose, no slip correction | 31 | 10/10 | – | 3 | 3 | 0 | 13.3 | 8.2 | 2.3 | 1.7 |
| 2F-140 | beer_stein_K58BL2P1CR | simulation, no slip correction | 34 | 7/10 | dropped_transport 2, unreachable 1 | 4 | 3 | 3 | 13.3 | 5.5 | 2.0 | 0.9 |
| 2F-140 | jar_2CKZZHE3PV | simulation, no slip correction | 45 | 4/10 | misplaced 6 | 4 | 2 | 6 | 16.0 | 1.7 | 0.1 | 0.1 |
| 2F-140 | lemon_8SUCD6125H | simulation, no slip correction | 49 | 8/10 | unreachable 2 | 4 | 4 | 6 | 16.3 | 2.4 | 0.0 | 0.1 |
| 2F-140 | mug_A07RDPW843 | simulation, no slip correction | 28 | 9/10 | unreachable 1 | 5 | 0 | 0 | 16.1 | 2.0 | 0.1 | 0.1 |
| 2F-140 | toy_block_6MPL7NVFVU | simulation, no slip correction | 31 | 9/10 | misplaced 1 | 5 | 2 | 2 | 9.9 | 7.7 | 2.3 | 1.7 |

Overall, pose from simulation: 43/50 cycles placed (86%).
Overall, pose from camera: 47/50 cycles placed (94%).
Overall, pose from camera via ROS 2: 47/50 cycles placed (94%).
Overall, pose from nowhere: blind, nominal pose: 23/50 cycles placed (46%).
Overall, pose from simulation, from the look pose, no slip correction: 47/50 cycles placed (94%).
Overall, pose from simulation, no slip correction: 37/50 cycles placed (74%).

## The camera's pose estimates

Against the true pose at the moment of the image. *Centre*: distance between the object model's centroid
as estimated and as true. *ADD-S*: mean distance from each estimated model point to the nearest true one
(near zero for a pose that a symmetric object cannot be told apart from). *Look s*: simulated time from
home to the image. *Compute s*: wall time of perception.locate().

| Object | Pose from | Found | Centre mm (median / max) | Rotation deg (median / max) | ADD-S mm (median / max) | Look s | Compute s |
|---|---|---|---|---|---|---|---|
| beer_stein_K58BL2P1CR | camera | 10/10 | 0.6 / 0.7 | 0.6 / 1.5 | 0.7 / 0.8 | 2.3 | 3.6 |
| beer_stein_K58BL2P1CR | camera via ROS 2 | 10/10 | 0.6 / 0.7 | 0.6 / 1.5 | 0.7 / 0.8 | 2.3 | 3.9 |
| jar_2CKZZHE3PV | camera | 10/10 | 0.7 / 0.8 | 99.3 / 144.4 | 1.1 / 1.1 | 2.3 | 5.5 |
| jar_2CKZZHE3PV | camera via ROS 2 | 10/10 | 0.7 / 0.8 | 99.2 / 149.9 | 1.1 / 1.1 | 2.3 | 6.0 |
| lemon_8SUCD6125H | camera | 10/10 | 0.5 / 0.8 | 0.9 / 43.0 | 0.6 / 1.4 | 2.3 | 4.0 |
| lemon_8SUCD6125H | camera via ROS 2 | 10/10 | 0.5 / 0.8 | 0.8 / 43.0 | 0.6 / 1.4 | 2.3 | 4.0 |
| mug_A07RDPW843 | camera | 10/10 | 0.5 / 1.1 | 0.7 / 0.9 | 0.6 / 1.0 | 2.3 | 6.2 |
| mug_A07RDPW843 | camera via ROS 2 | 10/10 | 0.5 / 1.1 | 0.7 / 0.9 | 0.6 / 1.0 | 2.3 | 6.7 |
| toy_block_6MPL7NVFVU | camera | 10/10 | 0.7 / 1.3 | 0.6 / 1.2 | 0.7 / 1.1 | 2.3 | 3.5 |
| toy_block_6MPL7NVFVU | camera via ROS 2 | 10/10 | 0.7 / 1.3 | 0.6 / 1.2 | 0.7 / 1.1 | 2.3 | 4.3 |

## Same start poses, pose from the camera vs from elsewhere

Cycle by cycle: each cycle index has the same start pose in every run with the same seed. *Blind*: the
robot takes the object to be at its nominal pose (not turned or shifted), what it would do without the
camera.

| Object | Camera run | Compared with | Cycles | Placed by both | Only with the other | Only with the camera's | Neither |
|---|---|---|---|---|---|---|---|
| beer_stein_K58BL2P1CR | camera | nowhere: blind, nominal pose | 10 | 2 | 2 | 5 | 1 |
| beer_stein_K58BL2P1CR | camera via ROS 2 | nowhere: blind, nominal pose | 10 | 3 | 1 | 4 | 2 |
| jar_2CKZZHE3PV | camera | nowhere: blind, nominal pose | 10 | 6 | 0 | 4 | 0 |
| jar_2CKZZHE3PV | camera via ROS 2 | nowhere: blind, nominal pose | 10 | 6 | 0 | 4 | 0 |
| lemon_8SUCD6125H | camera | nowhere: blind, nominal pose | 10 | 6 | 0 | 4 | 0 |
| lemon_8SUCD6125H | camera via ROS 2 | nowhere: blind, nominal pose | 10 | 6 | 0 | 4 | 0 |
| mug_A07RDPW843 | camera | nowhere: blind, nominal pose | 10 | 3 | 0 | 7 | 0 |
| mug_A07RDPW843 | camera via ROS 2 | nowhere: blind, nominal pose | 10 | 3 | 0 | 7 | 0 |
| toy_block_6MPL7NVFVU | camera | nowhere: blind, nominal pose | 10 | 4 | 0 | 6 | 0 |
| toy_block_6MPL7NVFVU | camera via ROS 2 | nowhere: blind, nominal pose | 10 | 4 | 0 | 6 | 0 |
| beer_stein_K58BL2P1CR | camera | simulation | 10 | 5 | 2 | 2 | 1 |
| beer_stein_K58BL2P1CR | camera via ROS 2 | simulation | 10 | 6 | 1 | 1 | 2 |
| jar_2CKZZHE3PV | camera | simulation | 10 | 10 | 0 | 0 | 0 |
| jar_2CKZZHE3PV | camera via ROS 2 | simulation | 10 | 10 | 0 | 0 | 0 |
| lemon_8SUCD6125H | camera | simulation | 10 | 8 | 0 | 2 | 0 |
| lemon_8SUCD6125H | camera via ROS 2 | simulation | 10 | 8 | 0 | 2 | 0 |
| mug_A07RDPW843 | camera | simulation | 10 | 9 | 0 | 1 | 0 |
| mug_A07RDPW843 | camera via ROS 2 | simulation | 10 | 9 | 0 | 1 | 0 |
| toy_block_6MPL7NVFVU | camera | simulation | 10 | 9 | 0 | 1 | 0 |
| toy_block_6MPL7NVFVU | camera via ROS 2 | simulation | 10 | 9 | 0 | 1 | 0 |
| beer_stein_K58BL2P1CR | camera | simulation, from the look pose, no slip correction | 10 | 6 | 1 | 1 | 2 |
| beer_stein_K58BL2P1CR | camera via ROS 2 | simulation, from the look pose, no slip correction | 10 | 6 | 1 | 1 | 2 |
| jar_2CKZZHE3PV | camera | simulation, from the look pose, no slip correction | 10 | 10 | 0 | 0 | 0 |
| jar_2CKZZHE3PV | camera via ROS 2 | simulation, from the look pose, no slip correction | 10 | 10 | 0 | 0 | 0 |
| lemon_8SUCD6125H | camera | simulation, from the look pose, no slip correction | 10 | 10 | 0 | 0 | 0 |
| lemon_8SUCD6125H | camera via ROS 2 | simulation, from the look pose, no slip correction | 10 | 10 | 0 | 0 | 0 |
| mug_A07RDPW843 | camera | simulation, from the look pose, no slip correction | 10 | 10 | 0 | 0 | 0 |
| mug_A07RDPW843 | camera via ROS 2 | simulation, from the look pose, no slip correction | 10 | 10 | 0 | 0 | 0 |
| toy_block_6MPL7NVFVU | camera | simulation, from the look pose, no slip correction | 10 | 10 | 0 | 0 | 0 |
| toy_block_6MPL7NVFVU | camera via ROS 2 | simulation, from the look pose, no slip correction | 10 | 10 | 0 | 0 | 0 |
| beer_stein_K58BL2P1CR | camera | simulation, no slip correction | 10 | 5 | 2 | 2 | 1 |
| beer_stein_K58BL2P1CR | camera via ROS 2 | simulation, no slip correction | 10 | 6 | 1 | 1 | 2 |
| jar_2CKZZHE3PV | camera | simulation, no slip correction | 10 | 4 | 0 | 6 | 0 |
| jar_2CKZZHE3PV | camera via ROS 2 | simulation, no slip correction | 10 | 4 | 0 | 6 | 0 |
| lemon_8SUCD6125H | camera | simulation, no slip correction | 10 | 8 | 0 | 2 | 0 |
| lemon_8SUCD6125H | camera via ROS 2 | simulation, no slip correction | 10 | 8 | 0 | 2 | 0 |
| mug_A07RDPW843 | camera | simulation, no slip correction | 10 | 9 | 0 | 1 | 0 |
| mug_A07RDPW843 | camera via ROS 2 | simulation, no slip correction | 10 | 9 | 0 | 1 | 0 |
| toy_block_6MPL7NVFVU | camera | simulation, no slip correction | 10 | 9 | 0 | 1 | 0 |
| toy_block_6MPL7NVFVU | camera via ROS 2 | simulation, no slip correction | 10 | 9 | 0 | 1 | 0 |

## Failed cycles

| Object | Pose from | Cycle | Grasp | Approach z | Outcome | Detail | Slip mm | Slip deg | Place error mm | Tilt deg |
|---|---|---|---|---|---|---|---|---|---|---|
| beer_stein_K58BL2P1CR | simulation | 0 | 38 | -0.87 | dropped_transport | object 35 mm from where the grip held it | – | – | – | – |
| beer_stein_K58BL2P1CR | simulation | 5 | 38 | -0.87 | dropped_transport | object 33 mm from where the grip held it | – | – | – | – |
| beer_stein_K58BL2P1CR | simulation | 8 | – | – | unreachable | 34 robust grasps, 0 ruled out by a later move (), no motion to the others | – | – | – | – |
| lemon_8SUCD6125H | simulation | 5 | – | – | unreachable | 49 robust grasps, 1 ruled out by a later move (approach), no motion to the others | – | – | – | – |
| lemon_8SUCD6125H | simulation | 6 | – | – | unreachable | 49 robust grasps, 0 ruled out by a later move (), no motion to the others | – | – | – | – |
| mug_A07RDPW843 | simulation | 3 | – | – | unreachable | 28 robust grasps, 0 ruled out by a later move (), no motion to the others | – | – | – | – |
| toy_block_6MPL7NVFVU | simulation | 1 | 5 | -0.07 | misplaced |  | 2.8 | 9.2 | 90.2 | 89.3 |
| beer_stein_K58BL2P1CR | camera | 1 | 102 | -1.00 | dropped_transport | object 20 mm from where the grip held it | – | – | – | – |
| beer_stein_K58BL2P1CR | camera | 5 | 102 | -1.00 | tracking | lower: tool 6.2 mm, 2.4 deg off | 11.6 | 20.1 | – | – |
| beer_stein_K58BL2P1CR | camera | 9 | 38 | -0.87 | dropped_transport | object 32 mm from where the grip held it | – | – | – | – |
| beer_stein_K58BL2P1CR | camera via ROS 2 | 0 | 38 | -0.87 | dropped_transport | object 32 mm from where the grip held it | – | – | – | – |
| beer_stein_K58BL2P1CR | camera via ROS 2 | 8 | 38 | -0.87 | dropped_transport | object 34 mm from where the grip held it | – | – | – | – |
| beer_stein_K58BL2P1CR | camera via ROS 2 | 9 | 38 | -0.87 | dropped_transport | object 36 mm from where the grip held it | – | – | – | – |
| beer_stein_K58BL2P1CR | nowhere: blind, nominal pose | 0 | 38 | -0.87 | dropped_lift | object rose 0 mm | – | – | – | – |
| beer_stein_K58BL2P1CR | nowhere: blind, nominal pose | 3 | 38 | -0.87 | dropped_transport | object 443 mm from where the grip held it | – | – | – | – |
| beer_stein_K58BL2P1CR | nowhere: blind, nominal pose | 4 | 38 | -0.87 | dropped_transport | object 44 mm from where the grip held it | – | – | – | – |
| beer_stein_K58BL2P1CR | nowhere: blind, nominal pose | 5 | 38 | -0.87 | misplaced |  | 9.2 | 7.0 | 59.8 | 24.9 |
| beer_stein_K58BL2P1CR | nowhere: blind, nominal pose | 6 | 38 | -0.87 | dropped_transport | object 23 mm from where the grip held it | – | – | – | – |
| beer_stein_K58BL2P1CR | nowhere: blind, nominal pose | 8 | 38 | -0.87 | dropped_lift | object rose -0 mm | – | – | – | – |
| jar_2CKZZHE3PV | nowhere: blind, nominal pose | 0 | 154 | -0.99 | misplaced |  | 0.6 | 0.2 | 27.6 | 0.3 |
| jar_2CKZZHE3PV | nowhere: blind, nominal pose | 4 | 154 | -0.99 | misplaced |  | 0.7 | 0.7 | 28.7 | 0.4 |
| jar_2CKZZHE3PV | nowhere: blind, nominal pose | 5 | 154 | -0.99 | misplaced |  | 3.1 | 5.1 | 23.0 | 0.4 |
| jar_2CKZZHE3PV | nowhere: blind, nominal pose | 6 | 154 | -0.99 | misplaced |  | 0.1 | 0.3 | 69.2 | 117.1 |
| lemon_8SUCD6125H | nowhere: blind, nominal pose | 0 | 118 | -0.93 | misplaced |  | 0.5 | 0.7 | 31.4 | 1.1 |
| lemon_8SUCD6125H | nowhere: blind, nominal pose | 3 | 118 | -0.93 | dropped_lift | object rose -0 mm | – | – | – | – |
| lemon_8SUCD6125H | nowhere: blind, nominal pose | 4 | 118 | -0.93 | misplaced |  | 0.5 | 1.0 | 27.5 | 8.8 |
| lemon_8SUCD6125H | nowhere: blind, nominal pose | 5 | 118 | -0.93 | misplaced |  | 1.5 | 3.8 | 26.1 | 8.7 |
| mug_A07RDPW843 | nowhere: blind, nominal pose | 0 | 116 | -0.97 | misplaced |  | 0.4 | 0.2 | 33.7 | 0.2 |
| mug_A07RDPW843 | nowhere: blind, nominal pose | 3 | 116 | -0.97 | misplaced |  | 1.1 | 1.6 | 36.2 | 0.2 |
| mug_A07RDPW843 | nowhere: blind, nominal pose | 4 | 116 | -0.97 | dropped_transport | object 30 mm from where the grip held it | – | – | – | – |
| mug_A07RDPW843 | nowhere: blind, nominal pose | 5 | 116 | -0.97 | misplaced |  | 10.8 | 19.8 | 51.9 | 0.1 |
| mug_A07RDPW843 | nowhere: blind, nominal pose | 6 | 116 | -0.97 | tracking | approach: tool 70 mm behind the planned path | – | – | – | – |
| mug_A07RDPW843 | nowhere: blind, nominal pose | 7 | 116 | -0.97 | tracking | approach: tool 52 mm behind the planned path | – | – | – | – |
| mug_A07RDPW843 | nowhere: blind, nominal pose | 8 | 116 | -0.97 | tracking | approach: tool 22 mm behind the planned path | – | – | – | – |
| toy_block_6MPL7NVFVU | nowhere: blind, nominal pose | 0 | 92 | -1.00 | misplaced |  | 4.0 | 6.1 | 32.3 | 0.0 |
| toy_block_6MPL7NVFVU | nowhere: blind, nominal pose | 3 | 92 | -1.00 | dropped_lift | object rose 26 mm | – | – | – | – |
| toy_block_6MPL7NVFVU | nowhere: blind, nominal pose | 4 | 92 | -1.00 | misplaced |  | 0.1 | 0.1 | 29.2 | 0.4 |
| toy_block_6MPL7NVFVU | nowhere: blind, nominal pose | 5 | 92 | -1.00 | misplaced |  | 2.9 | 6.5 | 34.3 | 0.1 |
| toy_block_6MPL7NVFVU | nowhere: blind, nominal pose | 6 | 20 | -1.00 | misplaced |  | 4.3 | 4.5 | 28.0 | 0.4 |
| toy_block_6MPL7NVFVU | nowhere: blind, nominal pose | 8 | 92 | -1.00 | misplaced |  | 1.9 | 2.5 | 29.4 | 0.0 |
| beer_stein_K58BL2P1CR | simulation, from the look pose, no slip correction | 5 | 38 | -0.87 | dropped_transport | object 32 mm from where the grip held it | – | – | – | – |
| beer_stein_K58BL2P1CR | simulation, from the look pose, no slip correction | 8 | 38 | -0.87 | dropped_transport | object 33 mm from where the grip held it | – | – | – | – |
| beer_stein_K58BL2P1CR | simulation, from the look pose, no slip correction | 9 | 38 | -0.87 | dropped_transport | object 36 mm from where the grip held it | – | – | – | – |
| beer_stein_K58BL2P1CR | simulation, no slip correction | 0 | 38 | -0.87 | dropped_transport | object 35 mm from where the grip held it | – | – | – | – |
| beer_stein_K58BL2P1CR | simulation, no slip correction | 5 | 38 | -0.87 | dropped_transport | object 33 mm from where the grip held it | – | – | – | – |
| beer_stein_K58BL2P1CR | simulation, no slip correction | 8 | – | – | unreachable | no motion to any of 34 robust grasps | – | – | – | – |
| jar_2CKZZHE3PV | simulation, no slip correction | 0 | 115 | -0.04 | misplaced |  | 0.0 | 0.1 | 35.4 | 0.5 |
| jar_2CKZZHE3PV | simulation, no slip correction | 1 | 115 | -0.04 | misplaced |  | 0.0 | 0.0 | 44.7 | 0.5 |
| jar_2CKZZHE3PV | simulation, no slip correction | 4 | 115 | -0.04 | misplaced |  | 0.1 | 0.0 | 37.0 | 0.2 |
| jar_2CKZZHE3PV | simulation, no slip correction | 5 | 115 | -0.04 | misplaced |  | 0.2 | 0.2 | 37.3 | 0.3 |
| jar_2CKZZHE3PV | simulation, no slip correction | 7 | 115 | -0.04 | misplaced |  | 0.0 | 0.0 | 35.8 | 0.7 |
| jar_2CKZZHE3PV | simulation, no slip correction | 8 | 115 | -0.04 | misplaced |  | 0.1 | 0.1 | 43.5 | 0.2 |
| lemon_8SUCD6125H | simulation, no slip correction | 5 | – | – | unreachable | 1 of 49 robust grasps ruled out by a later move (approach); no motion to any other | – | – | – | – |
| lemon_8SUCD6125H | simulation, no slip correction | 6 | – | – | unreachable | no motion to any of 49 robust grasps | – | – | – | – |
| mug_A07RDPW843 | simulation, no slip correction | 3 | – | – | unreachable | no motion to any of 28 robust grasps | – | – | – | – |
| toy_block_6MPL7NVFVU | simulation, no slip correction | 6 | 181 | 0.01 | misplaced |  | 6.7 | 8.1 | 22.5 | 0.5 |

## Settings (latest run of each pose source)

camera:

```json
{
 "version": 1,
 "physics_device": "cpu",
 "home_q": [
  0.0,
  -1.2,
  1.4,
  -1.7708,
  -1.5708,
  0.0
 ],
 "pick_xy": [
  0.75,
  -0.35
 ],
 "place_xy": [
  0.75,
  0.35
 ],
 "pre_grasp_m": 0.1,
 "lift_m": 0.15,
 "place_clearance_m": 0.003,
 "tool_tolerance_m": 0.003,
 "place_tolerance_m": 0.02,
 "max_tilt_deg": 10.0,
 "dt": 0.016666666666666666,
 "max_yaw_deg": 90.0,
 "jitter_m": 0.03,
 "seed": 1,
 "speed": 0.5,
 "perception": "camera",
 "depth_noise": "1",
 "handeye": {
  "method": "Park",
  "views": 40,
  "error_mm": 0.4605273755027771,
  "error_deg": 0.23763779562167606
 }
}
```

camera via ROS 2:

```json
{
 "version": 1,
 "physics_device": "cpu",
 "home_q": [
  0.0,
  -1.2,
  1.4,
  -1.7708,
  -1.5708,
  0.0
 ],
 "pick_xy": [
  0.75,
  -0.35
 ],
 "place_xy": [
  0.75,
  0.35
 ],
 "pre_grasp_m": 0.1,
 "lift_m": 0.15,
 "place_clearance_m": 0.003,
 "tool_tolerance_m": 0.003,
 "place_tolerance_m": 0.02,
 "max_tilt_deg": 10.0,
 "dt": 0.016666666666666666,
 "max_yaw_deg": 90.0,
 "jitter_m": 0.03,
 "seed": 1,
 "speed": 0.5,
 "perception": "ros",
 "depth_noise": "1",
 "handeye": {
  "method": "Park",
  "views": 40,
  "error_mm": 0.4605273755027771,
  "error_deg": 0.23763779562167606
 }
}
```

nowhere: blind, nominal pose:

```json
{
 "version": 1,
 "physics_device": "cpu",
 "home_q": [
  0.0,
  -1.2,
  1.4,
  -1.7708,
  -1.5708,
  0.0
 ],
 "pick_xy": [
  0.75,
  -0.35
 ],
 "place_xy": [
  0.75,
  0.35
 ],
 "pre_grasp_m": 0.1,
 "lift_m": 0.15,
 "place_clearance_m": 0.003,
 "tool_tolerance_m": 0.003,
 "place_tolerance_m": 0.02,
 "max_tilt_deg": 10.0,
 "dt": 0.016666666666666666,
 "max_yaw_deg": 90.0,
 "jitter_m": 0.03,
 "seed": 1,
 "speed": 0.5,
 "perception": "blind",
 "slip_correction": false,
 "look_first": true
}
```

simulation:

```json
{
 "version": 1,
 "physics_device": "cpu",
 "home_q": [
  0.0,
  -1.2,
  1.4,
  -1.7708,
  -1.5708,
  0.0
 ],
 "pick_xy": [
  0.75,
  -0.35
 ],
 "place_xy": [
  0.75,
  0.35
 ],
 "pre_grasp_m": 0.1,
 "lift_m": 0.15,
 "place_clearance_m": 0.003,
 "tool_tolerance_m": 0.003,
 "place_tolerance_m": 0.02,
 "max_tilt_deg": 10.0,
 "dt": 0.016666666666666666,
 "max_yaw_deg": 90.0,
 "jitter_m": 0.03,
 "seed": 1,
 "speed": 0.5
}
```

simulation, from the look pose, no slip correction:

```json
{
 "version": 1,
 "physics_device": "cpu",
 "home_q": [
  0.0,
  -1.2,
  1.4,
  -1.7708,
  -1.5708,
  0.0
 ],
 "pick_xy": [
  0.75,
  -0.35
 ],
 "place_xy": [
  0.75,
  0.35
 ],
 "pre_grasp_m": 0.1,
 "lift_m": 0.15,
 "place_clearance_m": 0.003,
 "tool_tolerance_m": 0.003,
 "place_tolerance_m": 0.02,
 "max_tilt_deg": 10.0,
 "dt": 0.016666666666666666,
 "max_yaw_deg": 90.0,
 "jitter_m": 0.03,
 "seed": 1,
 "speed": 0.5,
 "perception": "truth",
 "slip_correction": false,
 "look_first": true
}
```

simulation, no slip correction:

```json
{
 "version": 1,
 "physics_device": "cpu",
 "home_q": [
  0.0,
  -1.2,
  1.4,
  -1.7708,
  -1.5708,
  0.0
 ],
 "pick_xy": [
  0.75,
  -0.35
 ],
 "place_xy": [
  0.75,
  0.35
 ],
 "pre_grasp_m": 0.1,
 "lift_m": 0.15,
 "place_clearance_m": 0.003,
 "tool_tolerance_m": 0.003,
 "place_tolerance_m": 0.02,
 "max_tilt_deg": 10.0,
 "dt": 0.016666666666666666,
 "max_yaw_deg": 90.0,
 "jitter_m": 0.03,
 "seed": 1,
 "speed": 0.5,
 "perception": "truth",
 "slip_correction": false
}
```

