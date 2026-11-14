# Perception results

The object's pose from one wrist-camera depth image (pick_place/perception.py), against the true pose,
on the captures of capture_views.py: the start poses of the pick-and-place cycles, seen from the look pose.
*Centre*: distance between the model's centroid as estimated and as true. *ADD-S*: mean distance from each
estimated model point to the nearest true one (near zero for a pose a symmetric object cannot tell from
the true one); *ADD*: mean distance between corresponding model points. *Depth*: the renderer's perfect
depth, or the simulated stereo depth at 1 or 2 times camera_model's noise. Unless marked *calibrated
mount*, the camera's pose is the true one; with it, the robot uses the hand-eye calibration's mount, as
it does in the cell. *Wall s*: time of perception.locate() (offline, on this PC).

| Object | Depth | Found | Centre mm (median / max) | Rotation deg (median) | ADD-S mm (median / max) | ADD mm (median) | Object points | Wall s |
|---|---|---|---|---|---|---|---|---|
| beer_stein_K58BL2P1CR | perfect | 10/10 | 0.1 / 0.1 | 0.1 | 0.1 / 0.2 | 0.1 | 1864 | 3.90 |
| beer_stein_K58BL2P1CR | stereo x1 | 10/10 | 0.4 / 0.4 | 0.6 | 0.4 / 0.6 | 0.4 | 1508 | 3.89 |
| beer_stein_K58BL2P1CR | stereo x1, calibrated mount | 10/10 | 0.6 / 0.7 | 0.6 | 0.7 / 0.8 | 0.7 | 1509 | 3.92 |
| beer_stein_K58BL2P1CR | stereo x2 | 10/10 | 1.0 / 1.5 | 1.4 | 1.1 / 1.4 | 1.3 | 1496 | 3.97 |
| beer_stein_K58BL2P1CR | stereo x2, calibrated mount | 10/10 | 1.2 / 1.9 | 1.6 | 1.2 / 1.4 | 1.5 | 1491 | 3.93 |
| beer_stein_K58BL2P1CR | perfect, calibrated mount | 10/10 | 0.3 / 0.4 | 0.3 | 0.3 / 0.4 | 0.3 | 1864 | 3.90 |
| jar_2CKZZHE3PV | perfect | 10/10 | 0.1 / 0.5 | 15.3 | 1.0 / 1.1 | 9.3 | 3189 | 5.02 |
| jar_2CKZZHE3PV | stereo x1 | 10/10 | 0.5 / 0.8 | 111.7 | 1.1 / 1.1 | 57.6 | 2633 | 4.92 |
| jar_2CKZZHE3PV | stereo x1, calibrated mount | 10/10 | 0.7 / 0.8 | 129.3 | 1.1 / 1.1 | 62.8 | 2633 | 4.93 |
| jar_2CKZZHE3PV | stereo x2 | 10/10 | 0.5 / 1.0 | 55.9 | 1.1 / 1.2 | 32.5 | 2633 | 5.03 |
| jar_2CKZZHE3PV | stereo x2, calibrated mount | 10/10 | 0.7 / 1.7 | 66.0 | 1.1 / 1.4 | 37.0 | 2633 | 4.96 |
| jar_2CKZZHE3PV | perfect, calibrated mount | 10/10 | 0.5 / 0.7 | 88.3 | 1.1 / 1.1 | 48.5 | 3189 | 4.98 |
| lemon_8SUCD6125H | perfect | 10/10 | 0.0 / 0.8 | 0.2 | 0.1 / 1.6 | 0.1 | 3140 | 3.48 |
| lemon_8SUCD6125H | stereo x1 | 10/10 | 0.3 / 0.7 | 1.0 | 0.6 / 1.4 | 0.7 | 2850 | 3.49 |
| lemon_8SUCD6125H | stereo x1, calibrated mount | 10/10 | 0.5 / 0.8 | 0.9 | 0.6 / 1.4 | 0.7 | 2850 | 3.49 |
| lemon_8SUCD6125H | stereo x2 | 10/10 | 0.8 / 1.2 | 3.0 | 1.2 / 1.7 | 2.1 | 2850 | 3.56 |
| lemon_8SUCD6125H | stereo x2, calibrated mount | 10/10 | 0.9 / 1.4 | 3.0 | 1.2 / 1.6 | 2.0 | 2850 | 3.53 |
| lemon_8SUCD6125H | perfect, calibrated mount | 10/10 | 0.4 / 0.9 | 13.8 | 0.8 / 1.6 | 7.8 | 3140 | 3.45 |
| mug_A07RDPW843 | perfect | 10/10 | 0.0 / 0.0 | 0.0 | 0.0 / 0.1 | 0.0 | 4434 | 7.01 |
| mug_A07RDPW843 | stereo x1 | 10/10 | 0.3 / 0.8 | 0.6 | 0.5 / 0.9 | 0.5 | 3724 | 6.98 |
| mug_A07RDPW843 | stereo x1, calibrated mount | 10/10 | 0.4 / 1.1 | 0.7 | 0.6 / 1.0 | 0.6 | 3724 | 6.95 |
| mug_A07RDPW843 | stereo x2 | 10/10 | 0.6 / 1.7 | 1.3 | 0.9 / 1.4 | 1.0 | 3713 | 7.04 |
| mug_A07RDPW843 | stereo x2, calibrated mount | 10/10 | 0.7 / 2.1 | 1.4 | 1.0 / 1.5 | 1.3 | 3715 | 7.20 |
| mug_A07RDPW843 | perfect, calibrated mount | 10/10 | 0.3 / 0.4 | 0.3 | 0.4 / 0.4 | 0.4 | 4434 | 7.00 |
| toy_block_6MPL7NVFVU | perfect | 10/10 | 0.0 / 0.1 | 0.0 | 0.1 / 0.1 | 0.1 | 2350 | 2.81 |
| toy_block_6MPL7NVFVU | stereo x1 | 10/10 | 0.4 / 1.1 | 0.6 | 0.5 / 1.0 | 0.5 | 2121 | 2.81 |
| toy_block_6MPL7NVFVU | stereo x1, calibrated mount | 10/10 | 0.7 / 1.3 | 0.6 | 0.7 / 1.1 | 0.7 | 2121 | 2.77 |
| toy_block_6MPL7NVFVU | stereo x2 | 10/10 | 0.6 / 2.4 | 1.4 | 1.0 / 1.6 | 1.2 | 2126 | 2.83 |
| toy_block_6MPL7NVFVU | stereo x2, calibrated mount | 10/10 | 0.8 / 2.5 | 1.2 | 1.0 / 1.7 | 1.1 | 2127 | 2.79 |
| toy_block_6MPL7NVFVU | perfect, calibrated mount | 10/10 | 0.3 / 0.4 | 0.2 | 0.3 / 0.4 | 0.3 | 2350 | 2.78 |
