import time
import json
import numpy as np
import reachy_mini_kinematics as rk
import meshcat.transformations as tf

kin = rk.Kinematics(0.038, 0.09)
head_z_offset = 0.177

with open("motors.json", "r") as f:
    motors = json.load(f)

for motor in motors:
    kin.add_branch(
        np.array(motor["branch_position"]),
        np.linalg.inv(motor["T_motor_world"]),
        1 if motor["solution"] else -1,
    )

T_world_platform = tf.translation_matrix((0, 0, 0.177))
t0 = time.time()
for k in range(1_000):
    r = kin.inverse_kinematics(T_world_platform)
t1 = time.time()
print(r)
print(f"Total time (s): {t1 - t0:.2f}")
print(f"Time for ik (us): {(t1 - t0) * 1e6 / 1_000:.2f}")
