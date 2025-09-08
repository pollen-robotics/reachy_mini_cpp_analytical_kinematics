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

T_world_platform = tf.translation_matrix((0, 0, head_z_offset))

kin.reset_forward_kinematics(T_world_platform)
joints = np.array([0.3, 0., 0., 0., 0., 0.])

t0 = time.time()
for k in range(10_000):
    T = kin.forward_kinematics(joints)
t1 = time.time()
print(f"Computation time per call: {((t1-t0)/10_000)*1e6:.3f} us")

T[2, 3] -= head_z_offset
print(T)