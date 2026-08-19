"""dev_orient.py - inspect what pybullet actually returns for body orientation."""
import numpy as np
import pybullet as p

import config
from robot import TensegrityRobot, _expanded_hexagon_nodes

p.connect(p.DIRECT)
robot = TensegrityRobot(p, origin=(0.0, 0.0, 0.0))

verts, bars = _expanded_hexagon_nodes()
i0, i1 = bars[0]
a, b = verts[i0], verts[i1]
mid = (a + b) / 2
axis = b - a
axis = axis / np.linalg.norm(axis)
q_expected = robot._rot_local_x_to(axis)
print("strut 0: a=", a.round(4), "b=", b.round(4))
print("axis=", axis.round(4))
print("q_expected=", np.array(q_expected).round(4))

pos, orn = p.getBasePositionAndOrientation(robot.strut_bodies[0])
print("basePos=", np.array(pos).round(4))
print("baseOrn=", np.array(orn).round(4))
print("q_expected vs baseOrn equal:", np.allclose(q_expected, orn))

M = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
print("reshape(3,3) raw:\n", M.round(4))
print("col norms:", np.linalg.norm(M, axis=0).round(4))
print("with .T norms:", np.linalg.norm(M.T, axis=0).round(4))
# anchor candidates:
for label, R in [("no.T", M), ("with.T", M.T)]:
    wa = np.array(pos) + R @ np.array([config.STRUT_LENGTH/2, 0, 0])
    print(f"{label}: +x end -> {wa.round(4)}  (b={b.round(4)})")
p.disconnect()
