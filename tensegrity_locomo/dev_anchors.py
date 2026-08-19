"""dev_anchors.py - verify cable anchors land on the intended vertices."""
import numpy as np
import pybullet as p

import config
from robot import TensegrityRobot

p.connect(p.DIRECT)
robot = TensegrityRobot(p, origin=(0.0, 0.0, 0.0))

for ci in range(6):
    sa, la = robot.cable_body_a[ci], robot.cable_anchor_a[ci]
    sb, lb = robot.cable_body_b[ci], robot.cable_anchor_b[ci]
    pa, oa = p.getBasePositionAndOrientation(robot.strut_bodies[sa])
    pb, ob = p.getBasePositionAndOrientation(robot.strut_bodies[sb])
    Ra = np.array(p.getMatrixFromQuaternion(oa)).reshape(3, 3)
    Rb = np.array(p.getMatrixFromQuaternion(ob)).reshape(3, 3)
    wa = np.array(pa) + Ra @ np.array(la)
    wb = np.array(pb) + Rb @ np.array(lb)
    ia, ib = robot.cables[ci]
    va = robot.verts[ia]
    vb = robot.verts[ib]
    print(f"cable {ci}: nodes ({ia},{ib})")
    print(f"   anchorA {wa.round(3)}  vs vertexA {va.round(3)}  err={np.linalg.norm(wa-va):.4f}")
    print(f"   anchorB {wb.round(3)}  vs vertexB {vb.round(3)}  err={np.linalg.norm(wb-vb):.4f}")
p.disconnect()
