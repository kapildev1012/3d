"""dev_shape.py - settle on ground and report the resting shape statistics."""
import sys
import numpy as np
import pybullet as p

import config
from robot import TensegrityRobot

k = float(sys.argv[1]) if len(sys.argv) > 1 else config.CABLE_K
pre = float(sys.argv[2]) if len(sys.argv) > 2 else config.CABLE_PRETENSION
config.CABLE_K, config.CABLE_PRETENSION = k, pre

p.connect(p.DIRECT)
p.setGravity(*config.GRAVITY)
p.setPhysicsEngineParameter(fixedTimeStep=config.TIMESTEP, numSubSteps=config.SUBSTEPS)
plane = p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(baseCollisionShapeIndex=plane)
p.changeDynamics(1, -1, lateralFriction=config.GROUND_LATERAL_FRICTION,
                 spinningFriction=config.GROUND_SPINNING_FRICTION,
                 restitution=config.GROUND_RESTITUTION)

robot = TensegrityRobot(p, origin=(0.0, 0.0, 0.6))
for i in range(240 * 6):
    robot.apply_cables(i * config.TIMESTEP, relax=np.zeros(robot.n_cables))
    p.stepSimulation()

com = robot.get_com()
# node positions = strut endpoints
nodes = []
for ci in range(robot.n_cables):
    pass
# use strut ends instead
ends = []
for si, bid in enumerate(robot.strut_bodies):
    pos, orn = p.getBasePositionAndOrientation(bid)
    R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
    for sgn in (+1, -1):
        ends.append(np.array(pos) + sgn * config.STRUT_LENGTH / 2.0 * R[:, 0])
ends = np.array(ends)
rad = np.linalg.norm(ends - com, axis=1)
print(f"k={k} pret={pre}")
print(f"COM = ({com[0]:.3f}, {com[1]:.3f}, {com[2]:.3f})")
print(f"end radius mean={rad.mean():.3f} max={rad.max():.3f}")
print(f"z extent: min={ends[:,2].min():.3f} max={ends[:,2].max():.3f} "
      f"(com-relative span {ends[:,2].max()-ends[:,2].min():.3f})")
print(f"horizontal radius: mean={np.linalg.norm(ends[:, :2] - com[:2], axis=1).mean():.3f}")
p.disconnect()
