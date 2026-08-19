"""dev_diag.py  - diagnose the raw cable/structure behaviour at spawn.
Usage: python3 dev_diag.py [k_cable] [damp] [pret] [nocables] [steps]
"""
import sys
import numpy as np
import pybullet as p

import config
from robot import TensegrityRobot

k = float(sys.argv[1]) if len(sys.argv) > 1 else config.CABLE_K
d = float(sys.argv[2]) if len(sys.argv) > 2 else config.CABLE_DAMP
pre = float(sys.argv[3]) if len(sys.argv) > 3 else config.CABLE_PRETENSION
nocables = len(sys.argv) > 4 and sys.argv[4] == "nocables"
nsteps = int(sys.argv[5]) if len(sys.argv) > 5 else 240

config.CABLE_K, config.CABLE_DAMP, config.CABLE_PRETENSION = k, d, pre

p.connect(p.DIRECT)
p.setGravity(*config.GRAVITY)
p.setPhysicsEngineParameter(fixedTimeStep=config.TIMESTEP, numSubSteps=config.SUBSTEPS)
plane = p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(baseCollisionShapeIndex=plane)

robot = TensegrityRobot(p, origin=(0.0, 0.0, 0.6))

lens = robot.cable_lengths()
print(f"k={k} damp={d} pret={pre} nocables={nocables} | "
      f"rest {robot.cable_rest_nominal.min():.4f} spawn len {lens.min():.4f} "
      f"overshoot {(lens - robot.cable_rest_nominal).mean():.4f}")

vmax = 0.0
for i in range(nsteps):
    t = i * config.TIMESTEP
    if not nocables:
        robot.apply_cables(t, relax=np.zeros(robot.n_cables))
    p.stepSimulation()
    for bid in robot.strut_bodies:
        lv, av = p.getBaseVelocity(bid)
        vmax = max(vmax, np.linalg.norm(lv))
        if np.any(np.isnan(lv)) or np.any(np.isnan(av)):
            print("NAN velocity at step", i, bid)
            p.disconnect()
            raise SystemExit(1)
    if i % 60 == 0:
        com = robot.get_com()
        print(f"t={i*config.TIMESTEP:6.3f} com=({com[0]:7.3f},{com[1]:7.3f},{com[2]:7.3f}) vmax={vmax:7.3f}")
        vmax = 0.0
p.disconnect()
