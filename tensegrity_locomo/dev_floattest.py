"""dev_floattest.py - check cable+strut stability in free space (no gravity).
Usage: python3 dev_floattest.py [k] [damp] [pret] [steps] [substeps]"""
import sys
import numpy as np
import pybullet as p

import config
from robot import TensegrityRobot

k = float(sys.argv[1]) if len(sys.argv) > 1 else config.CABLE_K
d = float(sys.argv[2]) if len(sys.argv) > 2 else config.CABLE_DAMP
pre = float(sys.argv[3]) if len(sys.argv) > 3 else config.CABLE_PRETENSION
nsteps = int(sys.argv[4]) if len(sys.argv) > 4 else 240
substeps = int(sys.argv[5]) if len(sys.argv) > 5 else config.SUBSTEPS

config.CABLE_K, config.CABLE_DAMP, config.CABLE_PRETENSION = k, d, pre

p.connect(p.DIRECT)
p.setGravity(0, 0, 0)
p.setPhysicsEngineParameter(fixedTimeStep=config.TIMESTEP, numSubSteps=substeps)

robot = TensegrityRobot(p, origin=(0.0, 0.0, 0.0))

vmax = 0.0
exploded = False
for i in range(nsteps):
    t = i * config.TIMESTEP
    robot.apply_cables(t, relax=np.zeros(robot.n_cables))
    p.stepSimulation()
    for bid in robot.strut_bodies:
        lv, av = p.getBaseVelocity(bid)
        vmax = max(vmax, np.linalg.norm(lv))
        if np.any(np.isnan(lv)) or np.any(np.isnan(av)):
            exploded = True
    if i % 60 == 0:
        com = robot.get_com()
        print(f"t={i*config.TIMESTEP:6.3f} com=({com[0]:7.3f},{com[1]:7.3f},{com[2]:7.3f}) vmax={vmax:7.3f}")
        vmax = 0.0
p.disconnect()
print("EXPLODED" if exploded else "OK")
