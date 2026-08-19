"""
dev_gait.py - gait tuning harness.
Settle the ball, then run a gait and measure displacement.
Usage: python3 dev_gait.py [mode] [amp] [steer] [sim_t] [period]
modes: default | wave | lean_back | lean_pulse
"""
import sys
import numpy as np
import pybullet as p

import config
from robot import TensegrityRobot

mode = sys.argv[1] if len(sys.argv) > 1 else "default"
amp = float(sys.argv[2]) if len(sys.argv) > 2 else config.GAIT_AMP
steer = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
sim_t = float(sys.argv[4]) if len(sys.argv) > 4 else 15.0
period = float(sys.argv[5]) if len(sys.argv) > 5 else config.GAIT_PERIOD

config.GAIT_AMP = amp
config.GAIT_PERIOD = period
config.GAIT_OMEGA = 2.0 * np.pi / period

p.connect(p.DIRECT)
p.setGravity(*config.GRAVITY)
p.setPhysicsEngineParameter(fixedTimeStep=config.TIMESTEP, numSubSteps=config.SUBSTEPS)
plane = p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(baseCollisionShapeIndex=plane)
p.changeDynamics(1, -1, lateralFriction=config.GROUND_LATERAL_FRICTION,
                 spinningFriction=config.GROUND_SPINNING_FRICTION,
                 restitution=config.GROUND_RESTITUTION)

robot = TensegrityRobot(p, origin=(0.0, 0.0, config.BALL_RADIUS + 0.05))

def gait_mods(t):
    if mode == "default":
        return None
    if mode == "wave":
        return config.GAIT_SIGN * amp * np.cos(config.GAIT_OMEGA * t - robot.cable_phase)
    if mode == "lean_back":
        return +amp * robot.px_norm
    if mode == "lean_pulse":
        ph = np.sign(np.cos(config.GAIT_OMEGA * t))
        return -amp * robot.px_norm * ph
    raise ValueError(mode)

for i in range(240):
    robot.apply_cables(0.0, relax=np.zeros(robot.n_cables), mods=np.zeros(robot.n_cables))
    p.stepSimulation()

start = robot.get_com()
traj = []
step = 0
while step * config.TIMESTEP < sim_t:
    step += 1
    t = step * config.TIMESTEP
    robot.apply_cables(t, relax=np.zeros(robot.n_cables), mods=gait_mods(t), steer=steer)
    p.stepSimulation()
    if step % 15 == 0:
        com = robot.get_com()
        vel = robot.get_com_velocity()
        traj.append((t, com[0], com[1], com[2], np.linalg.norm(vel)))
p.disconnect()

traj = np.array(traj)
dx = traj[-1, 1] - start[0]
dy = traj[-1, 2] - start[1]
zmax = traj[:, 3].max()
print(f"mode={mode} amp={amp} steer={steer} | dx={dx:8.3f} dy={dy:8.3f} "
      f"speed_x={dx/max(traj[-1,0],1e-9):6.3f} |dy/dx|={abs(dy)/max(abs(dx),1e-6):5.3f} zmax={zmax:6.3f}")
