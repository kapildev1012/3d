"""
dev_steer.py - closed-loop heading control test on flat ground.
The ball rolls with the default lean gait; a PD controller on lateral
position and lateral velocity drives `steer` to hold the corridor y=0.
Usage: python3 dev_steer.py [Kp] [Kd] [sim_t]
"""
import sys
import numpy as np
import pybullet as p

import config
from robot import TensegrityRobot

Kp = float(sys.argv[1]) if len(sys.argv) > 1 else 1.2
Kd = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
sim_t = float(sys.argv[3]) if len(sys.argv) > 3 else 25.0

p.connect(p.DIRECT)
p.setGravity(*config.GRAVITY)
p.setPhysicsEngineParameter(fixedTimeStep=config.TIMESTEP, numSubSteps=config.SUBSTEPS)
plane = p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(baseCollisionShapeIndex=plane)
p.changeDynamics(1, -1, lateralFriction=config.GROUND_LATERAL_FRICTION,
                 spinningFriction=config.GROUND_SPINNING_FRICTION,
                 restitution=config.GROUND_RESTITUTION)

robot = TensegrityRobot(p, origin=(0.0, 0.0, config.BALL_RADIUS + 0.05))
for i in range(240):
    robot.apply_cables(0.0, relax=np.zeros(robot.n_cables), mods=np.zeros(robot.n_cables))
    p.stepSimulation()

step = 0
y_prev = 0.0
traj = []
while step * config.TIMESTEP < sim_t:
    step += 1
    t = step * config.TIMESTEP
    com = robot.get_com()
    vel = robot.get_com_velocity()
    y_err = com[1]
    vy = vel[1]
    steer = np.clip(-Kp * y_err - Kd * vy, -config.STEER_SAT, config.STEER_SAT)
    robot.apply_cables(t, relax=np.zeros(robot.n_cables), steer=steer)
    p.stepSimulation()
    if step % 15 == 0:
        com = robot.get_com()
        vel = robot.get_com_velocity()
        traj.append((t, com[0], com[1], com[2], vel[0], vel[1]))
p.disconnect()

traj = np.array(traj)
dx = traj[-1, 1]
max_y = np.abs(traj[:, 2]).max()
print(f"Kp={Kp} Kd={Kd} | dx={dx:8.3f} max|y|={max_y:6.3f} "
      f"speed={dx/max(traj[-1,0],1e-9):5.3f} final_y={traj[-1,2]:6.3f}")
