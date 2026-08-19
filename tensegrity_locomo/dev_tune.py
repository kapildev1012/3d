"""
dev_tune.py  (development harness - not part of the experiment)
Run the raw robot on a flat ground with only the base gait and print
COM state so gait parameters can be tuned.
    python3 dev_tune.py [seconds]
"""
import sys
import time

import numpy as np
import pybullet as p

import config
from robot import TensegrityRobot


def main():
    sim_t = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    p.connect(p.DIRECT)
    p.setGravity(*config.GRAVITY)
    p.setPhysicsEngineParameter(fixedTimeStep=config.TIMESTEP, numSubSteps=config.SUBSTEPS)

    # flat test plane
    plane = p.createCollisionShape(p.GEOM_PLANE)
    p.createMultiBody(baseCollisionShapeIndex=plane)
    p.changeDynamics(1, -1, lateralFriction=config.GROUND_LATERAL_FRICTION,
                     spinningFriction=config.GROUND_SPINNING_FRICTION,
                     restitution=config.GROUND_RESTITUTION)

    robot = TensegrityRobot(p, origin=(0.0, 0.0, config.BALL_RADIUS + 0.5))

    n_control = int(round(1.0 / (config.TIMESTEP * config.CONTROL_HZ)))
    com0 = None
    t0 = time.time()
    step = 0
    traj = []
    while step * config.TIMESTEP < sim_t:
        step += 1
        t = step * config.TIMESTEP
        robot.apply_cables(t)
        p.stepSimulation()
        if step % n_control == 0:
            com = robot.get_com()
            vel = robot.get_com_velocity()
            traj.append((t, com[0], com[1], com[2], np.linalg.norm(vel[:2])))
            if com0 is None:
                com0 = np.array(com)
    p.disconnect()

    traj = np.array(traj)
    print("steps:", step, " wall:", round(time.time() - t0, 2), "s")
    print("t       x       y       z      |v|")
    for row in traj[:: max(1, len(traj) // 12)]:
        print(f"{row[0]:6.2f} {row[1]:8.3f} {row[2]:8.3f} {row[3]:8.3f} {row[4]:8.3f}")
    if len(traj):
        print(f"total forward x displacement: {traj[-1,1]:.3f} m in {traj[-1,0]:.1f} s "
              f"(avg {traj[-1,1]/max(traj[-1,0],1e-9):.3f} m/s)")


if __name__ == "__main__":
    main()