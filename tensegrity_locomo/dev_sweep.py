"""
dev_sweep.py  (development harness)
Sweep gait parameters (amp, sign, period) on flat ground and report the
net forward displacement, straightness and energy behaviour.
"""
import sys
import numpy as np
import pybullet as p

import config
from robot import TensegrityRobot


def run(sim_t, amp, sign, period, k_cable=config.CABLE_K, damp=config.CABLE_DAMP):
    p.connect(p.DIRECT)
    p.setGravity(*config.GRAVITY)
    p.setPhysicsEngineParameter(fixedTimeStep=config.TIMESTEP, numSubSteps=config.SUBSTEPS)
    plane = p.createCollisionShape(p.GEOM_PLANE)
    p.createMultiBody(baseCollisionShapeIndex=plane)
    p.changeDynamics(1, -1, lateralFriction=config.GROUND_LATERAL_FRICTION,
                     spinningFriction=config.GROUND_SPINNING_FRICTION,
                     restitution=config.GROUND_RESTITUTION)

    config.GAIT_AMP = amp
    config.GAIT_SIGN = sign
    config.GAIT_PERIOD = period
    config.GAIT_OMEGA = 2.0 * np.pi / period

    robot = TensegrityRobot(p, origin=(0.0, 0.0, config.BALL_RADIUS + 0.4))
    n_control = int(round(1.0 / (config.TIMESTEP * config.CONTROL_HZ)))
    step = 0
    traj = []
    while step * config.TIMESTEP < sim_t:
        step += 1
        t = step * config.TIMESTEP
        robot.apply_cables(t, relax=np.zeros(robot.n_cables))
        p.stepSimulation()
        if step % n_control == 0:
            com = robot.get_com()
            vel = robot.get_com_velocity()
            traj.append((t, com[0], com[1], com[2], np.linalg.norm(vel[:3])))
    p.disconnect()
    traj = np.array(traj)
    dx = traj[-1, 1] - traj[0, 1]
    dy = traj[-1, 2] - traj[0, 2]
    zmax = traj[:, 3].max() if len(traj) else 0
    vmax = traj[:, 4].max() if len(traj) else 0
    return dict(dx=dx, dy=dy, zmax=zmax, vmax=vmax, n=len(traj),
                final=(traj[-1, 1], traj[-1, 2]) if len(traj) else (0, 0))


if __name__ == "__main__":
    sim_t = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    results = []
    for amp in [0.0, 0.015, 0.03, 0.05]:
        for sign in [1.0, -1.0]:
            r = run(sim_t, amp, sign, 1.55)
            results.append((amp, sign, r))
    print(f"{'amp':>6} {'sign':>5} {'dx':>8} {'dy':>8} {'zmax':>7} {'vmax':>7}")
    for amp, sign, r in results:
        print(f"{amp:6.3f} {sign:5.1f} {r['dx']:8.3f} {r['dy']:8.3f} {r['zmax']:7.3f} {r['vmax']:7.3f}")
