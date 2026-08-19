"""dev_topology.py - empirically test candidate tensegrity topologies.
Usage: python3 dev_topology.py [gravity] [pret]
Builds the cuboctahedron (12 nodes, 6 diameter bars, 24 edge cables) and
reports the settled shape."""
import sys
import numpy as np
import pybullet as p

import config

gravity = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
pre = float(sys.argv[2]) if len(sys.argv) > 2 else 0.02

# ---- cuboctahedron nodes (edge midpoints of a regular octahedron) ----
def cuboctahedron_nodes(radius=0.5):
    # octahedron edge midpoints
    nodes = []
    for u in [(1, 0, 0), (-1, 0, 0)]:
        for w in [(0, 1, 0), (0, -1, 0)]:
            nodes.append(tuple(0.5 * (np.array(u) + np.array(w))))
    for u in [(1, 0, 0), (-1, 0, 0)]:
        for w in [(0, 0, 1), (0, 0, -1)]:
            nodes.append(tuple(0.5 * (np.array(u) + np.array(w))))
    for u in [(0, 1, 0), (0, -1, 0)]:
        for w in [(0, 0, 1), (0, 0, -1)]:
            nodes.append(tuple(0.5 * (np.array(u) + np.array(w))))
    nodes = np.array(nodes)
    nodes = radius * nodes / np.linalg.norm(nodes[0])
    return nodes

nodes = cuboctahedron_nodes(config.STRUT_LENGTH / 2.0)
n = len(nodes)
bars = []
seen = set()
for i in range(n):
    for j in range(n):
        if j <= i:
            continue
        if np.linalg.norm(nodes[i] + nodes[j]) < 1e-9:  # antipodal pair
            bars.append((i, j))
cables = []
for i in range(n):
    for j in range(i + 1, n):
        if (i, j) in bars:
            continue
        d = np.linalg.norm(nodes[i] - nodes[j])
        if abs(d - np.linalg.norm(nodes[0] - nodes[4])) < 1e-6:  # edges
            cables.append((i, j))

print("nodes", n, "bars", len(bars), "cables", len(cables))

p.connect(p.DIRECT)
p.setGravity(0, 0, gravity)
p.setPhysicsEngineParameter(fixedTimeStep=config.TIMESTEP, numSubSteps=config.SUBSTEPS)
if gravity != 0.0:
    plane = p.createCollisionShape(p.GEOM_PLANE)
    p.createMultiBody(baseCollisionShapeIndex=plane)

# build struts
spawn_z = 1.0 if gravity != 0.0 else 0.0
bodies = []
for (i, j) in bars:
    a, b = nodes[i], nodes[j]
    mid = (a + b) / 2
    mid[2] += spawn_z
    axis = b - a
    axis /= np.linalg.norm(axis)
    # rotation local x -> axis
    x = np.array([1.0, 0.0, 0.0])
    d = np.dot(x, axis)
    if d < -0.9999:
        q = [0, 0, 1, 0]
    else:
        w = 1.0 + d
        cr = np.cross(x, axis)
        qv = np.array([cr[0], cr[1], cr[2], w])
        q = qv / np.linalg.norm(qv)
    cs = p.createCollisionShape(p.GEOM_CYLINDER, radius=config.STRUT_RADIUS,
                                height=config.STRUT_LENGTH, collisionFrameOrientation=q)
    vs = p.createVisualShape(p.GEOM_CYLINDER, radius=config.STRUT_RADIUS,
                             length=config.STRUT_LENGTH, visualFrameOrientation=q,
                             rgbaColor=[0.78, 0.35, 0.22, 1])
    bd = p.createMultiBody(baseMass=config.STRUT_MASS, baseCollisionShapeIndex=cs,
                           baseVisualShapeIndex=vs, basePosition=mid, baseOrientation=q)
    p.changeDynamics(bd, -1, lateralFriction=config.STRUT_FRICTION,
                     spinningFriction=0.4, rollingFriction=0.15,
                     restitution=config.STRUT_RESTITUTION,
                     linearDamping=0.02, angularDamping=0.02)
    bodies.append(bd)
for i in range(len(bodies)):
    for j in range(i + 1, len(bodies)):
        p.setCollisionFilterPair(bodies[i], bodies[j], -1, -1, enableCollision=0)

# anchor mapping
vertex_local = {}
for si, (ii, jj) in enumerate(bars):
    vertex_local[ii] = (si, [-config.STRUT_LENGTH / 2.0, 0, 0])
    vertex_local[jj] = (si, [config.STRUT_LENGTH / 2.0, 0, 0])

rest = np.array([np.linalg.norm(nodes[a] - nodes[b]) for (a, b) in cables]) * (1 - pre)
ca, cb = zip(*cables)
ba = np.array([vertex_local[a][0] for a in ca])
bb = np.array([vertex_local[b][0] for b in cb])
la = np.array([vertex_local[a][1] for a in ca])
lb = np.array([vertex_local[b][1] for b in cb])

def apply():
    states = {si: p.getBasePositionAndOrientation(bodies[si]) for si in range(6)}
    def to_world(si, local):
        pos, orn = states[si]
        R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        return np.array(pos) + R @ np.array(local)
    for ci in range(len(cables)):
        pa = to_world(ba[ci], la[ci]); pb = to_world(bb[ci], lb[ci])
        dvec = pb - pa; L = np.linalg.norm(dvec)
        if L < 1e-9: continue
        diru = dvec / L
        aL = L - rest[ci]
        if aL <= 0: continue
        lva, _ = p.getBaseVelocity(bodies[ba[ci]])
        lvb, _ = p.getBaseVelocity(bodies[bb[ci]])
        vn = np.dot(np.array(lvb) - np.array(lva), diru)
        F = config.CABLE_K * aL + config.CABLE_DAMP * max(0.0, vn)
        if F <= 0: continue
        f = F * diru
        p.applyExternalForce(bodies[ba[ci]], -1, f.tolist(), pa.tolist(), p.WORLD_FRAME)
        p.applyExternalForce(bodies[bb[ci]], -1, (-f).tolist(), pb.tolist(), p.WORLD_FRAME)

vmax = 0
for i in range(240 * 8):
    apply()
    p.stepSimulation()
    for bd in bodies:
        lv, _ = p.getBaseVelocity(bd)
        vmax = max(vmax, np.linalg.norm(lv))
    if i % 480 == 0:
        com = np.mean([np.array(p.getBasePositionAndOrientation(bd)[0]) for bd in bodies], axis=0)
        ends = []
        for si, bd in enumerate(bodies):
            pos, orn = p.getBasePositionAndOrientation(bd)
            R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
            for sgn in (1, -1):
                ends.append(np.array(pos) + sgn * config.STRUT_LENGTH / 2.0 * R[:, 0])
        ends = np.array(ends)
        print(f"t={i*config.TIMESTEP:6.2f} com=({com[0]:.2f},{com[1]:.2f},{com[2]:.3f}) "
              f"vmax={vmax:6.2f} zspan={(ends[:,2].max()-ends[:,2].min()):.3f} "
              f"radius={np.linalg.norm(ends-com,axis=1).mean():.3f}")
        vmax = 0
p.disconnect()
