"""
robot.py
--------
The 6-strut tensegrity "super ball" used by BOTH models.

Topology (expanded-hexagon / Super-Ball-Bot topology)
-----------------------------------------------------
* 12 nodes arranged in two staggered hexagons (a top ring at z=+h and a
  bottom ring at z=-h, twisted 60 degrees relative to each other).
* 6 rigid struts (bars) connect the rings tangentially - each bar spans
  from a bottom ring node to a top ring node; NO bar passes through the
  centre of the ball, which is what makes this a *stable* tensegrity
  equilibrium (unlike a singular all-bars-through-centre arrangement).
* 24 tension-only cables connect each node to its nearest neighbours
  (top ring, bottom ring and cross ring), so the ball is held together
  purely by tension (cables) and compression (bars).

Physics model (PyBullet)
------------------------
* Each strut is a free rigid cylinder (mass, inertia, full 6-DoF).
* Cables are NOT rigid bodies: they are tension-only spring+damper forces
  applied to the two strut endpoints each physics step.  Rest length can be
  modulated by the base gait and by the adaptive relaxation policy.
* Strut-strut collisions are disabled; strut-ground / strut-obstacle
  collisions are enabled.
"""

import numpy as np
import pybullet as p

import config


def _cuboctahedron_nodes(radius):
    """12 nodes = edge midpoints of a regular octahedron (a cuboctahedron).
    This is the classic *expanded-octahedron* tensegrity node set: each
    node has degree 4 in the cable graph + 1 bar = 5 members, and the
    configuration carries a genuine self-stress, so it is a stable ball."""
    edge_dirs = []
    for u in [(1, 0, 0), (-1, 0, 0)]:
        for w in [(0, 1, 0), (0, -1, 0)]:
            edge_dirs.append(0.5 * (np.array(u) + np.array(w)))
    for u in [(1, 0, 0), (-1, 0, 0)]:
        for w in [(0, 0, 1), (0, 0, -1)]:
            edge_dirs.append(0.5 * (np.array(u) + np.array(w)))
    for u in [(0, 1, 0), (0, -1, 0)]:
        for w in [(0, 0, 1), (0, 0, -1)]:
            edge_dirs.append(0.5 * (np.array(u) + np.array(w)))
    nodes = np.array(edge_dirs)
    nodes = radius * nodes / np.linalg.norm(nodes[0])
    return nodes


def _expanded_octahedron_members(verts):
    """Return (bars, cables): 6 antipodal bar pairs + 24 cuboctahedron edges."""
    n = verts.shape[0]
    bars = []
    for i in range(n):
        for j in range(i + 1, n):
            if np.linalg.norm(verts[i] + verts[j]) < 1e-9:
                bars.append((i, j))
    edge_len = np.linalg.norm(verts[0] - verts[4])
    cables = []
    for i in range(n):
        for j in range(i + 1, n):
            if (i, j) in bars:
                continue
            if abs(np.linalg.norm(verts[i] - verts[j]) - edge_len) < 1e-6:
                cables.append((i, j))
    return bars, cables


class TensegrityRobot:
    def __init__(self, pid, origin=(0.0, 0.0, 0.0)):
        self.pid = pid
        self.p = pid
        self.origin = np.array(origin, dtype=float)

        verts0 = _cuboctahedron_nodes(config.BALL_RADIUS)
        self.strut_pairs, self.cables = _expanded_octahedron_members(verts0)
        verts = verts0 + self.origin
        self.verts = verts

        # ---- build strut rigid bodies ----
        self.strut_bodies = []
        for (i, j) in self.strut_pairs:
            a, b = verts[i], verts[j]
            mid = (a + b) / 2.0
            axis = b - a
            axis = axis / np.linalg.norm(axis)
            q = self._rot_local_x_to(axis)
            cs = p.createCollisionShape(p.GEOM_CYLINDER,
                                        radius=config.STRUT_RADIUS,
                                        height=config.STRUT_LENGTH,
                                        collisionFrameOrientation=q)
            vs = p.createVisualShape(p.GEOM_CYLINDER,
                                     radius=config.STRUT_RADIUS,
                                     length=config.STRUT_LENGTH,
                                     visualFrameOrientation=q,
                                     rgbaColor=[0.78, 0.35, 0.22, 1.0])
            body = p.createMultiBody(
                baseMass=config.STRUT_MASS,
                baseCollisionShapeIndex=cs,
                baseVisualShapeIndex=vs,
                basePosition=mid.astype(float),
                baseOrientation=q,
            )
            p.changeDynamics(body, -1,
                             lateralFriction=config.STRUT_FRICTION,
                             spinningFriction=0.4,
                             rollingFriction=0.15,
                             restitution=config.STRUT_RESTITUTION,
                             linearDamping=0.05, angularDamping=0.05)
            self.strut_bodies.append(body)

        # disable strut-strut collisions
        for ii in range(len(self.strut_bodies)):
            for jj in range(ii + 1, len(self.strut_bodies)):
                p.setCollisionFilterPair(self.strut_bodies[ii], self.strut_bodies[jj],
                                         -1, -1, enableCollision=0)

        # ---- cable bookkeeping ----
        self.n_cables = len(self.cables)
        self.cable_rest_nominal = np.zeros(self.n_cables)
        self.cable_phase = np.zeros(self.n_cables)          # around +x roll axis
        self.cable_body_a = np.zeros(self.n_cables, dtype=int)
        self.cable_body_b = np.zeros(self.n_cables, dtype=int)
        self.cable_anchor_a = np.zeros((self.n_cables, 3))
        self.cable_anchor_b = np.zeros((self.n_cables, 3))

        # vertex index -> (strut body idx, local anchor).  Local +x points
        # from vertex `i` (its -x end) to vertex `j` (its +x end).
        vertex_local = {}
        for si, (ii, jj) in enumerate(self.strut_pairs):
            vertex_local[ii] = (si, [-config.STRUT_LENGTH / 2.0, 0.0, 0.0])
            vertex_local[jj] = (si, [config.STRUT_LENGTH / 2.0, 0.0, 0.0])

        for ci, (ai, bj) in enumerate(self.cables):
            va, vb = verts[ai], verts[bj]
            chord = np.linalg.norm(vb - va)
            self.cable_rest_nominal[ci] = chord * (1.0 - config.CABLE_PRETENSION)
            self.cable_body_a[ci], self.cable_anchor_a[ci] = vertex_local[ai]
            self.cable_body_b[ci], self.cable_anchor_b[ci] = vertex_local[bj]

        # body-frame forward/lateral bias per cable (for the lean gait)
        mids = np.array([(self.verts[self.cables[c][0]] + self.verts[self.cables[c][1]]) / 2.0
                         for c in range(self.n_cables)])
        rel = mids - self.origin
        R = config.BALL_RADIUS
        self.px_norm = np.clip(rel[:, 0] / R, -1.0, 1.0)   # +1 = front of ball
        self.py_norm = np.clip(rel[:, 1] / R, -1.0, 1.0)   # +1 = right side (+y)

        # body-frame angular phase of each cable around the Y (lateral) axis;
        # a wave travelling in this phase rolls the ball forward (+x).
        self.cable_phase = np.arctan2(rel[:, 0], rel[:, 2])

        self.gait_relax = np.zeros(self.n_cables)   # extra relaxation factor

    # ------------------------------------------------------------------
    # ROTATION HELPER
    # ------------------------------------------------------------------
    @staticmethod
    def _rot_local_x_to(axis):
        """Quaternion rotating local +x onto a normalised world `axis`."""
        x = np.array([1.0, 0.0, 0.0])
        axis = np.asarray(axis, dtype=float)
        axis = axis / (np.linalg.norm(axis) + 1e-12)
        d = float(np.dot(x, axis))
        if d < -0.9999:                       # 180 deg flip about y
            return [0.0, 0.0, 1.0, 0.0]
        w = 1.0 + d
        cross = np.cross(x, axis)
        q = np.array([cross[0], cross[1], cross[2], w])
        return (q / np.linalg.norm(q)).astype(float).tolist()
# ------------------------------------------------------------------
    # CABLE SOLVING  (call every physics step)
    # ------------------------------------------------------------------
    def apply_cables(self, t, relax=None, steer=0.0, mods=None):
        """Apply tension-only cable forces.

        Base gait (identical for both models): a WORLD-frame "lean" --
        cables ahead of the centre of mass are shortened, cables behind are
        lengthened, so the ball keeps rolling toward world +x regardless of
        its own rotation.  `steer` biases the lean laterally toward +y.
        relax: optional array (n_cables) of additional relative relaxation.
        Returns the per-cable tension vector.
        """
        states = {}
        for si, bid in enumerate(self.strut_bodies):
            states[si] = p.getBasePositionAndOrientation(bid)

        def to_world(si, local):
            pos, orn = states[si]
            R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
            return np.array(pos) + R @ np.array(local)

        if mods is None:
            com = np.mean([np.array(states[si][0]) for si in states], axis=0)
            world_mids = np.zeros((self.n_cables, 3))
            for ci in range(self.n_cables):
                pa = to_world(self.cable_body_a[ci], self.cable_anchor_a[ci])
                pb = to_world(self.cable_body_b[ci], self.cable_anchor_b[ci])
                world_mids[ci] = 0.5 * (pa + pb)
            rel = world_mids - com
            Rscale = config.BALL_RADIUS
            fwd = np.clip(rel[:, 0] / Rscale, -1.0, 1.0)
            lat = np.clip(rel[:, 1] / Rscale, -1.0, 1.0)
            mods = config.GAIT_SIGN * config.GAIT_AMP * fwd
            if steer != 0.0:
                steer = np.clip(steer, -config.STEER_SAT, config.STEER_SAT)
                mods = mods + steer * config.STEER_GAIN * lat
        if relax is not None:
            self.gait_relax = np.clip(np.asarray(relax, dtype=float), 0.0, config.RELAX_MAX)

        rest = self.cable_rest_nominal * (1.0 + mods + self.gait_relax)

        states = {}
        for si, bid in enumerate(self.strut_bodies):
            states[si] = p.getBasePositionAndOrientation(bid)

        def to_world(si, local):
            pos, orn = states[si]
            R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
            return np.array(pos) + R @ np.array(local)

        tensions = np.zeros(self.n_cables)
        for ci in range(self.n_cables):
            pa = to_world(self.cable_body_a[ci], self.cable_anchor_a[ci])
            pb = to_world(self.cable_body_b[ci], self.cable_anchor_b[ci])
            dvec = pb - pa
            L = np.linalg.norm(dvec)
            if L < 1e-9:
                continue
            diru = dvec / L
            aL = L - rest[ci]
            if aL <= 0.0:
                continue                        # tension-only: no pushing
            body_a = self.strut_bodies[self.cable_body_a[ci]]
            body_b = self.strut_bodies[self.cable_body_b[ci]]
            lva, _ = p.getBaseVelocity(body_a)
            lvb, _ = p.getBaseVelocity(body_b)
            vn = np.dot(np.array(lvb) - np.array(lva), diru)
            F = config.CABLE_K * aL + config.CABLE_DAMP * max(0.0, vn)
            if F <= 0.0:
                continue
            tensions[ci] = F
            fvec = F * diru
            p.applyExternalForce(body_a, -1, fvec.astype(float).tolist(),
                                 pa.astype(float).tolist(), p.WORLD_FRAME)
            p.applyExternalForce(body_b, -1, (-fvec).astype(float).tolist(),
                                 pb.astype(float).tolist(), p.WORLD_FRAME)
        return tensions

    def cable_lengths(self):
        """Current endpoint distance per cable (metrics / visualisation)."""
        states = {}
        for si, bid in enumerate(self.strut_bodies):
            states[si] = p.getBasePositionAndOrientation(bid)

        def to_world(si, local):
            pos, orn = states[si]
            R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
            return np.array(pos) + R @ np.array(local)

        lens = np.zeros(self.n_cables)
        for ci in range(self.n_cables):
            pa = to_world(self.cable_body_a[ci], self.cable_anchor_a[ci])
            pb = to_world(self.cable_body_b[ci], self.cable_anchor_b[ci])
            lens[ci] = np.linalg.norm(pb - pa)
        return lens

    # ------------------------------------------------------------------
    # STATE HELPERS
    # ------------------------------------------------------------------
    def get_com(self):
        """Centre of mass = mass-weighted mean of strut base positions."""
        pos = np.zeros(3)
        for bid in self.strut_bodies:
            bp, _ = p.getBasePositionAndOrientation(bid)
            pos += np.array(bp)
        return pos / len(self.strut_bodies)

    def get_com_velocity(self):
        vel = np.zeros(3)
        for bid in self.strut_bodies:
            lv, _ = p.getBaseVelocity(bid)
            vel += np.array(lv)
        return vel / len(self.strut_bodies)

    def get_ball_radius(self):
        """Average radius of the ball (for obstacle-relative sizing)."""
        com = self.get_com()
        r = 0.0
        for bid in self.strut_bodies:
            bp, _ = p.getBasePositionAndOrientation(bid)
            r += np.linalg.norm(np.array(bp) - com)
        return r / len(self.strut_bodies)
