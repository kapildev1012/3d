"""
config.py
---------
Shared, experiment-wide configuration for the two tensegrity locomotion
models (Rigid Baseline vs Adaptive String Relaxation).

Everything that must be IDENTICAL between the two models lives here:
robot geometry/mass, cable properties before adaptation, gravity, friction,
timestep, collision model, terrain, obstacle geometry, base locomotion
controller and simulation duration.

The single experimental difference (implemented in controllers.py) is the
availability of adaptive string relaxation and the traversal policy.
"""

import numpy as np

# ======================================================================
# PHYSICS (PyBullet)
# ======================================================================
GRAVITY     = [0.0, 0.0, -9.81]   # up = +z, forward = +x, lateral = +y
TIMESTEP    = 1.0 / 240.0          # [s] fixed physics step
SUBSTEPS    = 32                   # internal pybullet substeps
CONTROL_HZ  = 15                   # controller/state-machine update rate [Hz]
SIM_DURATION = 360.0               # [s] absolute simulation cap (safety)

# Ground / contact model
GROUND_LATERAL_FRICTION  = 1.2
GROUND_SPINNING_FRICTION = 0.6
GROUND_RESTITUTION       = 0.05
STRUT_FRICTION           = 1.4     # strut-ground / strut-obstacle friction
STRUT_RESTITUTION        = 0.08

# ======================================================================
# ROBOT - 6-strut tensegrity "super ball" (12 nodes, 6 bars, 24 cables)
# ======================================================================
STRUT_LENGTH = 1.0                 # [m] bar length  ==  robot diameter D
STRUT_RADIUS = 0.08                # [m]
STRUT_MASS   = 1.5                 # [kg] per bar  -> 9 kg robot

# Characteristic robot diameter = bar length (the expanded-octahedron
# tensegrity is inscribed in a sphere of diameter D = STRUT_LENGTH).
BALL_DIAMETER = STRUT_LENGTH
BALL_RADIUS   = STRUT_LENGTH / 2.0

# Cable model (tension-only spring + damper, matching a prestressed
# tensegrity: rest length slightly shorter than the natural chord).
CABLE_K           = 2000.0          # [N/m]  stiffness
CABLE_DAMP        = 150.0           # [N*s/m] damping along cable
CABLE_PRETENSION  = 0.010          # relative rest-length shortening at
                                   # nominal configuration (preload)

# Adaptive relaxation (Model B only)
RELAX_STEP        = 0.035          # per adaptation: +3.5% rest length
RELAX_MAX         = 0.45           # total extra-elongation safety cap
RELAX_TOP_K       = 8              # relax this many cables per episode
RETENSION_RATE    = 0.020          # re-tightening per control tick after success

# ======================================================================
# BASE LOCOMOTION CONTROLLER  (identical for both models)
# ----------------------------------------------------------------------
# A "phase wave" of cable rest-length modulation which rolls the ball
# forward.  Each cable i has a fixed body-frame phase zeta_i measured
# around the roll axis (+x).  Steering biases the wave toward the desired
# lateral direction.
# ======================================================================
GAIT_PERIOD = 1.55                  # [s]
GAIT_OMEGA  = 2.0 * np.pi / GAIT_PERIOD
GAIT_AMP    = 0.035                 # relative rest-length modulation amplitude
GAIT_SIGN   = 1.0                   # wave/lean direction sign (tuned flat)
STEER_GAIN  = 0.040                 # steering bias amplitude (relative)
STEER_SAT   = 1.5                   # steering saturation

# ======================================================================
# FIVE-OBSTACLE COURSE  (all dimensions relative to robot diameter D)
# ======================================================================
OBSTACLE_SPECS = [
    # (id, label,          height_frac_D, width_frac_D, depth_frac_D, shape)
    (1, "Small",              0.20, 1.00, 0.30, "box"),
    (2, "Medium",             0.35, 1.20, 0.35, "cylinder"),
    (3, "Large",              0.50, 1.40, 0.45, "wedge"),
    (4, "Extra-Large",        0.60, 1.60, 0.55, "stepped"),
    (5, "Variable",           0.45, 1.30, 0.40, "tilted_box"),
]

# ======================================================================
# TERRAIN / COURSE LAYOUT   (length computed, never hard-coded)
# ======================================================================
START_MARGIN    = 4.0          # free run before obstacle 1
APPROACH_DIST   = 3.0          # straight approach before each obstacle
RECOVERY_DIST   = 3.0          # re-tensioning lane after each obstacle
OBSTACLE_SPACING = 2.0         # clear gap between obstacle zones
END_MARGIN      = 6.0          # free run after the last obstacle

TERRAIN_WIDTH   = 16.0         # [m] total lateral extent (y)
TERRAIN_CELL    = 0.30         # [m] heightfield cell size
TERRAIN_OCTAVES = [
    # (amplitude [m], wavelength [m]) -- increasing frequency, decreasing amplitude
    (0.24, 14.0),
    (0.12, 6.0),
    (0.06, 2.5),
    (0.02, 0.9),
]
TERRAIN_SEED    = 1234
CORRIDOR_SMOOTHING = 1.6       # [m] half-width of the smoothed central lane
SCATTERED_ROCKS = 18           # small rocks off the smooth corridor

# ----------------------------------------------------------------------
# STUCK / FAILURE DETECTION
# ----------------------------------------------------------------------
STUCK_WINDOW       = 2.5       # [s] no-progress window -> STUCK
STUCK_VX_THRESHOLD = 0.02      # [m/s] forward speed below this is "no progress"
MAX_ATTEMPT_TIME   = 22.0      # [s] per-obstacle cap
MAX_ADAPT_ATTEMPTS = 14        # adaptive: max relaxation episodes per obstacle
BYPASS_SIDE_MARGIN = 0.9       # baseline lateral offset around obstacle
CROSSING_MIN_ELEV  = 0.55      # COM must reach >55% of obstacle height to count as "over"

# ======================================================================
# EXPERIMENT
# ======================================================================
MODEL_RIGID    = "Rigid Baseline - No String Relaxation"
MODEL_ADAPTIVE = "Adaptive - Dynamic String Relaxation"
RESULTS_DIR = "results"
def compute_course_layout():
    """Compute obstacle placement + terrain length from the margin/spacing
    components.

    Terrain Length = Start Margin
                   + Approach lanes
                   + Obstacle Zones (depths)
                   + Inter-Obstacle Spacing
                   + Recovery Zones
                   + End Margin

    Nothing is hard-coded: changing any margin in config produces a terrain
    exactly long enough for the robot to encounter all five obstacles.
    """
    n = len(OBSTACLE_SPECS)
    heights = np.array([s[3] * BALL_DIAMETER for s in OBSTACLE_SPECS])
    widths  = np.array([s[4] * BALL_DIAMETER for s in OBSTACLE_SPECS])
    depths  = np.array([s[5] * BALL_DIAMETER for s in OBSTACLE_SPECS])

    front, center, back = [], [], []
    x = START_MARGIN + APPROACH_DIST
    for i in range(n):
        front.append(x)
        center.append(x + depths[i] / 2.0)
        back.append(x + depths[i])
        x += depths[i]
        if i < n - 1:
            x += RECOVERY_DIST + OBSTACLE_SPACING + APPROACH_DIST
        else:
            x += RECOVERY_DIST
    terrain_len = x + END_MARGIN

    return dict(
        n=n,
        specs=OBSTACLE_SPECS,
        heights=heights,
        widths=widths,
        depths=depths,
        front=np.array(front),
        center=np.array(center),
        back=np.array(back),
        terrain_len=float(terrain_len),
    )