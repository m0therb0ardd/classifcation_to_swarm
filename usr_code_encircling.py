# -*- coding: utf-8 -*-
from __future__ import division
import math
import os
import random

# --- field bounds (meters) ---
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# --- dancer no-go circle (meters) ---
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
OBST_CX, OBST_CY = (-0.1, 0.475)

# --- dual-ring parameters ---
R_INNER   = SAFE_BUBBLE + 0.24
R_OUTER   = SAFE_BUBBLE + 0.42
DIR_INNER = +1   # +1 = CCW
DIR_OUTER = -1   # -1 = CW
ASSIGN_MODE = "by_id"  # "by_id" or "by_initial_radius"

# --- ring holding and tangent speed ---
V_TANGENT_BASE = 0.26
K_R            = 1.2
RADIAL_CLAMP   = 0.10

# --- angular spacing within a ring ---
ANG_REP_GAIN   = 0.24
ANG_REP_POW    = 1.2
ANG_REP_CUTOFF = 1.2
MIN_LINEAR_SEP = 0.18

# --- boundary softness ---
SOFT_MARGIN     = 0.08
CRIT_MARGIN     = 0.02
SOFT_MAX_FORCE  = 0.35

# --- drive / control (match sim) ---
MAX_WHEEL = 35 #max speed is 50 
TURN_K    = 3.0
FWD_FAST  = 0.8
FWD_SLOW  = 0.30
FWD_MIN   = 0.40
EPS       = 1e-3

# --- command smoothing for real robots ---
CMD_SMOOTH  = 0.25   # 0=no smoothing, 1=hold last
PRINT_PERIOD = 2.0
LOOP_DT_MS   = 40

# --- helpers ---
def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v

def wrap_angle(a):
    while a >  math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a

def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p) >= 3:
        return float(p[0]), float(p[1]), float(p[2])
    return None

def soft_boundary_check(x, y):
    # Return 0=ok, 1=warn, 2=critical based on margins.
    if (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
        y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN):
        return 2
    elif (x < X_MIN + SOFT_MARGIN or x > X_MAX - SOFT_MARGIN or
          y < Y_MIN + SOFT_MARGIN or y > Y_MAX - SOFT_MARGIN):
        return 1
    return 0

def soft_boundary_force(x, y):
    # Soft push back toward interior near walls.
    fx = 0.0
    fy = 0.0
    if x < X_MIN + SOFT_MARGIN:
        fx += SOFT_MAX_FORCE * (1.0 - (x - X_MIN) / SOFT_MARGIN)
    elif x > X_MAX - SOFT_MARGIN:
        fx -= SOFT_MAX_FORCE * (1.0 - (X_MAX - x) / SOFT_MARGIN)
    if y < Y_MIN + SOFT_MARGIN:
        fy += SOFT_MAX_FORCE * (1.0 - (y - Y_MIN) / SOFT_MARGIN)
    elif y > Y_MAX - SOFT_MARGIN:
        fy -= SOFT_MAX_FORCE * (1.0 - (Y_MAX - y) / SOFT_MARGIN)
    return fx, fy

def try_get_swarm_poses(robot):
    # Try a few common API names for neighbor poses; return [] if none.
    for nm in ('get_swarm_poses', 'get_all_poses', 'get_poses', 'swarm_poses'):
        fn = getattr(robot, nm, None)
        if callable(fn):
            try:
                poses = fn()
                if poses:
                    return poses
            except:
                pass
    return []

def get_vid(robot):
    try:
        return robot.virtual_id()
    except:
        return -1

def nearest_ring_radius(r):
    if abs(r - R_INNER) <= abs(r - R_OUTER):
        return R_INNER
    return R_OUTER

def ring_dir(ring_R):
    if abs(ring_R - R_INNER) < abs(ring_R - R_OUTER):
        return DIR_INNER
    return DIR_OUTER

# --- main user entrypoint ---
def usr(robot):
    robot.delay(2000)

    # open shared log file and define a safe writer
    log_main = open("experiment_log.txt", "a")
    def logw(s):
        if not s.endswith("\n"):
            s += "\n"
        log_main.write(s)
        log_main.flush()
        try:
            os.fsync(log_main.fileno())
        except:
            pass

    # basic id and per-robot seed
    try:
        rid = robot.virtual_id()
    except:
        rid = -1

    try:
        rnd_seed = int((rid if rid is not None else 0) * 73856093) & 0xFFFFFFFF
    except:
        rnd_seed = 0
    random.seed(rnd_seed)

    # initial visual
    try:
        robot.set_led(0, 180, 180)
    except:
        pass

    # ring assignment state
    assigned_R = None
    assigned_dir = None
    init_done = False

    last_log_time = robot.get_clock()
    last_left = 0
    last_right = 0
    told_no_api = False

    logw("Robot %s start" % str(rid))

    try:
        while True:
            pose = safe_pose(robot)
            if pose is None:
                robot.set_vel(0, 0)
                robot.delay(20)
                continue

            x, y, th = pose

            # safety around obstacle center
            dxo = x - OBST_CX
            dyo = y - OBST_CY
            r   = math.hypot(dxo, dyo)
            if r < OBST_RADIUS:
                logw("CRITICAL: robot %s inside dancer disk [%.3f, %.3f]" % (str(rid), x, y))
                robot.set_vel(0, 0)
                robot.set_led(255, 0, 0)
                robot.delay(LOOP_DT_MS)
                continue

            # boundary checks
            bstat = soft_boundary_check(x, y)
            if bstat == 2:
                logw("CRITICAL: robot %s at boundary [%.3f, %.3f]" % (str(rid), x, y))
                robot.set_vel(0, 0)
                robot.set_led(255, 0, 0)
                robot.delay(LOOP_DT_MS)
                continue
            elif bstat == 1:
                robot.set_led(255, 150, 0)
            else:
                robot.set_led(0, 180, 180)

            # one-time ring assignment
            if not init_done:
                if ASSIGN_MODE == "by_id":
                    if (rid % 2) == 0:
                        assigned_R = R_INNER
                        assigned_dir = DIR_INNER
                    else:
                        assigned_R = R_OUTER
                        assigned_dir = DIR_OUTER
                else:
                    assigned_R = nearest_ring_radius(r)
                    assigned_dir = ring_dir(assigned_R)
                init_done = True
                logw("Robot %s assigned R=%.2f dir=%s" % (str(rid), assigned_R,
                                                          ("CCW" if assigned_dir > 0 else "CW")))

            # polar basis around obstacle center
            if r < 1e-6:
                urx, ury = 1.0, 0.0
            else:
                urx, ury = dxo / r, dyo / r
            utx, uty = -ury, urx
            if assigned_dir < 0:
                utx, uty = -utx, -uty

            # base tangent velocity
            vx = V_TANGENT_BASE * utx
            vy = V_TANGENT_BASE * uty

            # radial correction to hold ring
            radial = clamp(K_R * (assigned_R - r), -RADIAL_CLAMP, RADIAL_CLAMP)
            vx += radial * urx
            vy += radial * ury

            # soft boundary force projected to radial
            bfx, bfy = soft_boundary_force(x, y)
            b_norm = bfx * urx + bfy * ury
            vx += b_norm * urx
            vy += b_norm * ury

            # angular spacing with ring-mates
            neighbors = try_get_swarm_poses(robot)
            if neighbors:
                theta = math.atan2(dyo, dxo)
                for item in neighbors:
                    if isinstance(item, (list, tuple)) and len(item) >= 3:
                        if len(item) == 4:
                            nid, nx, ny, nth = item
                        else:
                            nx, ny, nth = item[0], item[1], item[2]
                            nid = None

                        if nid is not None and str(nid) == str(rid):
                            continue

                        nr = math.hypot(nx - OBST_CX, ny - OBST_CY)
                        if ASSIGN_MODE == "by_id" and len(item) == 4 and nid is not None:
                            nR = (R_INNER if (int(nid) % 2 == 0) else R_OUTER)
                        else:
                            nR = nearest_ring_radius(nr)

                        # only interact tangentially with neighbors on the same ring
                        if abs(nR - assigned_R) < abs(R_OUTER - R_INNER) / 2.0:
                            ddx = x - nx
                            ddy = y - ny
                            d2 = ddx * ddx + ddy * ddy

                            # keep a minimum linear separation with a small radial push
                            if d2 < MIN_LINEAR_SEP * MIN_LINEAR_SEP:
                                s = ((MIN_LINEAR_SEP * MIN_LINEAR_SEP) - d2) / (MIN_LINEAR_SEP * MIN_LINEAR_SEP)
                                vx += s * urx
                                vy += s * ury

                            ntheta = math.atan2(ny - OBST_CY, nx - OBST_CX)
                            dtheta = wrap_angle(theta - ntheta)
                            ad = abs(dtheta)
                            if 1e-3 < ad <= ANG_REP_CUTOFF:
                                strength = ANG_REP_GAIN / (ad ** ANG_REP_POW)
                                tang_push = strength * (1.0 if dtheta > 0.0 else -1.0)
                                vx += tang_push * utx
                                vy += tang_push * uty
            else:
                if not told_no_api:
                    logw("Robot %s: no swarm pose API; angular spacing limited" % str(rid))
                    told_no_api = True

            # avoid stall
            spd = math.hypot(vx, vy)
            if spd < EPS:
                vx += 0.08 * utx
                vy += 0.08 * uty
                spd = math.hypot(vx, vy)

            # heading control and wheel mapping
            hdg = math.atan2(vy, vx)
            err = wrap_angle(hdg - th)

            ae = abs(err)
            if ae < 0.5:
                fwd = FWD_FAST
            elif ae < 1.2:
                fwd = FWD_FAST * 0.7
            else:
                fwd = FWD_SLOW

            if abs(b_norm) > 1e-6:
                fwd *= 0.85

            if fwd < FWD_MIN:
                fwd = FWD_MIN

            turn = clamp(TURN_K * err, -1.5, 1.5)
            left_cmd  = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
            right_cmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)

            # command smoothing
            left  = int((1.0 - CMD_SMOOTH) * left_cmd  + CMD_SMOOTH * last_left)
            right = int((1.0 - CMD_SMOOTH) * right_cmd + CMD_SMOOTH * last_right)
            last_left, last_right = left, right

            robot.set_vel(left, right)

            # periodic logging
            now = robot.get_clock()
            if now - last_log_time > PRINT_PERIOD:
                logw("Robot %s R*=%.2f r=%.3f pos[%.3f, %.3f] spd=%.3f" %
                     (str(rid), assigned_R, r, x, y, spd))
                last_log_time = now

            robot.delay(LOOP_DT_MS)

    except Exception as e:
        # stop and mark error
        try:
            robot.set_vel(0, 0)
            robot.set_led(255, 0, 0)
        except:
            pass
        logw("ERROR: %s" % str(e))
        raise
    finally:
        try:
            robot.set_vel(0, 0)
        except:
            pass
        logw("Robot %s finished" % str(rid))
        try:
            log_main.close()
        except:
            pass
