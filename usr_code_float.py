# float left
# glide left
# punch right
# slash right 




# -*- coding: utf-8 -*-
"""
FLOAT mode — gentle, airy drift with a shared motion to the LEFT.
Design:
  - Constant small leftward drift
  - Smooth "flow-noise" perturbations (low amplitude)
  - Weak alignment + very light cohesion to keep a soft cloud
  - Keep your existing safety rails: boundary softness, obstacle bubble, repulsion
"""

import math
import os
import random

# --- field bounds (meters) ---
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# --- dancer no-go circle (meters) ---
FEET = 0.6048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET   # ~0.1524
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
OBST_CX, OBST_CY = (-0.1, 0.475)

# --- drive / control ---
MAX_WHEEL = 35
TURN_K    = 3.0
FWD_FAST  = 0.80
FWD_SLOW  = 0.35
FWD_MIN   = 0.33
EPS       = 1e-3

# --- command smoothing (more smoothing for float) ---
CMD_SMOOTH  = 0.30

# --- boundary softness ---
SOFT_MARGIN     = 0.08
CRIT_MARGIN     = 0.02
SOFT_MAX_FORCE  = 0.35

# --- neighbor spacing (reuse your repulsion model) ---
REPULSE_RADIUS  = 0.75
REPULSE_GAIN    = 0.10
HARD_REP_RADIUS = 0.18
HARD_REP_GAIN   = 0.26

# --- FLOAT field params ---
LEFT_DRIFT_VX   = -0.10  # constant gentle push left
NOISE_GAIN      = 0.10   # slightly smaller than glitch
ALIGN_GAIN      = 0.06   # weak alignment
COHERE_GAIN     = 0.04   # very light cohesion
NOISE_SCALE     = 0.35   # flow field spatial scale
NOISE_SPEED     = 0.05   # flow field time drift

# --- timing ---
PRINT_PERIOD = 2.0
MAX_RUNTIME  = 55.0
LOOP_DT_MS   = 40  # 25 Hz

# ----------------- helpers -----------------

def clamp(v, lo, hi):
    if v < lo: return lo
    if v > hi: return hi
    return v

def wrap_angle(a):
    while a >  math.pi:
        a -= 2.0*math.pi
    while a <= -math.pi:
        a += 2.0*math.pi
    return a

def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p) >= 3:
        return float(p[0]), float(p[1]), float(p[2])
    return None

def soft_boundary_check(x, y):
    """Return 0=ok, 1=warn, 2=critical based on margins."""
    if (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
        y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN):
        return 2
    elif (x < X_MIN + SOFT_MARGIN or x > X_MAX - SOFT_MARGIN or
          y < Y_MIN + SOFT_MARGIN or y > Y_MAX - SOFT_MARGIN):
        return 1
    return 0

def soft_boundary_force(x, y):
    """Soft push back toward interior near walls."""
    fx = 0.0
    fy = 0.0
    if x < X_MIN + SOFT_MARGIN:
        fx += SOFT_MAX_FORCE * (1.0 - (x - X_MIN)/SOFT_MARGIN)
    elif x > X_MAX - SOFT_MARGIN:
        fx -= SOFT_MAX_FORCE * (1.0 - (X_MAX - x)/SOFT_MARGIN)
    if y < Y_MIN + SOFT_MARGIN:
        fy += SOFT_MAX_FORCE * (1.0 - (y - Y_MIN)/SOFT_MARGIN)
    elif y > Y_MAX - SOFT_MARGIN:
        fy -= SOFT_MAX_FORCE * (1.0 - (Y_MAX - y)/SOFT_MARGIN)
    return fx, fy

def soft_obstacle_force(x, y, max_force=0.55, buffer_width=0.10):
    """Soft radial push away from dancer disk within buffer ring."""
    dx = x - OBST_CX
    dy = y - OBST_CY
    r  = math.hypot(dx, dy)
    if r < SAFE_BUBBLE + buffer_width:
        if r < 1e-6:
            return max_force, 0.0
        strength = max(0.0, (SAFE_BUBBLE + buffer_width - r) / buffer_width)
        s = max_force * strength
        return s * (dx / r), s * (dy / r)
    return 0.0, 0.0

def is_critical_obstacle(x, y, critical_margin=0.0):
    dx = x - OBST_CX
    dy = y - OBST_CY
    r  = math.hypot(dx, dy)
    return r < (OBST_RADIUS + critical_margin)

def try_get_swarm_poses(robot):
    """Try a few common API names for neighbor poses; return [] if none."""
    names = ('get_swarm_poses', 'get_all_poses', 'get_poses', 'swarm_poses')
    for nm in names:
        fn = getattr(robot, nm, None)
        if callable(fn):
            try:
                poses = fn()
                if poses:
                    return poses
            except:
                pass
    return []

def get_id(robot):
    vid_attr = getattr(robot, "virtual_id", None)
    try:
        return vid_attr() if callable(vid_attr) else int(vid_attr or 0)
    except:
        return -1

def randn():
    # small zero-mean bounded-ish noise using sum of uniforms
    return (random.random() + random.random() + random.random() - 1.5) / 1.5

def flow_noise(x, y, t, scale=NOISE_SCALE, speed=NOISE_SPEED):
    """Lightweight pseudo-Perlin made from sines with drifting phase."""
    kx = 2.1*scale
    ky = 1.7*scale
    ph = speed * t
    nx = math.sin(kx*x + 0.7*ky*y + 1.2 + ph) + 0.6*math.sin(0.6*kx*x - 1.3*ky*y + ph*0.7)
    ny = math.sin(0.9*kx*x + 1.4*ky*y + ph*1.3) + 0.6*math.sin(-1.1*kx*x + 0.5*ky*y - 0.8 + ph*0.9)
    return (0.5*nx, 0.5*ny)  # ~[-1,1]

def alignment_vec(robot, neighbors, R=0.7):
    """Return (ax, ay) = unit-average of neighbor headings within radius R."""
    rx, ry, rth = robot.get_pose()
    sx = sy = 0.0; n = 0
    for item in neighbors or []:
        if isinstance(item, (list, tuple)) and len(item) >= 3:
            if len(item) == 4: _, nx, ny, nth = item
            else: nx, ny, nth = item[0], item[1], item[2]
            dx = nx - rx; dy = ny - ry
            if dx*dx + dy*dy <= R*R:
                sx += math.cos(nth); sy += math.sin(nth); n += 1
    if n == 0: return (0.0, 0.0)
    m = math.hypot(sx, sy) or 1.0
    return (sx/m, sy/m)

def cohesion_vec(robot, neighbors, R=0.8, target_sep=0.30):
    """Pull toward local centroid minus a separation bias when far."""
    rx, ry, _ = robot.get_pose()
    cx = cy = 0.0; n = 0
    for item in neighbors or []:
        if isinstance(item, (list, tuple)) and len(item) >= 3:
            if len(item) == 4: _, nx, ny, _ = item
            else: nx, ny = item[0], item[1]
            dx = nx - rx; dy = ny - ry
            if dx*dx + dy*dy <= R*R:
                cx += nx; cy += ny; n += 1
    if n == 0: return (0.0, 0.0)
    cx /= n; cy /= n
    vx = (cx - rx); vy = (cy - ry)
    d = math.hypot(vx, vy) or 1.0
    if d < 1e-6:
        return (0.0, 0.0)
    gain = max(0.0, (d - target_sep))  # no pull if already close
    return (gain * vx/d, gain * vy/d)

# ----------------- main -----------------

def usr(robot):
    robot.delay(3000)

    # per-robot log
    try:
        vid = robot.virtual_id()
    except:
        vid = -1

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

    try:
        # per-robot noise seed (decorrelate motions)
        try:
            rnd_seed = int((vid if vid is not None else 0) * 2654435761) & 0xFFFFFFFF
        except:
            rnd_seed = 0
        random.seed(rnd_seed)

        logw("FLOAT: I am robot %s" % str(vid))

        last_log_sec = -1
        last_pose = None
        last_left = 0
        last_right = 0
        told_no_swarm_api = False

        start_time = robot.get_clock()

        while (robot.get_clock() - start_time) < MAX_RUNTIME:
            pose = safe_pose(robot)
            if pose is None:
                robot.set_vel(0, 0)
                robot.delay(LOOP_DT_MS)
                continue

            x, y, th = pose
            last_pose = (x, y)
            now = robot.get_clock()
            t = now - start_time

            # boundary light + protection
            bstat = soft_boundary_check(x, y)
            if bstat == 2:
                logw("CRITICAL: Robot %s at boundary [%.3f, %.3f]" % (str(vid), x, y))
                robot.set_vel(0, 0)
                robot.set_led(255, 0, 0)
                break
            elif bstat == 1:
                robot.set_led(180, 220, 255)  # soft cyan-white (near boundary)
            else:
                # slow "breathing" cyan
                breathe = int(140 + 60 * (0.5 + 0.5*math.sin(0.6*t)))
                robot.set_led(0, breathe, breathe)

            # emergency stop if inside the dancer disk
            if is_critical_obstacle(x, y, 0.0):
                logw("CRITICAL: Robot %s inside obstacle [%.3f, %.3f]" % (str(vid), x, y))
                robot.set_vel(0, 0)
                robot.set_led(255, 0, 0)
                robot.delay(LOOP_DT_MS)
                continue

            # --- base field composition ---

            # 1) soft boundaries + obstacle bubble
            bfx, bfy = soft_boundary_force(x, y)
            ofx, ofy = soft_obstacle_force(x, y)

            vx = bfx + ofx + LEFT_DRIFT_VX
            vy = bfy + ofy

            # 2) smooth flow noise (low amplitude)
            nx, ny = flow_noise(x, y, t, scale=NOISE_SCALE, speed=NOISE_SPEED)
            vx += NOISE_GAIN * 0.80 * nx
            vy += NOISE_GAIN * 0.80 * ny

            # 3) neighbor repulsion (like glitch, but softer)
            neighbors = try_get_swarm_poses(robot)
            if neighbors:
                for item in neighbors:
                    if isinstance(item, (list, tuple)) and len(item) >= 3:
                        if len(item) == 4:
                            nid, nxp, nyp, nth = item
                        else:
                            nxp, nyp, nth = item[0], item[1], item[2]
                            nid = None
                        if (nid is not None) and (str(nid) == str(vid)):
                            continue
                        dxn = x - nxp
                        dyn = y - nyp
                        d2  = dxn*dxn + dyn*dyn
                        if d2 < 1e-12:
                            continue
                        if d2 < (REPULSE_RADIUS*REPULSE_RADIUS):
                            s = REPULSE_GAIN / d2
                            vx += s * dxn
                            vy += s * dyn
                        d = math.sqrt(d2)
                        if d < HARD_REP_RADIUS:
                            s_hard = HARD_REP_GAIN / (d2 * d + 1e-9)
                            vx += s_hard * dxn
                            vy += s_hard * dyn
            else:
                if not told_no_swarm_api:
                    logw("Robot %s: no swarm pose API; using float sans alignment" % str(vid))
                    told_no_swarm_api = True

            # 4) weak alignment + very light cohesion
            if neighbors:
                ax, ay = alignment_vec(robot, neighbors, R=0.7)
                cx, cy = cohesion_vec(robot, neighbors, R=0.8, target_sep=0.30)
                vx += ALIGN_GAIN  * ax
                vy += ALIGN_GAIN  * ay
                vx += COHERE_GAIN * cx
                vy += COHERE_GAIN * cy

            # ---- map (vx, vy) → wheels ----

            # If vector nearly zero, give a tiny nudge to avoid stall
            if abs(vx) + abs(vy) < EPS:
                # nudge sideways relative to current heading
                vx += 0.03 * (-math.sin(th))
                vy += 0.03 * ( math.cos(th))

            hdg = math.atan2(vy, vx)
            err = wrap_angle(hdg - th)

            # float prefers smooth long arcs, not hard pivots
            ae = abs(err)
            if ae < 0.6:
                fwd = FWD_FAST * 0.95
            elif ae < 1.2:
                fwd = FWD_FAST * 0.75
            else:
                fwd = FWD_SLOW * 0.65

            if bstat == 1:
                fwd *= 0.75

            if fwd < FWD_MIN:
                fwd = FWD_MIN

            turn = clamp(TURN_K * err, -1.2, 1.2)

            left_cmd  = clamp(int(MAX_WHEEL * 0.90 * (fwd - 0.75 * turn)), -MAX_WHEEL,  MAX_WHEEL)
            right_cmd = clamp(int(MAX_WHEEL * 0.90 * (fwd + 0.75 * turn)), -MAX_WHEEL,  MAX_WHEEL)

            # smooth wheel commands (EMA) to reduce jerk → floaty
            left  = int((1.0 - CMD_SMOOTH) * left_cmd  + CMD_SMOOTH * last_left)
            right = int((1.0 - CMD_SMOOTH) * right_cmd + CMD_SMOOTH * last_right)
            last_left, last_right = left, right

            robot.set_vel(left, right)

            # periodic log
            if int(now) != last_log_sec and (now - start_time) % PRINT_PERIOD < 0.2:
                logw("FLOAT %s pos [%.3f, %.3f]" % (str(vid), x, y))
                last_log_sec = int(now)

            robot.delay(LOOP_DT_MS)

    except Exception as e:
        # error path: stop + red LED, then re-raise
        logw("ERROR(FLOAT): %s" % str(e))
        try:
            robot.set_vel(0, 0)
            robot.set_led(255, 0, 0)
        except:
            pass
        raise
    finally:
        # final log line with last pose and elapsed time
        final_time = robot.get_clock()
        if last_pose:
            lx, ly = last_pose
        else:
            lx = float('nan')
            ly = float('nan')
        try:
            robot.set_vel(0, 0)
        except:
            pass
        logw("FLOAT %s finished at [%.3f, %.3f] after %.1fs" % (str(vid), lx, ly, final_time - start_time))
        log_main.close()
