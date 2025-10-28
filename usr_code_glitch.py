# -*- coding: utf-8 -*-
import math
import os
import random

# --- field bounds (meters) ---
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# --- dancer no-go circle (meters) ---
FEET = 0.6048             # keep consistent with your other scripts
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET   # ~0.1524
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
OBST_CX, OBST_CY = (-0.1, 0.475)

# --- drive / control (match sim) ---
MAX_WHEEL = 35
TURN_K    = 3.0
FWD_FAST  = 0.8
FWD_SLOW  = 0.30
FWD_MIN   = 0.35         # forward floor to avoid spin-stall
EPS       = 1e-3

# --- command smoothing for real robots ---
CMD_SMOOTH  = 0.25       # 0=no smoothing, 1=hold last

# --- boundary softness ---
SOFT_MARGIN     = 0.08
CRIT_MARGIN     = 0.02
SOFT_MAX_FORCE  = 0.35

# --- "glitch" field params ---
REPULSE_RADIUS  = 0.75   # neighbor influence radius
REPULSE_GAIN    = 0.12   # 1/r^2 repulsion
HARD_REP_RADIUS = 0.18   # hard-core zone to prevent bumper-to-bumper
HARD_REP_GAIN   = 0.28   # extra 1/r^3 push in very close range
NOISE_GAIN      = 0.12
NOISE_SIDE_FRAC = 0.7    # project most noise sideways to current motion
FWD_GAIN        = 0.95
LEFT_BIAS_VX    = 0.00   # e.g., -0.06 for slow collective left drift

# --- timing / runtime ---
PRINT_PERIOD = 2.0
MAX_RUNTIME  = 55.0
LOOP_DT_MS   = 40        # 25 Hz

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
    warning_margin  = SOFT_MARGIN
    critical_margin = CRIT_MARGIN
    if (x < X_MIN + critical_margin or x > X_MAX - critical_margin or
        y < Y_MIN + critical_margin or y > Y_MAX - critical_margin):
        return 2
    elif (x < X_MIN + warning_margin or x > X_MAX - warning_margin or
          y < Y_MIN + warning_margin or y > Y_MAX - warning_margin):
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

def soft_obstacle_force(x, y, max_force=0.6, buffer_width=0.10):
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

def usr(robot):
    robot.delay(3000)

    # get id first so we can use a per-robot log
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
        # per-robot noise seed so motions are decorrelated but repeatable
        try:
            rnd_seed = int((vid if vid is not None else 0) * 73856093) & 0xFFFFFFFF
        except:
            rnd_seed = 0
        random.seed(rnd_seed)

        logw("I am robot %s" % str(vid))

        # state
        last_log_sec = -1
        last_pose = None    # for final log
        told_no_swarm_api = False
        last_left = 0
        last_right = 0

        start_time = robot.get_clock()

        while (robot.get_clock() - start_time) < MAX_RUNTIME:
            pose = safe_pose(robot)
            if pose is None:
                # no pose right now; safe hold
                robot.set_vel(0, 0)
                robot.delay(LOOP_DT_MS)
                continue

            x, y, th = pose
            last_pose = (x, y)

            # boundary light + protection
            bstat = soft_boundary_check(x, y)
            if bstat == 2:
                logw("CRITICAL: Robot %s at boundary [%.3f, %.3f]" % (str(vid), x, y))
                robot.set_vel(0, 0)
                robot.set_led(255, 0, 0)
                break
            elif bstat == 1:
                robot.set_led(255, 150, 0)  # warn
            else:
                robot.set_led(0, 180, 180)  # normal

            # emergency stop if inside the dancer disk
            if is_critical_obstacle(x, y, 0.0):
                logw("CRITICAL: Robot %s inside obstacle [%.3f, %.3f]" % (str(vid), x, y))
                robot.set_vel(0, 0)
                robot.set_led(255, 0, 0)
                robot.delay(LOOP_DT_MS)
                continue

            # base velocity from soft boundary + optional left bias
            bfx, bfy = soft_boundary_force(x, y)
            vx = bfx + LEFT_BIAS_VX
            vy = bfy

            # add soft obstacle repulsion
            obx, oby = soft_obstacle_force(x, y)
            vx += obx
            vy += oby

            # neighbor repulsion (1/r^2) + hard-core 1/r^3 anti-clump
            neighbors = try_get_swarm_poses(robot)
            if neighbors:
                for item in neighbors:
                    if isinstance(item, (list, tuple)) and len(item) >= 3:
                        if len(item) == 4:
                            nid, nx, ny, nth = item
                        else:
                            nx, ny, nth = item[0], item[1], item[2]
                            nid = None
                        if (nid is not None) and (str(nid) == str(vid)):
                            continue
                        dxn = x - nx
                        dyn = y - ny
                        d2  = dxn*dxn + dyn*dyn
                        if d2 < 1e-12:
                            continue
                        # regular 1/r^2
                        if d2 < (REPULSE_RADIUS*REPULSE_RADIUS):
                            s = REPULSE_GAIN / d2
                            vx += s * dxn
                            vy += s * dyn
                        # hard-core 1/r^3 close up
                        d = math.sqrt(d2)
                        if d < HARD_REP_RADIUS:
                            s_hard = HARD_REP_GAIN / (d2 * d + 1e-9)
                            vx += s_hard * dxn
                            vy += s_hard * dyn
            else:
                if not told_no_swarm_api:
                    logw("Robot %s: no swarm pose API; using jitter fallback" % str(vid))
                    told_no_swarm_api = True

            # noise projected mostly sideways to current motion
            spd_tmp = math.hypot(vx, vy)
            if spd_tmp > 1e-6:
                ux = vx / spd_tmp
                uy = vy / spd_tmp
                tx = -uy
                ty = ux
            else:
                ux, uy, tx, ty = 1.0, 0.0, 0.0, 1.0

            a1 = random.uniform(-math.pi, math.pi)
            a2 = random.uniform(-math.pi, math.pi)
            noise_side = (NOISE_GAIN * math.cos(a1) + 0.6 * NOISE_GAIN * math.cos(a2))
            noise_fwd  = (NOISE_GAIN * math.sin(a1) + 0.6 * NOISE_GAIN * math.sin(a2))
            vx += NOISE_SIDE_FRAC * noise_side * tx + (1.0 - NOISE_SIDE_FRAC) * noise_fwd * ux
            vy += NOISE_SIDE_FRAC * noise_side * ty + (1.0 - NOISE_SIDE_FRAC) * noise_fwd * uy

            # map (vx,vy) -> wheels
            if abs(vx) + abs(vy) < EPS:
                # tiny nudge to avoid dead-zero stall
                vx += 0.04 * tx
                vy += 0.04 * ty

            hdg = math.atan2(vy, vx)
            err = wrap_angle(hdg - th)

            ae = abs(err)
            if ae < 0.5:
                fwd = FWD_FAST * FWD_GAIN
            elif ae < 1.2:
                fwd = FWD_FAST * 0.7 * FWD_GAIN
            else:
                fwd = FWD_SLOW * 0.6 * FWD_GAIN

            if bstat == 1:
                fwd *= 0.7

            # forward floor to keep moving even while turning
            if fwd < FWD_MIN:
                fwd = FWD_MIN

            turn = clamp(TURN_K * err, -1.5, 1.5)

            left_cmd  = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL,  MAX_WHEEL)
            right_cmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL,  MAX_WHEEL)

            # smooth wheel commands (EMA) to reduce jerk
            left  = int((1.0 - CMD_SMOOTH) * left_cmd  + CMD_SMOOTH * last_left)
            right = int((1.0 - CMD_SMOOTH) * right_cmd + CMD_SMOOTH * last_right)
            last_left, last_right = left, right

            robot.set_vel(left, right)

            # periodic log (not too chatty)
            now = robot.get_clock()
            if int(now) != last_log_sec and (now - start_time) % PRINT_PERIOD < 0.2:
                logw("Robot %s pos [%.3f, %.3f]" % (str(vid), x, y))
                last_log_sec = int(now)

            robot.delay(LOOP_DT_MS)

    except Exception as e:
        # error path: stop + red LED, then re-raise
        logw("ERROR: %s" % str(e))
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
        logw("Robot %s finished at [%.3f, %.3f] after %.1fs" % (str(vid), lx, ly, final_time - start_time))
        log_main.close()
