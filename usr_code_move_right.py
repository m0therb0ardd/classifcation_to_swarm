import math
import os

# --- field and ring center ---
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35
CX, CY = (-0.1, 0.475)

# --- Dancer no-go circle (meters) ---
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN

# --- Motion & control (MATCH SIM) ---
MOVE_DIR        = 1          # <- RIGHT
BASE_SHIFT_RATE = 0.18
STOP_MARGIN     = 0.08
MAX_WHEEL = 35
TURN_K    = 3.0
FWD_FAST  = 0.8
FWD_SLOW  = 0.30
EPS       = 1e-3
KX = 1.2
KY = 2.0
KR = 2.6

def clamp(v, lo, hi): 
    return max(lo, min(v, hi))

def wrap_angle(a):
    while a > math.pi: 
        a -= 2.0*math.pi
    while a <= -math.pi: 
        a += 2.0*math.pi
    return a

def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p) >= 3:
        return float(p[0]), float(p[1]), float(p[2])
    return None

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
        logw("I am robot %s" % str(vid))

        def soft_boundary_check(x, y):
            warning_margin = 0.08
            critical_margin = 0.02
            if (x < X_MIN + critical_margin or x > X_MAX - critical_margin or
                y < Y_MIN + critical_margin or y > Y_MAX - critical_margin):
                return 2
            elif (x < X_MIN + warning_margin or x > X_MAX - warning_margin or
                  y < Y_MIN + warning_margin or y > Y_MAX - warning_margin):
                return 1
            return 0

        # State
        rel_off = None
        R_form = None
        s_stop = None
        t0 = None
        started = False
        last_log_sec = -1
        last_pose = None  # for final log

        start_time = robot.get_clock()
        max_runtime = 55.0

        while (robot.get_clock() - start_time) < max_runtime:
            pose = safe_pose(robot)
            if pose is None:
                robot.set_vel(0, 0)
                robot.delay(20)
                continue

            x, y, th = pose
            last_pose = (x, y)

            # Boundary light + protection
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

            # Init on first loop with pose
            if rel_off is None:
                rel_off = (x - CX, y - CY)
                R_form = math.hypot(rel_off[0], rel_off[1])

                safety_buffer = 0.05
                if MOVE_DIR < 0:  # LEFT
                    s_wall = max(0.0, CX - (X_MIN + STOP_MARGIN + safety_buffer + R_form))
                else:             # RIGHT
                    s_wall = max(0.0, (X_MAX - STOP_MARGIN - safety_buffer - R_form) - CX)

                s_obst = max(0.0, R_form - SAFE_BUBBLE)
                s_stop = min(s_wall, s_obst)

                t0 = robot.get_clock() + 1.0
                logw("Robot %s: R=%.3f, s_stop=%.3f, rate=%.3f" % (str(vid), R_form, s_stop, BASE_SHIFT_RATE))
                if s_stop <= 0.0:
                    logw("Robot %s: s_stop=0, no safe translation; holding position" % str(vid))
                robot.set_led(255, 200, 0)

            # Wait for synchronized start
            if not started:
                if robot.get_clock() < t0:
                    robot.set_vel(0, 0)
                    robot.delay(10)
                    continue
                started = True
                robot.set_led(0, 200, 0)
                logw("Robot %s started" % str(vid))

            # Compute center shift along -x
            s = max(0.0, (robot.get_clock() - t0) * BASE_SHIFT_RATE)
            if s_stop is not None:
                s = min(s, s_stop)

            Cx = CX + MOVE_DIR * s
            Cy = CY

            # Keep ring center valid
            Cx = max(X_MIN + R_form + 0.08, min(X_MAX - R_form - 0.08, Cx))
            Cy = max(Y_MIN + R_form + 0.08, min(Y_MAX - R_form - 0.08, Cy))

            # Target is ring-offset from shifted center
            tx = Cx + rel_off[0]
            ty = Cy + rel_off[1]

            ex = tx - x
            ey = ty - y

            # If reached the planned shift and position error is tiny -> done
            if (s_stop is not None) and (abs(s_stop - s) < 1e-6) and (math.hypot(ex, ey) < 0.02):
                robot.set_vel(0, 0)
                robot.set_led(0, 80, 255)
                logw("Robot %s completed mission" % str(vid))
                break

            # Smooth heading + speed control toward (tx, ty)
            vx = KX * ex + MOVE_DIR * BASE_SHIFT_RATE + KR * (rel_off[0] - (x - Cx))
            vy = KY * ey + KR * (rel_off[1] - (y - Cy))

            if abs(vx) + abs(vy) > EPS:
                hdg = math.atan2(vy, vx)
                err = wrap_angle(hdg - th)

                abs_err = abs(err)
                if abs_err < 0.5:
                    fwd = FWD_FAST
                elif abs_err < 1.2:
                    fwd = FWD_FAST * 0.7
                else:
                    fwd = FWD_SLOW

                if bstat == 1:
                    fwd *= 0.7

                turn = clamp(TURN_K * err, -1.5, 1.5)

                left  = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL,  MAX_WHEEL)
                right = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL,  MAX_WHEEL)
                robot.set_vel(left, right)
            else:
                robot.set_vel(0, 0)

            robot.delay(20)

    except Exception as e:
        logw("ERROR: %s" % str(e))
        try:
            robot.set_vel(0, 0)
            robot.set_led(255, 0, 0)
        except:
            pass
        raise
    finally:
        final_time = robot.get_clock()
        if last_pose:
            lx, ly = last_pose
        else:
            lx = ly = float('nan')
        try:
            robot.set_vel(0, 0)
        except:
            pass
        logw("Robot %s finished at [%.3f, %.3f] after %.1fs" % (str(vid), lx, ly, final_time - start_time))
        log_main.close()

