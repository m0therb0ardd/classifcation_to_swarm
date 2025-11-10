# -*- coding: utf-8 -*-
# FLOAT (HW): light • sustained • indirect
# - Uses robot.virtual_id()
# - Logs to experiment_log.txt (fsync'd)
# - 0..100 LED scale (Coachbot)
# - P2P heartbeats for soft flocking
# - Hard guards: arena edges + dancer no-go disk

import math, struct, random, os

# --- field & obstacle (meters) ---
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
OBST_CX, OBST_CY = (-0.1, 0.475)

# --- control/loop ---
MAX_WHEEL   = 35
TURN_K      = 2.2
FWD_BASE    = 0.65
FWD_MIN     = 0.35
DT_MS       = 40
CMD_SMOOTH  = 0.35          # higher smoothing → lower jerk
VEL_SLEW    = 8             # wheel cmd change limit per step

# --- float style gains ---
K_MIG   = 0.08              # gentle leftward drift
K_SEP   = 0.20              # spacing
K_ALI   = 0.18              # match headings (fluid flock)
K_COH   = 0.10              # gentle cohesion
CURVE_NOISE = 0.20          # softly varying heading

SEP_RADIUS   = 0.26
NEIGH_RADIUS = 0.75

# --- boundary softness ---
SOFT_MARGIN = 0.08
CRIT_MARGIN = 0.02
SOFT_MAX_F  = 0.35

# --- P2P heartbeats ---
HB_FMT   = 'fffffi'         # x,y,th,vx,vy,id
HB_BYTES = struct.calcsize(HB_FMT)
HB_DT    = 0.12
STALE_S  = 0.7

# ----------------- helpers -----------------
def clamp(v, lo, hi): return lo if v < lo else (hi if v > hi else v)

def wrap(a):
    while a >  math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a

def soft_boundary_force(x,y):
    fx=fy=0.0
    if x < X_MIN+SOFT_MARGIN: fx += SOFT_MAX_F*(1-(x-X_MIN)/SOFT_MARGIN)
    elif x > X_MAX-SOFT_MARGIN: fx -= SOFT_MAX_F*(1-(X_MAX-x)/SOFT_MARGIN)
    if y < Y_MIN+SOFT_MARGIN: fy += SOFT_MAX_F*(1-(y-Y_MIN)/SOFT_MARGIN)
    elif y > Y_MAX-SOFT_MARGIN: fy -= SOFT_MAX_F*(1-(Y_MAX-y)/SOFT_MARGIN)
    return fx,fy

def soft_obstacle_force(x, y, maxf=0.55, w=0.12):
    dx,dy = x-OBST_CX, y-OBST_CY; r=math.hypot(dx,dy)
    if r < SAFE_BUBBLE + w:
        if r<1e-6: return maxf,0.0
        s = max(0.0, (SAFE_BUBBLE+w-r)/w)*maxf
        return s*(dx/r), s*(dy/r)
    return 0.0,0.0

def boundary_state(x,y):
    if (x < X_MIN+CRIT_MARGIN or x > X_MAX-CRIT_MARGIN or
        y < Y_MIN+CRIT_MARGIN or y > Y_MAX-CRIT_MARGIN): return 2
    if (x < X_MIN+SOFT_MARGIN or x > X_MAX-SOFT_MARGIN or
        y < Y_MIN+SOFT_MARGIN or y > Y_MAX-SOFT_MARGIN): return 1
    return 0

def is_critical_obstacle(x,y,margin=0.0):
    dx,dy = x-OBST_CX, y-OBST_CY
    return (dx*dx + dy*dy) < (OBST_RADIUS + margin)**2

def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p)>=3: return float(p[0]),float(p[1]),float(p[2])
    return None

# ----------------- main -----------------
def usr(robot):
    # ID + logging
    try: vid = int(robot.virtual_id())
    except: vid = -1

    log = open("experiment_log.txt", "a", buffering=1)
    def logw(s):
        if not s.endswith("\n"): s += "\n"
        try:
            log.write(s); log.flush(); os.fsync(log.fileno())
        except: pass

    random.seed((vid if vid is not None else 0)*1103515245 & 0xFFFFFFFF)
    logw(f"[FLOAT HW] boot id={vid}")

    # state
    neighbors, last_seen = {}, {}
    last_hb = -1e9
    lastL = lastR = 0
    drift_phase = random.uniform(-math.pi, math.pi)

    # wake localization
    robot.set_vel(20,20); robot.delay(150)

    try:
        while True:
            pose = safe_pose(robot)
            if not pose:
                robot.set_vel(0,0); robot.delay(DT_MS); continue
            x,y,th = pose
            now = robot.get_clock()

            # hard guards
            if is_critical_obstacle(x,y,0.0) or boundary_state(x,y)==2:
                robot.set_led(100,0,0); robot.set_vel(0,0)
                logw(f"[FLOAT HW] CRITICAL id={vid} pos[{x:.3f},{y:.3f}]"); robot.delay(DT_MS); continue

            # LEDs
            b = boundary_state(x,y)
            if b==1: robot.set_led(100,60,0)  # amber near boundary
            else:    robot.set_led(0,70,80)   # calm teal

            # --- heartbeat send (finite diff for vx,vy) ---
            if now - last_hb >= HB_DT:
                x1,y1,_ = pose; t1 = now
                robot.delay(60)  # tiny pause between send/recv cycles
                p2 = safe_pose(robot)
                if p2:
                    x2,y2,th2 = p2; t2 = robot.get_clock()
                    dt = max(1e-3, t2-t1)
                    vx=(x2-x1)/dt; vy=(y2-y1)/dt
                    try:
                        robot.send_msg(struct.pack(HB_FMT, x2,y2,th2,vx,vy,vid))
                    except: pass
                    last_hb = t2; x,y,th = x2,y2,th2
                else:
                    last_hb = now

            # --- receive ---
            for m in (robot.recv_msg() or []):
                try:
                    nx,ny,nth,nvx,nvy,nid = struct.unpack(HB_FMT, m[:HB_BYTES])
                    if int(nid)!=vid:
                        neighbors[int(nid)] = (nx,ny,nth,nvx,nvy)
                        last_seen[int(nid)] = now
                except: pass
            # prune stale heartbeats
            cut = now - STALE_S
            for nid in list(neighbors.keys()):
                if last_seen.get(nid,0) < cut:
                    neighbors.pop(nid,None); last_seen.pop(nid,None)

            # --- boidsy float field ---
            ex,ey = soft_boundary_force(x,y)           # walls
            ox,oy = soft_obstacle_force(x,y)           # dancer bubble
            mx,my = -K_MIG, 0.0                        # gentle left drift

            repx=repy=cx=cy=ax=ay=0.0; n=0
            for _,(nx,ny,nth,_,_) in neighbors.items():
                dx,dy = x-nx, y-ny
                d2 = dx*dx+dy*dy
                if d2>1e-9:
                    d = math.sqrt(d2)
                    if d<SEP_RADIUS:
                        s = K_SEP * (SEP_RADIUS - d)/SEP_RADIUS
                        repx += s*(dx/d); repy += s*(dy/d)
                    if d <= NEIGH_RADIUS:
                        cx += nx; cy += ny
                        ax += math.cos(nth); ay += math.sin(nth)
                        n+=1
            cohx=cohy=alx=aly=0.0
            if n>0:
                cx/=n; cy/=n
                cohx = K_COH*(cx-x); cohy = K_COH*(cy-y)
                ah = math.atan2(ay,ax)
                alx = K_ALI*math.cos(ah); aly = K_ALI*math.sin(ah)

            # softly varying curvature
            drift_phase += 0.03
            curl = CURVE_NOISE*math.sin(drift_phase)
            curlx = 0.0; curly = curl

            vx = ex+ox+mx+repx+cohx+alx+curlx
            vy = ey+oy+my+repy+cohy+aly+curly
            if abs(vx)<1e-6 and abs(vy)<1e-6: vx = 1e-3

            # map to wheels (low jerk)
            hdg = math.atan2(vy, vx)
            err = wrap(hdg - th)

            fwd = FWD_BASE
            if b==1: fwd *= 0.8
            fwd = max(FWD_MIN, fwd)

            turn = clamp(TURN_K*err, -1.2, 1.2)
            lcmd = clamp(int(MAX_WHEEL*0.9*(fwd - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
            rcmd = clamp(int(MAX_WHEEL*0.9*(fwd + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)

            # slew limit
            if lcmd > lastL + VEL_SLEW: lcmd = lastL + VEL_SLEW
            if lcmd < lastL - VEL_SLEW: lcmd = lastL - VEL_SLEW
            if rcmd > lastR + VEL_SLEW: rcmd = lastR + VEL_SLEW
            if rcmd < lastR - VEL_SLEW: rcmd = lastR - VEL_SLEW

            left  = int((1-CMD_SMOOTH)*lcmd + CMD_SMOOTH*lastL)
            right = int((1-CMD_SMOOTH)*rcmd + CMD_SMOOTH*lastR)
            lastL, lastR = left, right
            robot.set_vel(left, right)

            robot.delay(DT_MS)

    except Exception as e:
        try: robot.set_vel(0,0); robot.set_led(100,0,0)
        except: pass
        logw(f"[FLOAT HW] ERROR id={vid}: {repr(e)}")
        raise
    finally:
        try: robot.set_vel(0,0)
        except: pass
        try: log.close()
        except: pass
