import os, time, json, csv, math, tempfile, shutil
from collections import deque, defaultdict

import cv2
import numpy as np
import pandas as pd
import joblib
from datetime import datetime
import mediapipe as mp

# ---------- CONFIG ----------
WINDOW_SEC      = 2.0          # sliding window size
STEP_SEC        = 0.20         # how often to run classification
FPS_TARGET      = 20.0         # frames per second
MIN_VIS         = 0.5

ON_THRESH       = 0.60         # probability to consider a gesture "on"
OFF_THRESH      = 0.55         # below this we consider it "off" (hysteresis)
MIN_EVENT_SEC   = 0.40         # must persist above ON_THRESH this long to emit an event
COOLDOWN_SEC    = 0.60         # ignore same label again for this long after an event

CAM_INDEX       = 6
FRAME_SIZE      = (640, 480)   # width, height
POSE_DRAW       = True

SESSION_TS      = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
OUT_DIR         = os.path.join("live_stream_logs", SESSION_TS)
os.makedirs(OUT_DIR, exist_ok=True)
EVENT_CSV       = os.path.join(OUT_DIR, "events.csv")
RAW_MP4         = os.path.join(OUT_DIR, "raw.mp4")


# map pose label -> (mode, extras)
label_to_mode = {
    "float":      ("float",              {}),
    "glide":      ("glide",              {}),
    "handsup":    ("glitch",             {}),
    "lefthand":   ("directional_left",   {}),
    "righthand":  ("directional_right",  {}),
    "punch":      ("punch",              {}),
    "slash":      ("slash",              {}),
    "stillness":  ("encircling",         {}),
}

PAUSE_AFTER_EVENT_SEC = 10.0   # <— your “don’t look for poses” window

# ---------- MODEL LOAD ----------
THIS_DIR   = os.path.abspath(os.path.dirname(__file__))
MODEL_PATH = os.path.join(THIS_DIR, "random_forest_model.pkl")
clf = joblib.load(MODEL_PATH)
print("Loaded model:", MODEL_PATH)
print("Classes:", list(clf.classes_))

# ---------- FEATURE EXTRACTOR (must match training) ----------
I = {
    "left_shoulder":11, "right_shoulder":12,
    "left_wrist":15, "right_wrist":16,
    "left_hip":23, "right_hip":24,
    "left_ankle":27, "right_ankle":28
}
RANGE_LMS = {"left_wrist":15,"right_wrist":16,"left_ankle":27,"right_ankle":28}

def normalize_per_frame(A):
    out = A.copy()
    for t in range(out.shape[0]):
        lhip = out[t, I["left_hip"]*3:I["left_hip"]*3+3]
        rhip = out[t, I["right_hip"]*3:I["right_hip"]*3+3]
        center = (lhip + rhip) / 2.0
        scale = np.linalg.norm(lhip - rhip) or 1.0
        for j in range(33):
            s = j*3; e = s+3
            out[t, s:e] = (out[t, s:e] - center) / scale
    return out

def joint_xy(A, idx):
    return A[:, idx*3+0], A[:, idx*3+1]

def start_end_xy(A, idx):
    x, y = joint_xy(A, idx)
    return x[0], y[0], x[-1], y[-1]

def path_len(A, idx):
    x, y = joint_xy(A, idx)
    return float(np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2)))

def straightness(A, idx):
    x0, y0, x1, y1 = start_end_xy(A, idx)
    L = path_len(A, idx) + 1e-9
    return float(np.hypot(x1 - x0, y1 - y0) / L)

def compute_features(positions, dt):
    # positions shape: (T, 99) after interpolation and normalization
    vel  = np.gradient(positions, dt, axis=0)
    acc  = np.gradient(vel,       dt, axis=0)
    jerk = np.gradient(acc,       dt, axis=0)
    vel_mag  = np.linalg.norm(vel,  axis=1)
    acc_mag  = np.linalg.norm(acc,  axis=1)
    jerk_mag = np.linalg.norm(jerk, axis=1)

    feat = {
        "mean_velocity": float(np.mean(vel_mag)),
        "max_velocity":  float(np.max(vel_mag)),
        "std_velocity":  float(np.std(vel_mag)),
        "mean_acceleration": float(np.mean(acc_mag)),
        "max_acceleration":  float(np.max(acc_mag)),
        "std_acceleration":  float(np.std(acc_mag)),
        "mean_jerk": float(np.mean(jerk_mag)),
        "max_jerk":  float(np.max(jerk_mag)),
        "std_jerk":  float(np.std(jerk_mag)),
    }
    for name, idx in RANGE_LMS.items():
        x_vals = positions[:, idx*3+0]
        y_vals = positions[:, idx*3+1]
        feat[f"range_x_{name}"] = float(np.ptp(x_vals))
        feat[f"range_y_{name}"] = float(np.ptp(y_vals))

    # body reference levels
    LSh_y = float(np.mean(positions[:, I["left_shoulder"]*3+1]))
    RSh_y = float(np.mean(positions[:, I["right_shoulder"]*3+1]))
    shoulder_y = 0.5*(LSh_y + RSh_y)
    LH_y = float(np.mean(positions[:, I["left_hip"]*3+1]))
    RH_y = float(np.mean(positions[:, I["right_hip"]*3+1]))
    hip_y = 0.5*(LH_y + RH_y)
    def rel_levels(y): return float(y - shoulder_y), float(y - hip_y)

    for tag, jidx in [("lw", I["left_wrist"]), ("rw", I["right_wrist"]),
                      ("la", I["left_ankle"]), ("ra", I["right_ankle"])]:
        x0, y0, x1, y1 = start_end_xy(positions, jidx)
        dx, dy = (x1 - x0), (y1 - y0)
        y0_sh, y0_hip = rel_levels(y0)
        y1_sh, y1_hip = rel_levels(y1)
        L = path_len(positions, jidx)
        St = straightness(positions, jidx)
        feat[f"{tag}_x0"] = x0;  feat[f"{tag}_y0"] = y0
        feat[f"{tag}_x1"] = x1;  feat[f"{tag}_y1"] = y1
        feat[f"{tag}_dx"] = dx;  feat[f"{tag}_dy"] = dy
        feat[f"{tag}_y0_minus_sh"]  = y0_sh
        feat[f"{tag}_y0_minus_hip"] = y0_hip
        feat[f"{tag}_y1_minus_sh"]  = y1_sh
        feat[f"{tag}_y1_minus_hip"] = y1_hip
        feat[f"{tag}_path_len"]     = L
        feat[f"{tag}_straight"]     = St

    feat["wrist_y_diff_start"] = float(positions[0, I["right_wrist"]*3+1] - positions[0, I["left_wrist"]*3+1])
    feat["wrist_y_diff_end"]   = float(positions[-1, I["right_wrist"]*3+1] - positions[-1, I["left_wrist"]*3+1])
    return feat

# align to model columns
MODEL_COLS = list(getattr(clf, "feature_names_in_", []))

def write_debug_json(*, label, true_label, feat, X_df, session_folder, raw_video_path):
    """Writes prediction.json in the session folder using your structure."""
    ts = datetime.now().isoformat(timespec="seconds")
    mapped_mode, mapped_extras = label_to_mode.get(label, ("unknown", {}))

    debug_info = {
        "predicted_label": str(label),
        "true_label": true_label,                    # use None if unknown
        "features": feat,                            # dict of floats
        "columns_used": list(X_df.columns),
        "timestamp": ts,
        "session_folder": session_folder,
        "raw_video": raw_video_path,
        # helpful extras:
        "mapped_mode": mapped_mode,
        "mapped_extras": mapped_extras
    }

    out_path = os.path.join(session_folder, "prediction.json")
    with open(out_path, "w") as f:
        json.dump(debug_info, f, indent=2)
    print(f"[JSON] wrote {out_path}")

def write_swarm_config(label, session_folder, raw_video_path):
    """Creates or overwrites swarm_config.json with current mode + source info."""
    mode, extras = label_to_mode.get(label, ("unknown", {}))
    data = {
        "extras": extras,
        "mode": mode,
        "source": {
            "label": label,
            "raw_video": raw_video_path,
            "session_folder": session_folder,
            "type": "live_classify"
        },
        "timestamp": time.time(),
        "version": 1
    }

    # save this in the *root project folder*, not inside session folder
    out_path = os.path.join(os.path.dirname(__file__), "swarm_config.json")
    with open(out_path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"[SWARM CONFIG] wrote {out_path}")



# ---------- MEDIA PIPE ----------
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5)
mp_draw = mp.solutions.drawing_utils

# ---------- VIDEO ----------
cap = cv2.VideoCapture(CAM_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_SIZE[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_SIZE[1])
cap.set(cv2.CAP_PROP_FPS, FPS_TARGET)

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
writer = cv2.VideoWriter(RAW_MP4, fourcc, FPS_TARGET, FRAME_SIZE)

# ---------- BUFFERS ----------
win_frames   = int(round(WINDOW_SEC * FPS_TARGET))
step_frames  = int(round(STEP_SEC   * FPS_TARGET))
deque_xyv    = deque(maxlen=win_frames)  # each item is (99,) vector
deque_time   = deque(maxlen=win_frames)

frame_idx = 0
last_run_idx = -10**9

# EMA state
ema = np.zeros(len(clf.classes_), dtype=float)
EMA_ALPHA = 0.4  # smoothing factor (0..1)
pause_until_time = -1e9   # wall-clock seconds since t0 when we resume checking


# event state: per label timing + cooldown
last_above = {c: None for c in clf.classes_}
last_event_time = {c: -1e9 for c in clf.classes_}

# write event header
with open(EVENT_CSV, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t_start", "t_end", "label", "peak_prob"])

t0 = time.time()
print("Press 'q' to quit.")
try:
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        now = time.time() - t0

        # --- GLOBAL PAUSE GATE: skip classification while paused, still record video
        is_paused = (now < pause_until_time)
        if is_paused:
            # optional HUD
            remaining = pause_until_time - now
            cv2.putText(frame, f"PAUSED {remaining:0.0f}s", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 180, 255), 3)


        # pose
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = pose.process(rgb)

        # build 99-vector
        if res.pose_landmarks:
            vec = []
            for lm in res.pose_landmarks.landmark:
                if lm.visibility > MIN_VIS:
                    vec.extend([lm.x, lm.y, lm.z])
                else:
                    vec.extend([np.nan, np.nan, np.nan])
            if POSE_DRAW:
                mp_draw.draw_landmarks(frame, res.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        else:
            vec = [np.nan]*99

        arr = np.array(vec, dtype=float)
        # per-dimension interpolation across the window happens just before feature compute
        deque_xyv.append(arr)
        deque_time.append(now)

        # classification step?
        # classification step?
        if (not is_paused) and len(deque_xyv) == win_frames and (frame_idx - last_run_idx) >= step_frames:


            last_run_idx = frame_idx
            A = np.stack(deque_xyv, axis=0)  # (T, 99)

            # interpolate NaNs per column
            for d in range(A.shape[1]):
                s = A[:, d]
                m = np.isnan(s)
                if not np.all(m):
                    s[m] = np.interp(np.flatnonzero(m), np.flatnonzero(~m), s[~m])
                A[:, d] = s
            A = np.nan_to_num(A, nan=0.0)

            # normalize
            A = normalize_per_frame(A)

            # features
            feat = compute_features(A, 1.0/FPS_TARGET)

            # align to model columns
            cols = MODEL_COLS if MODEL_COLS else list(feat.keys())
            for c in cols:
                if c not in feat:
                    feat[c] = 0.0
            X = pd.DataFrame([[feat[c] for c in cols]], columns=cols)

            # predict proba + EMA smoothing
            probs = clf.predict_proba(X)[0]
            ema = EMA_ALPHA * probs + (1.0 - EMA_ALPHA) * ema

            # event logic per class with hysteresis + min duration + cooldown
            top_idx = int(np.argmax(ema))
            top_label = clf.classes_[top_idx]
            top_prob  = float(ema[top_idx])

            # update above-threshold timers
            for i, cls in enumerate(clf.classes_):
                if ema[i] >= ON_THRESH:
                    if last_above[cls] is None:
                        last_above[cls] = deque_time[0]  # mark start at beginning of window
                else:
                    last_above[cls] = None  # reset if falls below ON

            # decide events (only for the top class to reduce overlaps)
            cls = top_label
            t_last = last_above.get(cls, None)
            recently = (deque_time[-1] - last_event_time[cls]) < COOLDOWN_SEC

            # === your existing event block ===
            if (t_last is not None) and (deque_time[-1] - t_last >= MIN_EVENT_SEC) and (top_prob >= ON_THRESH) and not recently:
                t_start = t_last
                t_end   = deque_time[-1]
                last_event_time[cls] = deque_time[-1]
                last_above[cls] = None

                # write event to CSV
                with open(EVENT_CSV, "a", newline="") as f:
                    w = csv.writer(f)
                    w.writerow([f"{t_start:.2f}", f"{t_end:.2f}", cls, f"{top_prob:.3f}"])
                print(f"[EVENT] {cls:10s} {t_start:.2f}–{t_end:.2f}  peak≈{top_prob:.2f}")

                # === NEW: write prediction.json ===
                # true_label is unknown in live mode; use None or "".
                write_debug_json(
                    label=cls,
                    true_label=None,
                    feat=feat,
                    X_df=X,                         # the aligned DataFrame you just built
                    session_folder=OUT_DIR,
                    raw_video_path=RAW_MP4
                )

                # also update global swarm_config.json
                write_swarm_config(
                    label=cls,
                    session_folder=OUT_DIR,
                    raw_video_path=RAW_MP4
                )


                # === NEW: start global pause ===
                pause_until_time = now + PAUSE_AFTER_EVENT_SEC


            # HUD
            if not is_paused:
                txt = f"{top_label}  {top_prob:0.2f}"
                cv2.putText(frame, txt, (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.1,
                            (0,255,0) if top_prob>=ON_THRESH else (0,200,200), 3)

                # tiny class bar chart
                x0, y0, w, h = 20, 60, 220, 16
                for i, cls in enumerate(clf.classes_):
                    p = float(ema[i])
                    cv2.rectangle(frame, (x0, y0 + i*(h+6)),
                                (x0 + int(w*p), y0 + i*(h+6) + h),
                                (50,200,50), -1)
                    cv2.putText(frame, f"{cls[:10]:10s} {p:0.2f}",
                                (x0 + w + 10, y0 + i*(h+6) + h - 2),
                                cv2.FONT_HERSHEY_PLAIN, 1.1, (240,240,240), 1)
            else:
                # show paused banner in the same spot
                remaining = pause_until_time - now
                cv2.putText(frame, f"PAUSED {remaining:0.0f}s", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 180, 255), 3)

        # show & record
        cv2.imshow("Live Sliding-Window Classify", frame)
        writer.write(frame)
        frame_idx += 1

        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break

finally:
    cap.release()
    writer.release()
    cv2.destroyAllWindows()
    pose.close()

print("\nSaved:")
print("  video:", RAW_MP4)
print("  events:", EVENT_CSV)
