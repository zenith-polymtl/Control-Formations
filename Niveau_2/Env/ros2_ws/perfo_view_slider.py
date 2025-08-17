#!/usr/bin/env python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import matplotlib
matplotlib.use("TkAgg")  # ou "Qt5Agg" si Qt est installé

# ========= Load CSV =========
file_path = "pose_distances.csv"   # <-- change to your actual path
df = pd.read_csv(file_path)

def col(name):
    if name not in df.columns:
        raise ValueError(f"Missing column: {name}")
    return pd.to_numeric(df[name], errors="coerce").to_numpy().ravel()

# Positions
p1x, p1y, p1z = col("pose1_x"), col("pose1_y"), col("pose1_z")
p2x, p2y, p2z = col("pose2_x"), col("pose2_y"), col("pose2_z")

# Time index (prefer 'time'; else build from sec+nsec)
if "time" in df.columns:
    t_index = pd.to_datetime(df["time"], errors="coerce", utc=True)
else:
    if not {"sample_stamp_sec", "sample_stamp_nanosec"}.issubset(df.columns):
        raise ValueError("Need either 'time' column or both 'sample_stamp_sec' and 'sample_stamp_nanosec'.")
    sec = pd.to_numeric(df["sample_stamp_sec"], errors="coerce").fillna(0).astype("int64")
    nsec = pd.to_numeric(df["sample_stamp_nanosec"], errors="coerce").fillna(0).astype("int64")
    total_ns = sec * 1_000_000_000 + nsec
    t_index = pd.to_datetime(total_ns, unit="ns", utc=True)

t_vals = pd.Series(t_index).to_numpy()
N = min(len(p1x), len(p1y), len(p1z), len(p2x), len(p2y), len(p2z), len(t_vals))

# ========= Figure & 3D axes =========
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

# Full (static) trajectories
(line1,) = ax.plot(p1x[:N], p1y[:N], p1z[:N], label="pose1 path", linewidth=1.0)
(line2,) = ax.plot(p2x[:N], p2y[:N], p2z[:N], label="pose2 path", linewidth=1.0)

# Moving points + trails
(p1_scatter,) = ax.plot([p1x[0]], [p1y[0]], [p1z[0]], marker="o", linestyle="", label="pose1")
(p2_scatter,) = ax.plot([p2x[0]], [p2y[0]], [p2z[0]], marker="o", linestyle="", label="pose2")
(p1_trail,) = ax.plot([], [], [], linewidth=2.0, label="pose1 trail")
(p2_trail,) = ax.plot([], [], [], linewidth=2.0, label="pose2 trail")

ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")

def fmt_time(i):
    try:
        return pd.to_datetime(t_vals[i]).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] + "Z"
    except Exception:
        return str(t_vals[i])

ax.set_title(f"3D Trajectories (t = {fmt_time(0)})")
ax.legend(loc="best")

# Robust axis limits
def finite_concat(*arrays):
    out = []
    for a in arrays:
        a = np.asarray(a).ravel()
        m = np.isfinite(a)
        if m.any():
            out.append(a[m])
    return np.concatenate(out) if out else np.array([])

xs = finite_concat(p1x[:N], p2x[:N])
ys = finite_concat(p1y[:N], p2y[:N])
zs = finite_concat(p1z[:N], p2z[:N])

def padded_limits(arr):
    if arr.size == 0:
        return None
    lo, hi = np.nanmin(arr), np.nanmax(arr)
    if not np.isfinite(lo) or not np.isfinite(hi):
        return None
    if hi == lo:
        pad = 1.0
        return lo - pad, hi + pad
    span = hi - lo
    pad = 0.05 * span
    return lo - pad, hi + pad

xlim, ylim, zlim = padded_limits(xs), padded_limits(ys), padded_limits(zs)
if xlim and ylim and zlim:
    ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.set_zlim(*zlim)
    try:
        ax.set_box_aspect((xlim[1]-xlim[0], ylim[1]-ylim[0], zlim[1]-zlim[0]))
    except Exception:
        pass

# ========= Slider UI =========
plt.subplots_adjust(bottom=0.15)
slider_ax = fig.add_axes([0.13, 0.06, 0.65, 0.03])
idx_slider = Slider(slider_ax, "Index", 0, N-1, valinit=0, valstep=1)

play_ax = fig.add_axes([0.81, 0.055, 0.08, 0.04])
pause_ax = fig.add_axes([0.90, 0.055, 0.08, 0.04])
play_btn = Button(play_ax, "Play")
pause_btn = Button(pause_ax, "Pause")

def set_frame(i):
    i = int(np.clip(i, 0, N-1))
    p1_scatter.set_data([p1x[i]], [p1y[i]]); p1_scatter.set_3d_properties([p1z[i]])
    p2_scatter.set_data([p2x[i]], [p2y[i]]); p2_scatter.set_3d_properties([p2z[i]])
    p1_trail.set_data(p1x[:i+1], p1y[:i+1]); p1_trail.set_3d_properties(p1z[:i+1])
    p2_trail.set_data(p2x[:i+1], p2y[:i+1]); p2_trail.set_3d_properties(p2z[:i+1])
    ax.set_title(f"3D Trajectories (t = {fmt_time(i)})")
    fig.canvas.draw_idle()

def on_slider_change(val):
    set_frame(val)

idx_slider.on_changed(on_slider_change)

# ========= Animation =========
anim_running = False
timer = fig.canvas.new_timer(interval=50)  # ~20 FPS

def on_timer(_):
    i = int(idx_slider.val) + 1
    if i >= N:
        i = 0
    idx_slider.set_val(i)  # moves slider -> triggers set_frame

timer.add_callback(on_timer, None)

def on_play(event):
    global anim_running              # <-- FIX: use global, not nonlocal
    if not anim_running:
        anim_running = True
        timer.start()

def on_pause(event):
    global anim_running              # <-- FIX: use global, not nonlocal
    if anim_running:
        anim_running = False
        timer.stop()

play_btn.on_clicked(on_play)
pause_btn.on_clicked(on_pause)

set_frame(0)
plt.show()
