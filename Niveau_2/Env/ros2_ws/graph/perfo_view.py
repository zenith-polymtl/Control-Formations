import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

file_path = "pose_distances.csv"  # <--- change to your actual path
df = pd.read_csv(file_path)

# ---- Build/normalize arrays ----
def col(name):
    if name not in df.columns:
        raise ValueError(f"Missing expected column: {name}")
    # Keep NaNs in the series (matplotlib will break lines at NaNs),
    # but convert to numeric 1-D arrays.
    return pd.to_numeric(df[name], errors="coerce").to_numpy().ravel()


# Compatibility helper: try multiple possible column names (older scripts may
# expect pose1_*/pose2_*, but the CSV produced by monitor.py uses
# actual_pos_*/target_pos_*).
def get_col(*names):
    for name in names:
        if name in df.columns:
            return pd.to_numeric(df[name], errors="coerce").to_numpy().ravel()
    raise ValueError(f"Missing expected column(s): {names}")


# Positions: prefer pose1_/pose2_ names, fall back to actual_pos_/target_pos_
p1x, p1y, p1z = (
    get_col("pose1_x", "actual_pos_x"),
    get_col("pose1_y", "actual_pos_y"),
    get_col("pose1_z", "actual_pos_z"),
)
p2x, p2y, p2z = (
    get_col("pose2_x", "target_pos_x"),
    get_col("pose2_y", "target_pos_y"),
    get_col("pose2_z", "target_pos_z"),
)

# Total distance if available
total_distance = None
if "distance_sum" in df.columns:
    ds = pd.to_numeric(df["distance_sum"], errors="coerce").to_numpy().ravel()
    finite_ds = ds[np.isfinite(ds)]
    if finite_ds.size:
        total_distance = float(finite_ds[-1])

# ---- 3D plot ----
fig = plt.figure(figsize=(9, 7))
ax = fig.add_subplot(111, projection="3d")

ax.plot(p1x, p1y, p1z, label="measured", linewidth=1.5)
ax.plot(p2x, p2y, p2z, label="target", linewidth=1.5)

# Mark start/end (guard against empty/NaN)
def first_finite(a):
    idx = np.where(np.isfinite(a))[0]
    return idx[0] if idx.size else None


def last_finite(a):
    idx = np.where(np.isfinite(a))[0]
    return idx[-1] if idx.size else None


i1s, i1e = first_finite(p1x), last_finite(p1x)
j1s, j1e = first_finite(p2x), last_finite(p2x)

if i1s is not None:
    ax.scatter(p1x[i1s], p1y[i1s], p1z[i1s], marker="o", s=40, label="Measured start")
if i1e is not None:
    ax.scatter(p1x[i1e], p1y[i1e], p1z[i1e], marker="^", s=40, label="Measured end")
if j1s is not None:
    ax.scatter(p2x[j1s], p2y[j1s], p2z[j1s], marker="o", s=40, label="Target start")
if j1e is not None:
    ax.scatter(p2x[j1e], p2y[j1e], p2z[j1e], marker="^", s=40, label="Target end")

ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")

title = "3D Trajectories: pose1 vs pose2"
if total_distance is not None:
    title += f"  |  total distance = {total_distance:.3f} m"
ax.set_title(title)

ax.legend(loc="best")

# ---- Robust axis limits ----
def finite_concat(*arrays):
    out = []
    for a in arrays:
        a = np.asarray(a).ravel()
        m = np.isfinite(a)
        if m.any():
            out.append(a[m])
    return np.concatenate(out) if out else np.array([])


xs = finite_concat(p1x, p2x)
ys = finite_concat(p1y, p2y)
zs = finite_concat(p1z, p2z)


def padded_limits(arr):
    if arr.size == 0:
        return None
    lo, hi = np.nanmin(arr), np.nanmax(arr)
    if not np.isfinite(lo) or not np.isfinite(hi):
        return None
    if hi == lo:  # degenerate span
        pad = 1.0
        return (lo - pad, hi + pad)
    span = hi - lo
    pad = 0.05 * span
    return (lo - pad, hi + pad)


xlim = padded_limits(xs)
ylim = padded_limits(ys)
zlim = padded_limits(zs)

if xlim and ylim and zlim:
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_zlim(*zlim)
    try:
        ax.set_box_aspect((xlim[1] - xlim[0], ylim[1] - ylim[0], zlim[1] - zlim[0]))
    except Exception:
        pass  # older Matplotlib
else:
    # Fallback: let Matplotlib autoscale with any finite data we have
    if xs.size and ys.size and zs.size:
        ax.auto_scale_xyz(xs, ys, zs)
    # else: nothing finite to scale; leave defaults

plt.show()
