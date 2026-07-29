#!/usr/bin/env python3
"""
plot_kinematics.py
==================
Real-time visualiser of kinematic profiles published by the local Chebyshev
planner (CERES_MODE 5 or 7, PUBLISH_KINEMATICS_PROFILE 1).

Subscribes to (under <planner_ns>):
  kinematics/s                     — parameter s ∈ [-1, 1]   (N samples)
  kinematics/velocity_magnitude    — ||v(s)||₂  [m/s]
  kinematics/acceleration_magnitude — ||a(s)||₂ [m/s²]
  kinematics/jerk_magnitude        — ||j(s)||₂  [m/s³]
  kinematics/T                     — trajectory duration T   [s]  (Float64)

Kinematic limits are read from ROS params (set by local_planner_configuration.yaml):
  <planner_ns>/v_max_ms   (default 3.0)
  <planner_ns>/a_max_ms2  (default 2.0)
  <planner_ns>/j_max_ms3  (default 4.0)

Usage:
  rosrun heuristic_planners plot_kinematics.py
  rosrun heuristic_planners plot_kinematics.py _planner_ns:=/my_node
"""

import threading

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float64, Float64MultiArray

# ─── shared state ─────────────────────────────────────────────────────────────
_lock    = threading.Lock()
_latest  = {"s": None, "vel": None, "accel": None, "jerk": None, "T": None}
_updated = threading.Event()
_iters   = [0]


# ─── ROS callbacks ────────────────────────────────────────────────────────────

def _cb_s(msg):
    with _lock:
        _latest["s"] = list(msg.data)
    _check_ready()

def _cb_vel(msg):
    with _lock:
        _latest["vel"] = list(msg.data)
    _check_ready()

def _cb_accel(msg):
    with _lock:
        _latest["accel"] = list(msg.data)
    _check_ready()

def _cb_jerk(msg):
    with _lock:
        _latest["jerk"] = list(msg.data)
    _check_ready()

def _cb_T(msg):
    with _lock:
        _latest["T"] = msg.data
    _check_ready()

def _check_ready():
    with _lock:
        if all(v is not None for v in _latest.values()):
            _updated.set()


# ─── figure setup ─────────────────────────────────────────────────────────────

def _make_figure(v_max, a_max, j_max):
    fig, (ax_v, ax_a, ax_j) = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    fig.suptitle("Kinematic profiles — local Chebyshev planner",
                 fontsize=12, fontweight="bold")

    # velocity
    line_v, = ax_v.plot([], [], color="royalblue", linewidth=2, label="||v(s)||")
    ax_v.axhline(v_max, color="royalblue", linewidth=1.2, linestyle="--",
                 alpha=0.7, label=f"v_max = {v_max} m/s")
    ax_v.set_ylabel("velocity  [m/s]")
    ax_v.legend(loc="upper right", fontsize=8)
    ax_v.grid(True, linestyle="--", alpha=0.5)

    # acceleration
    line_a, = ax_a.plot([], [], color="tomato", linewidth=2, label="||a(s)||")
    ax_a.axhline(a_max, color="tomato", linewidth=1.2, linestyle="--",
                 alpha=0.7, label=f"a_max = {a_max} m/s²")
    ax_a.set_ylabel("acceleration  [m/s²]")
    ax_a.legend(loc="upper right", fontsize=8)
    ax_a.grid(True, linestyle="--", alpha=0.5)

    # jerk
    line_j, = ax_j.plot([], [], color="seagreen", linewidth=2, label="||j(s)||")
    ax_j.axhline(j_max, color="seagreen", linewidth=1.2, linestyle="--",
                 alpha=0.7, label=f"j_max = {j_max} m/s³")
    ax_j.set_xlabel("s  ∈  [−1, 1]")
    ax_j.set_ylabel("jerk  [m/s³]")
    ax_j.legend(loc="upper right", fontsize=8)
    ax_j.grid(True, linestyle="--", alpha=0.5)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    status_text = fig.text(0.01, 0.005,
                           "Waiting for first planner iteration…",
                           fontsize=8, color="gray")

    return fig, line_v, line_a, line_j, ax_v, ax_a, ax_j, status_text


# ─── animation update ─────────────────────────────────────────────────────────

def _make_update(line_v, line_a, line_j, ax_v, ax_a, ax_j, status_text):

    def update(_frame):
        if not _updated.is_set():
            return line_v, line_a, line_j

        _updated.clear()

        with _lock:
            s     = list(_latest["s"])
            vel   = list(_latest["vel"])
            accel = list(_latest["accel"])
            jerk  = list(_latest["jerk"])
            T_val = _latest["T"]

        line_v.set_data(s, vel)
        line_a.set_data(s, accel)
        line_j.set_data(s, jerk)

        for ax in (ax_v, ax_a, ax_j):
            ax.relim()
            ax.autoscale_view()

        _iters[0] += 1
        T_str = f"{T_val:.3f} s" if T_val is not None else "—"
        status_text.set_text(f"Iteration {_iters[0]}   |   T = {T_str}")

        return line_v, line_a, line_j

    return update


# ─── main ─────────────────────────────────────────────────────────────────────

def main():
    rospy.init_node("plot_kinematics", anonymous=True)

    ns = rospy.get_param("~planner_ns", "/local_planner_ros_node")

    v_max = rospy.get_param(f"{ns}/v_max_ms",  3.0)
    a_max = rospy.get_param(f"{ns}/a_max_ms2", 2.0)
    j_max = rospy.get_param(f"{ns}/j_max_ms3", 4.0)

    rospy.Subscriber(f"{ns}/kinematics/s",                      Float64MultiArray, _cb_s)
    rospy.Subscriber(f"{ns}/kinematics/velocity_magnitude",     Float64MultiArray, _cb_vel)
    rospy.Subscriber(f"{ns}/kinematics/acceleration_magnitude", Float64MultiArray, _cb_accel)
    rospy.Subscriber(f"{ns}/kinematics/jerk_magnitude",         Float64MultiArray, _cb_jerk)
    rospy.Subscriber(f"{ns}/kinematics/T",                      Float64,           _cb_T)

    rospy.loginfo(f"[plot_kinematics] Subscribed to {ns}/kinematics/{{s, velocity_magnitude, "
                  f"acceleration_magnitude, jerk_magnitude, T}}")
    rospy.loginfo(f"[plot_kinematics] Limits: v_max={v_max} m/s  a_max={a_max} m/s²  j_max={j_max} m/s³")

    fig, line_v, line_a, line_j, ax_v, ax_a, ax_j, status_text = _make_figure(v_max, a_max, j_max)
    update_fn = _make_update(line_v, line_a, line_j, ax_v, ax_a, ax_j, status_text)

    _ani = animation.FuncAnimation(fig, update_fn, interval=100, blit=False)  # noqa: F841

    plt.show()


if __name__ == "__main__":
    main()
