#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
benchmark_viz.py
================

Show the three planners' trajectories together in RViz, replayed in sync as the
drone flies.

The three benchmark runs were recorded separately, each replaying the SAME bag,
so their `stamp` column is the bag's own clock: at a given simulation time all
three logs place the drone within a few centimetres of each other. That is what
makes synchronised playback possible without re-recording anything.

Modes (~mode)
-------------
  live    (default) replay all three trajectories in time, so they update as the
          drone flies. Follows /clock when a bag is playing, otherwise runs its
          own clock (see ~playback).
  sites   static: only the matched comparison sites, for a clean screenshot.
  all     static: every logged trajectory, subsampled by ~stride.

Typical use
-----------
  # A) alongside a bag replay -- trajectories follow the real flight
  roslaunch hector_quadrotor_demo simulator_fr_campus_replay.launch
  rosrun heuristic_planners benchmark_viz.py _playback:=clock
  rosbag play -d 3 --clock bench_flight.bag

  # B) standalone -- no bag, the node drives its own clock
  roslaunch hector_quadrotor_demo simulator_fr_campus_replay.launch
  rosrun heuristic_planners benchmark_viz.py _playback:=internal _rate:=1.0

  # then in RViz: Add -> By topic -> /benchmark_viz/*

Topics (frame `map`)
--------------------
  /benchmark_viz/c3to           MarkerArray   trajectory (+ trail in live mode)
  /benchmark_viz/fast_planner   MarkerArray
  /benchmark_viz/ego_planner    MarkerArray
  /benchmark_viz/global_path    MarkerArray   straight start-to-goal reference
  /benchmark_viz/columns        MarkerArray   column obstacles from the .world
  /benchmark_viz/drone          MarkerArray   drone position (live mode)

Parameters
----------
  ~results_dir   folder holding the CSV pairs
                 (default: /home/ros/exchange/RAL26_Benchmark_Multibag/results)
  ~c3to/~fast_planner/~ego_planner   explicit metrics CSV paths (else newest)
  ~world         Gazebo .world to read the columns from
  ~playback      "clock" follows /clock, "internal" drives its own (default)
  ~rate          playback speed multiplier for internal mode (default 1.0)
  ~loop          restart when the run ends (default True)
  ~trail         how many past trajectories to keep faintly visible (default 0)
  ~fps           publish rate in live mode (default 20)
  ~n_sites/~max_mismatch/~stride/~line_width/~frame_id
"""

import bisect
import csv
import glob
import os
import re

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# Same Okabe-Ito palette as the paper figures, so RViz and the plots agree.
PLANNERS = [
    ("c3to",         "C-3TO",        (0.000, 0.447, 0.698)),
    ("fast_planner", "Fast-Planner", (0.902, 0.624, 0.000)),
    ("ego_planner",  "EGO-Planner",  (0.000, 0.620, 0.451)),
]

GLOBAL_START = (3.0, 69.0, 2.5)
GLOBAL_GOAL = (3.0, 50.0, 2.5)
MAP_OFFSET = (146.3, 83.6, 8.2)
WORLD_DEFAULT = ("/home/ros/exchange/catkin_ws/src/hector_quadrotor_noetic/"
                 "hector_gazebo/hector_gazebo_worlds/worlds/my_world_fr_campus.world")


# --------------------------------------------------------------------------- #
# Loading
# --------------------------------------------------------------------------- #
def newest_metrics(folder, key):
    """Newest <key>_*.csv (not the _traj companion), by filename timestamp."""
    hits = [h for h in glob.glob(os.path.join(folder, "%s_*.csv" % key))
            if not h.endswith("_traj.csv")]
    if not hits:
        return None

    def stamp(p):
        m = re.search(r"_(\d{8}_\d{6})\.csv$", os.path.basename(p))
        return m.group(1) if m else "0"

    return max(hits, key=stamp)


def load_run(metrics_path):
    """Join metrics and geometry into a time-ordered list of trajectories.

    Returns (stamps, trajectories, positions): the simulation time of each
    replan, its sampled geometry, and the drone position at that moment.
    """
    if not metrics_path or not os.path.exists(metrics_path):
        return [], [], []
    geo_path = metrics_path[:-4] + "_traj.csv"
    geo = {}
    if os.path.exists(geo_path):
        for r in csv.DictReader(open(geo_path)):
            try:
                geo.setdefault(int(float(r["replan"])), []).append(
                    (float(r["x"]), float(r["y"]), float(r["z"])))
            except (TypeError, ValueError, KeyError):
                continue

    rows = []
    for r in csv.DictReader(open(metrics_path)):
        try:
            rep = int(float(r["replan"]))
            pts = geo.get(rep)
            if not pts or len(pts) < 2:
                continue
            rows.append((float(r["stamp"]), pts,
                         (float(r["x"]), float(r["y"]), float(r["z"]))))
        except (TypeError, ValueError, KeyError):
            continue
    rows.sort(key=lambda t: t[0])
    return ([t for t, _, _ in rows],
            [p for _, p, _ in rows],
            [d for _, _, d in rows])


def load_columns(world_path):
    """Column obstacles from the Gazebo world, in MAP coordinates.

    They live in the .world but not in fr_campus.bt, so the octomap point cloud
    RViz shows does not contain them -- hence publishing them as markers.
    """
    import xml.etree.ElementTree as ET
    if not world_path or not os.path.exists(world_path):
        return []
    try:
        root = ET.parse(world_path).getroot()
    except ET.ParseError:
        return []
    out = []
    for model in root.iter("model"):
        if not str(model.get("name", "")).startswith("column"):
            continue
        link = model.find("link")
        if link is None:
            continue
        pose = link.find("pose")
        size = link.find(".//collision/geometry/box/size")
        if pose is None or size is None:
            continue
        try:
            px, py, pz = [float(v) for v in pose.text.split()[:3]]
            sx, sy, sz = [float(v) for v in size.text.split()[:3]]
        except (ValueError, AttributeError):
            continue
        out.append((px + MAP_OFFSET[0], py + MAP_OFFSET[1], pz + MAP_OFFSET[2],
                    sx, sy, sz))
    return out


# --------------------------------------------------------------------------- #
# Markers
# --------------------------------------------------------------------------- #
def line_marker(pts, ns, mid, rgb, width, frame, alpha=1.0):
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = rospy.Time.now()
    m.ns, m.id = ns, mid
    m.type, m.action = Marker.LINE_STRIP, Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = width
    m.color = ColorRGBA(rgb[0], rgb[1], rgb[2], alpha)
    m.points = [Point(p[0], p[1], p[2]) for p in pts]
    m.frame_locked = True
    return m


def shape_marker(p, ns, mid, rgb, scale, frame, shape=Marker.SPHERE, alpha=1.0):
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = rospy.Time.now()
    m.ns, m.id = ns, mid
    m.type, m.action = shape, Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = p
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = scale
    m.color = ColorRGBA(rgb[0], rgb[1], rgb[2], alpha)
    m.frame_locked = True
    return m


def text_marker(p, txt, ns, mid, frame, size=0.35):
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = rospy.Time.now()
    m.ns, m.id = ns, mid
    m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = p[0], p[1], p[2] + 0.5
    m.pose.orientation.w = 1.0
    m.scale.z = size
    m.color = ColorRGBA(0.95, 0.95, 0.95, 1.0)
    m.text = txt
    m.frame_locked = True
    return m


def delete_all(ns):
    m = Marker()
    m.ns, m.action = ns, Marker.DELETEALL
    return m


# --------------------------------------------------------------------------- #
# Static site matching (modes "sites")
# --------------------------------------------------------------------------- #
def hdist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5   # horizontal only


def match_sites(runs, n_sites, max_mismatch):
    ref_key = next((k for k, _, _ in PLANNERS if runs[k][1]), None)
    if ref_key is None:
        return []
    _, trajs, _ = runs[ref_key]
    ys = [t[0][1] for t in trajs]
    lo, hi = min(ys), max(ys)
    targets = [hi + (lo - hi) * (i + 1) / (n_sites + 1) for i in range(n_sites)]
    n_avail = sum(1 for k, _, _ in PLANNERS if runs[k][1])

    sites = []
    for ty in targets:
        p_ref = min(trajs, key=lambda t: abs(t[0][1] - ty))[0]
        picks, starts = {}, []
        for key, _, _ in PLANNERS:
            cand = runs[key][1]
            if not cand:
                continue
            best = min(cand, key=lambda t: hdist(t[0], p_ref))
            picks[key] = best
            starts.append(best[0])
        if len(picks) < n_avail:
            continue
        spread = max((hdist(a, b) for i, a in enumerate(starts)
                      for b in starts[i + 1:]), default=0.0)
        if spread <= max_mismatch:
            sites.append(picks)
    return sites


# --------------------------------------------------------------------------- #
def main():
    rospy.init_node("benchmark_viz")
    default_dir = "/home/ros/exchange/RAL26_Benchmark_Multibag/results"
    if not os.path.isdir(default_dir) and \
            os.path.isdir("/home/ros/exchange/bench_results_RAL26"):
        default_dir = "/home/ros/exchange/bench_results_RAL26"
    folder = rospy.get_param("~results_dir", default_dir)
    mode = rospy.get_param("~mode", "live")
    playback = rospy.get_param("~playback", "internal")
    rate_mult = float(rospy.get_param("~rate", 1.0))
    loop = bool(rospy.get_param("~loop", True))
    trail = int(rospy.get_param("~trail", 0))
    fps = float(rospy.get_param("~fps", 20.0))
    n_sites = int(rospy.get_param("~n_sites", 8))
    max_mismatch = float(rospy.get_param("~max_mismatch", 0.10))
    stride = int(rospy.get_param("~stride", 25))
    width = float(rospy.get_param("~line_width", 0.06))
    frame = rospy.get_param("~frame_id", "map")
    world = rospy.get_param("~world", WORLD_DEFAULT)

    runs, pubs = {}, {}
    for key, nice, _ in PLANNERS:
        path = rospy.get_param("~" + key, None) or newest_metrics(folder, key)
        runs[key] = load_run(path)
        pubs[key] = rospy.Publisher("/benchmark_viz/" + key, MarkerArray,
                                    queue_size=1, latch=True)
        rospy.loginfo("[benchmark_viz] %-13s %s (%d trajectories)", nice,
                      os.path.basename(path) if path else "NOT FOUND",
                      len(runs[key][1]))
    pub_global = rospy.Publisher("/benchmark_viz/global_path", MarkerArray,
                                 queue_size=1, latch=True)
    pub_cols = rospy.Publisher("/benchmark_viz/columns", MarkerArray,
                               queue_size=1, latch=True)
    pub_drone = rospy.Publisher("/benchmark_viz/drone", MarkerArray,
                                queue_size=1, latch=True)

    if not any(runs[k][1] for k, _, _ in PLANNERS):
        rospy.logerr("[benchmark_viz] no trajectory data found in %s", folder)
        return

    # ---- columns (missing from fr_campus.bt, so drawn from the .world) ----
    cols = load_columns(world)
    ca = MarkerArray()
    for i, (cx, cy, cz, sx, sy, sz) in enumerate(cols):
        ca.markers.append(shape_marker((cx, cy, cz), "columns", i,
                                       (0.45, 0.45, 0.48), (sx, sy, sz),
                                       frame, shape=Marker.CUBE, alpha=0.85))
    pub_cols.publish(ca)
    rospy.loginfo("[benchmark_viz] %d columns published from %s",
                  len(cols), os.path.basename(world) if cols else "(not found)")

    # ---- global reference ----
    s = [GLOBAL_START[i] + MAP_OFFSET[i] for i in range(3)]
    g = [GLOBAL_GOAL[i] + MAP_OFFSET[i] for i in range(3)]
    ga = MarkerArray()
    ga.markers.append(line_marker([s, g], "global", 0, (0.6, 0.6, 0.6),
                                  width * 0.7, frame, alpha=0.85))
    ga.markers.append(shape_marker(s, "global", 1, (0.9, 0.9, 0.9),
                                   (0.4, 0.4, 0.4), frame))
    ga.markers.append(shape_marker(g, "global", 2, (0.15, 0.8, 0.3),
                                   (0.5, 0.5, 0.5), frame, shape=Marker.CUBE))
    ga.markers.append(text_marker(s, "start", "global", 3, frame))
    ga.markers.append(text_marker(g, "goal", "global", 4, frame))
    pub_global.publish(ga)

    # ---------------- static modes ----------------
    if mode in ("sites", "all"):
        if mode == "sites":
            sites = match_sites(runs, n_sites, max_mismatch)
            rospy.loginfo("[benchmark_viz] %d comparison sites (spread <= %.2f m)",
                          len(sites), max_mismatch)
            arrays = {k: MarkerArray() for k, _, _ in PLANNERS}
            for i, picks in enumerate(sites):
                for key, _, rgb in PLANNERS:
                    if key in picks:
                        arrays[key].markers.append(
                            line_marker(picks[key], key, i, rgb, width, frame))
                first = next(iter(picks.values()))
                arrays[PLANNERS[0][0]].markers.append(
                    text_marker(first[0], str(i + 1), "site_label", 1000 + i, frame))
        else:
            arrays = {}
            for key, _, rgb in PLANNERS:
                arr = MarkerArray()
                for i, pts in enumerate(runs[key][1][::stride]):
                    arr.markers.append(
                        line_marker(pts, key, i, rgb, width, frame, alpha=0.5))
                arrays[key] = arr
        for key, _, _ in PLANNERS:
            pubs[key].publish(arrays[key])
        rospy.loginfo("[benchmark_viz] static markers published (latched)")
        rospy.spin()
        return

    # ---------------- live mode ----------------
    t_lo = min(runs[k][0][0] for k, _, _ in PLANNERS if runs[k][0])
    t_hi = max(runs[k][0][-1] for k, _, _ in PLANNERS if runs[k][0])
    rospy.loginfo("[benchmark_viz] live playback over sim time [%.1f, %.1f] "
                  "(%.1f s), source=%s rate=%.2fx", t_lo, t_hi, t_hi - t_lo,
                  playback, rate_mult)

    ref_key = next(k for k, _, _ in PLANNERS if runs[k][1])
    t0_wall = rospy.get_time()
    hist = {k: [] for k, _, _ in PLANNERS}

    def tick(_evt):
        if playback == "clock":
            t = rospy.get_time()
        else:
            elapsed = (rospy.get_time() - t0_wall) * rate_mult
            t = t_lo + (elapsed % (t_hi - t_lo) if loop else min(elapsed, t_hi - t_lo))

        for key, _, rgb in PLANNERS:
            stamps, trajs, _ = runs[key]
            if not stamps:
                continue
            i = bisect.bisect_right(stamps, t) - 1
            arr = MarkerArray()
            arr.markers.append(delete_all(key))
            if i < 0:                       # this planner has not started yet
                pubs[key].publish(arr)
                continue
            if trail > 0:
                if not hist[key] or hist[key][-1] != i:
                    hist[key].append(i)
                    hist[key] = hist[key][-(trail + 1):]
                for j, idx in enumerate(hist[key][:-1]):
                    a = 0.10 + 0.25 * (j + 1) / max(len(hist[key]) - 1, 1)
                    arr.markers.append(line_marker(trajs[idx], key, 100 + j, rgb,
                                                   width * 0.7, frame, alpha=a))
            arr.markers.append(line_marker(trajs[i], key, 0, rgb, width, frame))
            pubs[key].publish(arr)

        stamps, _, poss = runs[ref_key]
        i = bisect.bisect_right(stamps, t) - 1
        da = MarkerArray()
        if i >= 0:
            da.markers.append(shape_marker(poss[i], "drone", 0, (0.95, 0.95, 0.95),
                                           (0.35, 0.35, 0.35), frame))
            da.markers.append(text_marker(poss[i], "t = %.1f s" % (t - t_lo),
                                          "drone", 1, frame, size=0.3))
        pub_drone.publish(da)

    rospy.Timer(rospy.Duration(1.0 / max(fps, 1.0)), tick)
    rospy.loginfo("[benchmark_viz] running. In RViz: Add -> By topic -> /benchmark_viz/*")
    rospy.spin()


if __name__ == "__main__":
    main()
