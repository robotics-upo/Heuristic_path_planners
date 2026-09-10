#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
benchmark_logger.py
===================

Planner-agnostic logger for the fr_campus benchmarking.

Every planner (C-3TO, Fast-Planner, EGO-Planner) publishes, once per
successful replan, a std_msgs/String containing a flat JSON object on the
common topic:

    /benchmark/metrics

This node subscribes to that single topic, appends one CSV row per message,
and prints an aggregate summary on shutdown. Because the transport is
std_msgs/String (available in every workspace), there is NO custom message
and therefore NO cross-workspace build dependency: this script runs from
either catkin_ws or ego_ws, or standalone with `python3`.

Expected JSON keys (missing keys are written as empty cells):
    planner, replan, x, y, z,
    plan_ms, min_dist, path_len, traj_T,
    max_v, max_a, max_j, dyn_valid, success

Validity is decided by THIS logger (not by the planners) as an obstacle-safety
check: a trajectory is valid when its minimum distance to obstacles stays at or
above dist_valid metres. Tunable without rebuilding any planner. The raw
max_v/max_a/max_j are still logged, so dynamic validity is recoverable offline.

Parameters (private):
    ~output_dir   : folder for the CSV
                    (default: /home/ros/exchange/RAL26_Benchmark_Multibag/results)
    ~run_tag      : optional label appended to the filename (default: "")
    ~metrics_topic: topic to subscribe to   (default: /benchmark/metrics)
    ~dist_valid   : min clearance for a valid trajectory, in metres (default: 0.4)

The CSV filename is decided from the FIRST message received:
    <output_dir>/<planner>_<run_tag>_<YYYYmmdd_HHMMSS>.csv
so you get one file per planner run without overwriting previous ones.
"""

import csv
import json
import os
import time
import datetime

import rospy
from std_msgs.msg import String

# Fixed CSV column order. Keys not present in a message are left blank.
COLUMNS = [
    "stamp",        # ROS time (SIMULATION time under use_sim_time) at reception
    "wall",         # wall-clock time at reception -- use THIS for replan rates,
                    # so the rate and plan_ms share one time base. A heavy planner
                    # slows the simulator, which inflates the sim-time rate.
    "planner",
    "replan",
    "x", "y", "z",
    "plan_ms",
    "min_dist",
    "path_len",
    "traj_T",
    "max_v", "max_a", "max_j",
    "max_v_axis", "max_a_axis",   # largest single-axis |v| / |a| (Fast/EGO's own per-axis limit)
    "avg_v", "avg_a",   # mean speed / mean acceleration over the (in-window) trajectory
    "valid",        # obstacle-safety validity: 1 if min_dist >= dist_valid
    "success",
]


class BenchmarkLogger(object):
    def __init__(self):
        # Benchmark suite lives in its own folder; the analysis scripts read
        # the CSVs from here. Legacy location kept as a fallback so a container
        # that has not been migrated yet still logs somewhere sensible.
        default_out = "/home/ros/exchange/RAL26_Benchmark_Multibag/results"
        if not os.path.isdir(default_out) and \
                os.path.isdir("/home/ros/exchange/bench_results_RAL26"):
            default_out = "/home/ros/exchange/bench_results_RAL26"
        out_dir = rospy.get_param("~output_dir", default_out)
        self.out_dir = os.path.expanduser(out_dir)
        self.run_tag = str(rospy.get_param("~run_tag", ""))
        topic = rospy.get_param("~metrics_topic", "/benchmark/metrics")

        # Validity is decided HERE (not by each planner) as an obstacle-safety
        # check: a trajectory is valid when its minimum distance to obstacles
        # stays at or above dist_valid metres. Tunable without rebuilding any
        # planner. The raw max_v/max_a/max_j are still logged, so dynamic
        # validity remains recoverable offline from the CSV.
        self.dist_valid = float(rospy.get_param("~dist_valid", 0.4))

        os.makedirs(self.out_dir, exist_ok=True)

        self.csv_file = None
        self.writer = None
        self.csv_path = None
        self.traj_file = None      # sampled trajectory geometry (separate CSV)
        self.traj_writer = None
        self.traj_path = None
        self.rows = []            # kept in memory for the shutdown summary
        self._shutdown = False    # set on shutdown so late callbacks stop writing

        self.sub = rospy.Subscriber(topic, String, self.cb, queue_size=100)
        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("[benchmark_logger] listening on %s", topic)
        rospy.loginfo("[benchmark_logger] output dir: %s", self.out_dir)

    # ------------------------------------------------------------------ #
    def _open_csv(self, planner):
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        tag = ("_" + self.run_tag) if self.run_tag else ""
        fname = "{}{}_{}.csv".format(planner, tag, ts)
        self.csv_path = os.path.join(self.out_dir, fname)
        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=COLUMNS)
        self.writer.writeheader()
        self.csv_file.flush()
        rospy.loginfo("[benchmark_logger] writing CSV: %s", self.csv_path)

        # Trajectory geometry goes to its own file: one row per sampled point,
        # so the metrics CSV stays one-row-per-replan and easy to analyse.
        self.traj_path = os.path.join(self.out_dir, "{}{}_{}_traj.csv".format(planner, tag, ts))
        self.traj_file = open(self.traj_path, "w", newline="")
        self.traj_writer = csv.writer(self.traj_file)
        self.traj_writer.writerow(["replan", "k", "x", "y", "z"])
        self.traj_file.flush()
        rospy.loginfo("[benchmark_logger] writing trajectory geometry: %s", self.traj_path)

    # ------------------------------------------------------------------ #
    def cb(self, msg):
        # Ignore any callback that fires during/after shutdown (the CSV is closed
        # in on_shutdown; a late message would otherwise write to a closed file).
        if self._shutdown or (self.csv_file is not None and self.csv_file.closed):
            return
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError) as e:
            rospy.logwarn("[benchmark_logger] bad JSON dropped: %s (%s)",
                          msg.data, e)
            return

        # Open the CSV lazily, naming it after the first planner seen.
        if self.writer is None:
            self._open_csv(str(data.get("planner", "unknown")))

        # Decide validity HERE as an obstacle-safety check: valid when the
        # minimum distance to obstacles stays at or above dist_valid metres.
        md = data.get("min_dist")
        if isinstance(md, (int, float)):
            data["valid"] = 1 if md >= self.dist_valid else 0

        row = {c: "" for c in COLUMNS}
        row["stamp"] = "%.6f" % rospy.get_time()
        row["wall"] = "%.6f" % time.time()
        for k, v in data.items():
            if k in row:
                row[k] = v
        self.writer.writerow(row)
        self.csv_file.flush()
        self.rows.append(data)

        # Sampled trajectory geometry -> separate CSV (never into the metrics row)
        traj = data.get("traj")
        if isinstance(traj, list) and self.traj_writer is not None:
            rep = data.get("replan", "")
            for k, pt in enumerate(traj):
                if isinstance(pt, (list, tuple)) and len(pt) == 3:
                    self.traj_writer.writerow([rep, k, pt[0], pt[1], pt[2]])
            self.traj_file.flush()

    # ------------------------------------------------------------------ #
    def on_shutdown(self):
        # Stop new callbacks first, then close the file, so cb() never writes
        # to a closed file (avoids the "I/O operation on closed file" race).
        self._shutdown = True
        try:
            self.sub.unregister()
        except Exception:
            pass
        if self.csv_file is not None and not self.csv_file.closed:
            self.csv_file.close()
        if self.traj_file is not None and not self.traj_file.closed:
            self.traj_file.close()

        n = len(self.rows)
        if n == 0:
            rospy.loginfo("[benchmark_logger] no messages received; "
                          "nothing to summarize.")
            return

        def col(key):
            return [r[key] for r in self.rows if isinstance(r.get(key), (int, float))]

        def stats(vals):
            if not vals:
                return (float("nan"), float("nan"), float("nan"))
            s = sorted(vals)
            mean = sum(s) / len(s)
            median = s[len(s) // 2]
            return (mean, median, s[-1])

        planner = self.rows[0].get("planner", "unknown")
        pm_mean, pm_med, pm_max = stats(col("plan_ms"))
        md_mean, md_med, md_min = (stats(col("min_dist"))[0],
                                   stats(col("min_dist"))[1],
                                   min(col("min_dist")) if col("min_dist") else float("nan"))
        valid_ok = sum(1 for r in self.rows if r.get("valid") == 1)
        succ = sum(1 for r in self.rows if r.get("success") == 1)

        print("\n" + "=" * 55)
        print(" BENCHMARK SUMMARY  ({})".format(planner))
        print("=" * 55)
        print(" replans logged     : {}".format(n))
        print(" plan_ms  mean/med/max : {:.2f} / {:.2f} / {:.2f}".format(pm_mean, pm_med, pm_max))
        print(" min_dist mean/med/min : {:.3f} / {:.3f} / {:.3f}".format(md_mean, md_med, md_min))
        print(" valid (min_dist>={:.2f}m): {}/{}  ({:.1f}%)".format(
            self.dist_valid, valid_ok, n, 100.0 * valid_ok / n))
        print(" success             : {}/{}  ({:.1f}%)".format(succ, n, 100.0 * succ / n))
        print(" CSV                 : {}".format(self.csv_path))
        if self.traj_path:
            print(" Trajectory geometry : {}".format(self.traj_path))
        print("=" * 55 + "\n")


def main():
    rospy.init_node("benchmark_logger")
    BenchmarkLogger()
    rospy.spin()


if __name__ == "__main__":
    main()
