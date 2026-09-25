"""Offline evaluation of tram_odometry_core against the GNSS reference (docs/contracts.md §4).

Reads bags with `rosbags`, feeds them to `pipeline.Odometry.step` in recording order (as
`ros2 bag play` does), cuts GNSS after the init window (D-005) and scores the estimates.
Entry point: `.venv/bin/python tools/eval/run_eval.py --split quick` (tools/eval/README.md).
"""
