"""Tram backup odometry core.

Pure Python + numpy (the only runtime dependency available in ros:humble-ros-base, D-004).
No rclpy here: ROS nodes in `tram_odometry` are thin wrappers around this library, and
`tools/eval` drives the same code faster than real time. Public types — `types.py`
(contract, docs/contracts.md).
"""
