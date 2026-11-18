#!/usr/bin/python3
"""Exec a ROS launch/run command with SIGINT restored for background jobs."""
import os
import signal
import sys

if len(sys.argv) < 4 or sys.argv[1] not in ('launch', 'run'):
    raise SystemExit('Usage: exec_ros2.py {launch|run} PACKAGE ENTRY [ARGS...]')
signal.signal(signal.SIGINT, signal.SIG_DFL)
os.execvp('ros2', ['ros2', *sys.argv[1:]])
