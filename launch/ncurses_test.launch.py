#!/usr/bin/env python3

import os
import shutil

import launch
import launch_ros

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "mrs_uav_status"

    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    # Pick a terminal prefix only if a known terminal exists; fallback to none.
    prefix_cmd = None
    for candidate in (
        ["gnome-terminal", "--"],
        ["xterm", "-e"],
        ["konsole", "-e"],
    ):
        if shutil.which(candidate[0]):
            prefix_cmd = " ".join(candidate)
            break

    node_kwargs = dict(
        package=pkg_name,
        executable='MrsUavStatus_NcursesTest',
        namespace="",
        name='ncurses_test',
        output="screen",
        emulate_tty=True,
        env=proc_env,
    )

    if prefix_cmd:
        node_kwargs["prefix"] = prefix_cmd

    ld.add_action(launch_ros.actions.Node(**node_kwargs))

    return ld
