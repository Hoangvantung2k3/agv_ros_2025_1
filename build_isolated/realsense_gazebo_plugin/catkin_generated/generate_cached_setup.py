# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/noetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/noetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/tung/agv_ros_2025_1/devel_isolated/realsense2_camera;/home/tung/agv_ros_2025_1/devel_isolated/openslam_gmapping;/home/tung/agv_ros_2025_1/devel_isolated/carmodel_nav;/home/tung/agv_ros_2025_1/devel_isolated/move_base_msgs;/home/tung/agv_ros_2025_1/devel_isolated/map_msgs;/home/tung/agv_ros_2025_1/devel_isolated/ig_lio;/home/tung/agv_ros_2025_1/devel_isolated/livox_ros_driver;/home/tung/agv_ros_2025_1/devel_isolated/carmodel_teleop;/home/tung/agv_ros_2025_1/devel_isolated/carmodel_slam;/home/tung/agv_ros_2025_1/devel_isolated/carmodel_description;/opt/ros/noetic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python3/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/tung/agv_ros_2025_1/devel_isolated/realsense_gazebo_plugin/env.sh')

output_filename = '/home/tung/agv_ros_2025_1/build_isolated/realsense_gazebo_plugin/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
