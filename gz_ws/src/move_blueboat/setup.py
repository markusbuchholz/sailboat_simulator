# Copyright 2024, Markus Buchholz

from setuptools import setup
import os
from glob import glob

package_name = 'move_blueboat'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include any other directories here
    ],
    install_requires=['setuptools', 'tf2_ros'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Your package description',
    license='Your license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Update this if you have any executables
            'robot_controller = move_blueboat.robot_controller:main',
            'run_blueboat_mission = move_blueboat.run_blueboat_mission:main',
            'run_mpc = move_blueboat.run_mpc_blueboat_mission:main',
            'run_horizion_mpc = move_blueboat.run_horizon_mpc_blueboat_mission:main',
            'thrust_allocator = move_blueboat.thrust_allocator:main',
            'run_mission = move_blueboat.run_mission:main',
            'dp_run = move_blueboat.dp_run:main',
            'thread_dp_run_with_heading = move_blueboat.threading_dp_run_with_heading:main',
            'watchdog = move_blueboat.watchdog:main',
            'key_controller = move_blueboat.key_controller:main',
            'gui_joy_controller = move_blueboat.gui_joy_controller:main',
            'joy_sim_controller = move_blueboat.joy_sim_controller:main',
            'mission_planner_blueboat = move_blueboat.mission_planner_blueboat:main',
            'way_controller_for_boat_mission = move_blueboat.way_controller_for_boat_mission:main',
            'pid_controller_for_boat_mission = move_blueboat.pid_controller_for_boat_mission:main',
            'mpc_controller_for_boat_mission = move_blueboat.mpc_controller_for_boat_mission:main',
            'blueboat_tf_broadcaster = move_blueboat.blueboat_tf_broadcaster:main',
            'run_way_controller_for_boat_mission = move_blueboat.run_way_controller_for_boat_mission:main',
            'run_navsat_way_controller_for_boat_mission = move_blueboat.run_navsat_way_controller_for_boat_mission:main',
            'run_gps_true_way_controller_for_boat_mission = move_blueboat.run_gps_true_way_controller_for_boat_mission:main',
            'gps_topics = move_blueboat.gps_topics:main',
            'run_gps_path_planner_waypoints = move_blueboat.run_gps_path_planner_waypoints:main',
            'dvl_run_boat_waypoint_mission = move_blueboat.dvl_run_boat_waypoint_mission:main',
            'odom_wavetank = move_blueboat.odom_wavetank:main',
            'odom_watchdog = move_blueboat.odom_watchdog:main',
            'odom_yaw = move_blueboat.odom_yaw:main',
            'mission_dp = move_blueboat.mission_dp:main',
            'mission_dp_one = move_blueboat.mission_dp_one:main',
            'mission_dp_modes = move_blueboat.mission_dp_modes:main',
            'beacon_dvl_run_boat_waypoint = move_blueboat.beacon_dvl_run_boat_waypoint:main',
            'dp_beacon_dvl_run_boat_waypoint = move_blueboat.dp_beacon_dvl_run_boat_waypoint:main',
            'linker_offset = move_blueboat.linker_offset:main',
            'x_linker_offset = move_blueboat.x_linker_offset:main',
            'wp_rov = move_blueboat.wp_rov:main',
            'wp_velo_rov = move_blueboat.wp_velo_rov:main',
            'blue_linker_monitor = move_blueboat.blue_linker_monitor:main',
            'wp_rov_to_boat = move_blueboat.wp_rov_to_boat:main',
            'dynamic_rov_to_boat = move_blueboat.dynamic_rov_to_boat:main',
            'pid_boat_follow_rov = move_blueboat.pid_boat_follow_rov:main',
            'mpc_boat_follow_rov = move_blueboat.mpc_boat_follow_rov:main',
            'mpc_dp_boat_follow_rov = move_blueboat.mpc_dp_boat_follow_rov:main',
            'mpc_dp_boat_follow_rov_without = move_blueboat.mpc_dp_boat_follow_rov_without:main',
            'dynamic_rov_to_boat_sim_1 = move_blueboat.dynamic_rov_to_boat_sim_1:main',
            'dynamic_rov_to_boat_sim_2 = move_blueboat.dynamic_rov_to_boat_sim_2:main',
            'dynamic_rov_to_boat_sim_2_rough = move_blueboat.dynamic_rov_to_boat_sim_2_rough:main',
            'dynamic_rov_to_boat_sim_3_meas = move_blueboat.dynamic_rov_to_boat_sim_3_meas:main',
            'dynamic_rov_to_boat_wavetank_1 = move_blueboat.dynamic_rov_to_boat_wavetank_1:main',
            'dynamic_rov_to_boat_wavetank_2 = move_blueboat.dynamic_rov_to_boat_wavetank_2:main',
            'dynamic_rov_to_boat_wavetank_3 = move_blueboat.dynamic_rov_to_boat_wavetank_3:main',
            'dynamic_rov_to_boat_wavetank_4 = move_blueboat.dynamic_rov_to_boat_wavetank_4:main',
            'dynamic_rov_to_boat_wavetank_5 = move_blueboat.dynamic_rov_to_boat_wavetank_5:main',
            'mpc_boat_follow_rov_wavetank_sim = move_blueboat.mpc_boat_follow_rov_wavetank_sim:main',
            'dynamic_rov_to_boat_sim_paper = move_blueboat.dynamic_rov_to_boat_sim_paper:main',
            
            

            
            
            
            
            
            
            

        ],
    },
)
