#usr/bin/env python3

from setuptools import find_packages, setup
import os
import glob
package_name = 'hb_task4b'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='e-yanthra',
    maintainer_email='aftaabahamed2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'controller = hb_task4b.bot_controller:main',

             'nextgoalpub = hb_task4b.nextGoalPub:main',
             'draw_pub = hb_task4b.publish_draw:main',
             'simple_draw_pub = hb_task4b.publish_draw_simple:main',

            #  'feedback1 = hb_task4b.feedback_2b_bot1:main',
            #  'feedback2 = hb_task4b.feedback_2b_bot2:main',
            #  'feedback3 = hb_task4b.feedback_2b_bot3:main',
            #  'feedback3_test = hb_task4b.feedback_2b_bot3_test:main',

             'pen_feedback = hb_task4b.pen_pose_feedback:main',

             'test_controller = hb_task4b.test_controller:main',
             'controller1 = hb_task4b.controller_bot1:main',
             'controller2 = hb_task4b.controller_bot2:main',
             'controller3 = hb_task4b.controller_bot3:main',
             
             'cam_test = hb_task4b.ros2_camera_test:main',
             'cam_rectify = hb_task4b.cam_rectify:main',
             'create_mtx = hb_task4b.create_p_matrix:main',
             'vis = hb_task4b.visualisation:main',

             'ol_sq = hb_task4b.open_loop_square:main', 
             'ol_tri = hb_task4b.open_loop_triangle:main',  
             'ol_circ = hb_task4b.open_loop_circle:main',  
             'ol_mv = hb_task4b.open_loop_move:main',           
        ],
    },
)
