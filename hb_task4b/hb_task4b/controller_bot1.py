#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 5a of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''


# Team ID:		[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		feedback.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.msg import Goal    
import numpy as np
from geometry_msgs.msg import Pose2D, Wrench   
from std_msgs.msg import Int32,Bool,Float32
from geometry_msgs.msg import Vector3   
   
# from get_pwm import pwm_provider


class pwm_provider():
    def __init__(self,path_list) -> None:

        file21 = open(path_list[0], 'r')
        file22 = open(path_list[1], 'r')
        file23 = open(path_list[2], 'r')
        m21 = []
        m22 = []
        m23 = []

        maps = [m21,m22,m23] 
        files = [file21,file22,file23]
        count = 0
        for m,file in zip(maps,files): 
            while True:
                count += 1
                line = file.readline()                
                if not line:
                    break
                line  = line.strip()
                pwm , vel = line.split()
                m.append([float(vel)])
                
            m = np.asarray(m)
            file.close()

        m21 = np.asarray(m21)
        m22 = np.asarray(m22)
        m23 = np.asarray(m23)
        maps = [m21,m22,m23]
        minlist = [] 
        maxlist = []
        for m in maps:
            minlist.append(m[:90].min())
            maxlist.append(m[90:].max())
        
        min_ = max(minlist)
        max_ = min(maxlist)

        for m in maps:
            m[:90] = (m[:90]/abs(min_))
            m[90:] = (m[90:]/abs(max_))

        self.maps = maps


    def get_pwm(self,ik):
        m1  = self.maps[0]
        m2  = self.maps[1]
        m3  = self.maps[2]
        
        diff = abs(m1 - (ik[0]))
        pwm1 = diff.argmin() + 1
        diff = abs(m2 - (ik[1]))
        pwm2 = diff.argmin() + 1 
        diff = abs(m3 - (ik[2]))
        pwm3 = diff.argmin() + 1 
        if ik[0] == 0:
            pwm1 = 90
        if ik[1] == 0:
            pwm2 = 90
        if ik[2] == 0:
            pwm3 = 90

        return float(pwm1) ,float(pwm2), float(pwm3)




class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller_1')

        self.bot_no = 1
        self.penstate  = Bool()
        self.penstate.data = False
        self.ppnts = [1]

        self.subscription=self.create_subscription(Pose2D,f'/pen{self.bot_no}_pose',self.pose_cb,10)

        self.publisher = self.create_publisher(Vector3, f'/cmd_vel/bot{self.bot_no}', 10)
        self.pen_publisher = self.create_publisher(Bool, f'/penstate_bot{self.bot_no}', 10)
        

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0 
        self.wheel = Vector3()

        self.kpx = 0.08
        self.kpy = 0.08
        self.kp_theta = 0.08

        self.limit_x = 0.6
        self.limit_y = 0.4
        self.limit_theta = .7

        self.goal_index = 0      

        # Initialise the required variables
        self.goal_x = []
        self.goal_y = []
        self.goal_theta = 0.0
        self.goal_len = 0
        self.goal_end_flag = False

        

        # Initialze Publisher and Subscriber
        # NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	    #	Use the below given topics to generate motion for the robot.
	    #   /hb_bot_1/left_wheel_force,
	    #   /hb_bot_1/right_wheel_force,
	    #   /hb_bot_1/left_wheel_force

        #Similar to this you can create subscribers for hb_bot_2 and hb_bot_3
        self.subscription_goal = self.create_subscription(
            Goal,  
            f'hb_bot_{self.bot_no}/goal',  
            self.goalCallBack,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        )  

        self.subscription  # Prevent unused variable warning

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

    def inverse_kinematics(self,vx,vy,w):
        transform_matrix=np.array([[-0.33,  0.58, 0.33],
                                   [-0.33, -0.58, 0.33],
                                   [ 0.67,     0, 0.33]])
        vel_chassis_array=np.array([[vx],
                                    [vy],
                                    [w]])
        wheel_vel=np.transpose(np.dot(transform_matrix, vel_chassis_array))
        wheel_vel.flatten()
        # self.get_logger().info(f'error:{wheel_vel}')


        return wheel_vel[0][0], wheel_vel[0][1], wheel_vel[0][2]
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Use the target velocity you calculated for the robot in previous task, and
        #	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        #	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ############################################

    def goalCallBack(self, msg: Goal):
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.goal_theta = msg.theta
        self.goal_len = len(self.goal_x)

    def pose_cb(self, msg: Pose2D):
        self.pose_x = msg.x
        self.pose_y = msg.y
        self.pose_theta = msg.theta
        
    def kpx_cb(self, msg: Pose2D):
        self.pose_x = msg.x
        self.pose_y = msg.y
        self.pose_theta = msg.theta
        
    def kpy_cb(self, msg: Pose2D):
        self.pose_x = msg.x
        self.pose_y = msg.y
        self.pose_theta = msg.theta

    def kptheta_cb(self, msg: Pose2D):
        self.pose_x = msg.x
        self.pose_y = msg.y
        self.pose_theta = msg.theta
                
def main(args=None):
    rclpy.init(args=args)

    hb_controller = HBController()


    bot_path_list = ['/home/e-yanthra/eyrc_hb/hb_task4_ws/src/hb_task4b/hb_task4b/motor_cal/11.txt',
                     '/home/e-yanthra/eyrc_hb/hb_task4_ws/src/hb_task4b/hb_task4b/motor_cal/12.txt',
                     '/home/e-yanthra/eyrc_hb/hb_task4_ws/src/hb_task4b/hb_task4b/motor_cal/13.txt']

    
    pwm = pwm_provider(bot_path_list)


    # hb_controller = HBController()
    # hb_controller.bot_no = botno
    while (hb_controller.goal_len == 0):
        rclpy.spin_once(hb_controller)
        print(hb_controller.goal_x)
    # Main loop
    while rclpy.ok():            
                        
            x_goal   = hb_controller.goal_x[hb_controller.goal_index] 
            y_goal   = hb_controller.goal_y[hb_controller.goal_index] 

            theta_goal  = hb_controller.goal_theta

            # while theta_goal>1.57:
            #     theta_goal = theta_goal - 3.14


            hb_controller.goal_end_flag = True if hb_controller.goal_index == (hb_controller.goal_len-1) else False
            # hb_controller.get_logger().info(
            #     f'next:goal {hb_controller.index}, {x_goal}, {y_goal}, {theta_goal}')
            ####################################################
            error_x = x_goal - hb_controller.pose_x
            error_y = y_goal - hb_controller.pose_y
            error_theta = theta_goal - hb_controller.pose_theta  
            error_mtx = np.asarray([error_x, error_y])
            rot_mtx = np.asarray([[math.cos(error_theta) , -math.sin(error_theta)],
                                  [math.sin(error_theta) ,  math.cos(error_theta)]])
            
            ret_mtx = np.matmul(rot_mtx,error_mtx)
            # print(ret_mtx)
            error_x = ret_mtx[0]
            error_y = ret_mtx[1]
            # print(f"index={hb_controller.goal_index}  goal:{x_goal},{y_goal},{theta_goal}")           
            # Calculate Error from feedback
            # hb_controller.get_logger().info(f'error:{error_x}, {error_y}, {error_theta}')

            # Change the frame by using Rotation Matrix (If you find it required)
            cmd_vel_x = hb_controller.kpx*(-error_x)
            cmd_vel_y = hb_controller.kpy*(error_y)
            cmd_vel_theta = hb_controller.kp_theta*(-error_theta)
            # print(f"index={hb_controller.goal_index}  vel:{error_x},{error_y},{error_theta}")           

            # Calculate the required velocity of bot for the next iteration(s)
            # cmd_vel_x = cmd_vel_x/hb_controller.limit_x
            # cmd_vel_y = cmd_vel_y/hb_controller.limit_y
            # cmd_vel_theta = cmd_vel_theta/hb_controller.limit_theta

            if abs(cmd_vel_x) > hb_controller.limit_x :
                cmd_vel_x = hb_controller.limit_x*(cmd_vel_x/abs(cmd_vel_x))

            if abs(cmd_vel_y) > hb_controller.limit_y :
                cmd_vel_y = hb_controller.limit_y*(cmd_vel_y/abs(cmd_vel_y))

            if abs(cmd_vel_theta) > hb_controller.limit_theta :
                cmd_vel_theta = hb_controller.limit_theta*(cmd_vel_theta/abs(cmd_vel_theta))
            


            # cmd_vel_x = cmd_vel_x/hb_controller.limit_x
            # cmd_vel_y = cmd_vel_y/hb_controller.limit_y
            # cmd_vel_theta = cmd_vel_theta/hb_controller.limit_theta

            # if error_theta > 0.3:
            #     cmd_vel_x = -0.1
            #     cmd_vel_y = 

            print(f"index={hb_controller.goal_index}  vel:{cmd_vel_x},{cmd_vel_y},{cmd_vel_theta}")           
            print(f"index={hb_controller.goal_index}  error:{error_x},{error_y},{error_theta}")           


            s3,s2,s1 = pwm.get_pwm(hb_controller.inverse_kinematics(cmd_vel_x,cmd_vel_y,cmd_vel_theta))
            hb_controller.wheel.x=(s3)
            hb_controller.wheel.y=(s2)
            hb_controller.wheel.z=(s1)               
            
            # Find the required force vectors for individual wheels from it.(Inverse Kinematics)
            hb_controller.publisher.publish(hb_controller.wheel)
            hb_controller.pen_publisher.publish(hb_controller.penstate)

                        
            # Apply appropriate force vectors
            
            # Modify the condition to Switch to Next goal (given position in pixels instead of meters)
            if ((abs(error_x)<10) and (abs(error_y)<10) ):  #and (abs(error_theta)<0.2)  
                
                s3,s2,s1 = pwm.get_pwm(hb_controller.inverse_kinematics(0,0,0))

                hb_controller.wheel.x=s3
                hb_controller.wheel.y=s2
                hb_controller.wheel.z=s1
                hb_controller.publisher.publish(hb_controller.wheel)
                print(hb_controller.goal_index)
                # time.sleep(0.2)


                ############     DO NOT MODIFY THIS       #########
                hb_controller.goal_index += 1
                if hb_controller.goal_end_flag == True :
                    hb_controller.goal_index = 0
                hb_controller.get_logger().info(
                f'next:goal {hb_controller.goal_index}, {x_goal}, {y_goal}, {theta_goal}')
                if hb_controller.goal_index == 1 :
                    hb_controller.penstate.data = (not hb_controller.penstate.data)
                    print("reached first point")
                    hb_controller.pen_publisher.publish(hb_controller.penstate)
                ####################################################

            # Spin once to process callbacks
            rclpy.spin_once(hb_controller)
            
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
