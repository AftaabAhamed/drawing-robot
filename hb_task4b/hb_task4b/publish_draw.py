#######################################     DO NOT MODIFY THIS  FILE     ##########################################

import numpy as np
import matplotlib.pyplot as plt
from my_robot_interfaces.srv import NextGoal             
import rclpy
from rclpy.node import Node  
import random
import time
from my_robot_interfaces.msg import Goal           
from my_robot_interfaces.msg import Shape  
import cv2,math         


class ServiceNode(Node):

    def __init__(self):
        super().__init__('service_node')
        image  = cv2.imread("/home/e-yanthra/eyrc_hb/hb_task_2_ws/src/eYRC-2023_Hologlyph_Bots/hb_task2b/hb_task2b/smiley.jpg")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
        ret, thresh = cv2.threshold(gray, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_LIST, 
                                               cv2.CHAIN_APPROX_NONE) 
        
        pnts_list = []

        for contour_index in range(len(contours)-1):

            contour = contours[contour_index]

            for pnts in contour:

                x,y = pnts[0]
                pnts_list.append((float(x),float(y)))
                # print(pnts[0])
                
                # image2 = cv2.circle(image.copy(), (x,y), radius=2, 
                #                     color=(250, 0, 0), thickness=-1)
                
                # cv2.imshow('plot path',image2)
                # cv2.waitKey(0)
            pnts_list.append((-10,-10)) 
        self.publish_goal_1 = self.create_publisher(Goal, 'hb_bot_1/goal', 10)
        self.publish_goal_2 = self.create_publisher(Goal, 'hb_bot_2/goal', 10)
        self.publish_goal_3 = self.create_publisher(Goal, 'hb_bot_3/goal', 10)
        
        eq_no = len(pnts_list)//3

        self.bot1_pnts = np.array(pnts_list[0:eq_no])
        self.bot2_pnts = np.array(pnts_list[eq_no:(2*eq_no)])
        self.bot3_pnts = np.array(pnts_list[(2*eq_no):(3*eq_no)])


        print(len(self.bot1_pnts))
        print(len(self.bot2_pnts))
        print(len(self.bot3_pnts))
        

        self.timer = self.create_timer(5.0, self.publish_shapes)

    def publish_shapes(self):
        msg_bot_1 = Goal()
        msg_bot_2 = Goal()
        msg_bot_3 = Goal()

        print([p[0] for p in self.bot1_pnts])
        print([p[1] for p in self.bot1_pnts])
        # print(self.bot2_pnts)
        

        msg_bot_1.bot_id = 1
        msg_bot_1.x = [p[0] for p in self.bot1_pnts]
        msg_bot_1.y = [p[1] for p in self.bot1_pnts]
        msg_bot_1.theta = 0.0

        msg_bot_2.bot_id = 2
        msg_bot_2.x = [p[0] for p in self.bot2_pnts]
        msg_bot_2.y = [p[1] for p in self.bot2_pnts]
        msg_bot_2.theta = 0.0

        msg_bot_3.bot_id = 3
        msg_bot_3.x = [p[0] for p in self.bot3_pnts]
        msg_bot_3.y = [p[1] for p in self.bot3_pnts]
        msg_bot_3.theta = 0.0


        self.publish_goal_1.publish(msg_bot_1)
        self.publish_goal_2.publish(msg_bot_2)
        self.publish_goal_3.publish(msg_bot_3)




def main(args=None):
    rclpy.init(args=args)
    service_node = ServiceNode()

    rclpy.spin(service_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
