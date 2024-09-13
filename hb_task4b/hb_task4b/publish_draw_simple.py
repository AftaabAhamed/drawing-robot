#######################################     DO NOT MODIFY THIS  FILE     ##########################################

import numpy as np
import matplotlib.pyplot as plt
from my_robot_interfaces.srv import NextGoal             
import rclpy
from rclpy.node import Node  

from my_robot_interfaces.msg import Goal           
from my_robot_interfaces.msg import Shape  


class ServiceNode(Node):

    def __init__(self):
        super().__init__('service_node')

        self.publish_goal_1 = self.create_publisher(Goal, 'hb_bot_1/goal', 10)
        self.publish_goal_2 = self.create_publisher(Goal, 'hb_bot_2/goal', 10)
        self.publish_goal_3 = self.create_publisher(Goal, 'hb_bot_3/goal', 10)

        x_given_hex = [200, 175, 125, 100, 125, 175, 200]
        y_given_hex = [150, 200, 200, 150, 100, 100, 150]
        x_interp_hex =[]
        y_interp_hex =[]

        for i in range(len(x_given_hex)-2):
            print("a")
            x_interp_seg = np.linspace(x_given_hex[i],x_given_hex[i+1],num=14)
            print(x_interp_hex)

            for j in range (len(x_interp_seg)-2):
                print("b")
                y = (x_interp_seg[j]-x_given_hex[i])*((y_given_hex[i] - y_given_hex[i+1])/(x_given_hex[i] - x_given_hex[i+1])) + y_given_hex[i]
                y_interp_hex.append(float(y))
                x_interp_hex.append(float(x_interp_seg[j]))
                

        self.bot1_pnts = [x_interp_hex,y_interp_hex]
        self.bot2_pnts = [x_interp_hex,y_interp_hex]
        self.bot3_pnts = [x_interp_hex,y_interp_hex]


        # print(len(self.bot1_pnts))
        # print(len(self.bot2_pnts))
        # print(len(self.bot3_pnts))
        

        self.timer = self.create_timer(5.0, self.publish_shapes)

    def publish_shapes(self):
        msg_bot_1 = Goal()
        msg_bot_2 = Goal()
        msg_bot_3 = Goal()
        msg_bot_2.bot_id = 2
        msg_bot_1.bot_id = 1
        # print(self.bot1_pnts[0])
        msg_bot_2.x = self.bot2_pnts[0]
        msg_bot_2.y = self.bot2_pnts[1]

        msg_bot_1.x = self.bot1_pnts[0]
        msg_bot_1.y = self.bot1_pnts[1]

        # msg_bot_1.x = [250.0,125.0]
        # msg_bot_1.y = [250.0,125.0]

        msg_bot_2.theta = 0.0
        msg_bot_1.theta = 0.0

        # print([p[0] for p in self.bot1_pnts])
        # print([p[1] for p in self.bot1_pnts])
        # print(self.bot2_pnts)        

        # self.publish_goal_1.publish(msg_bot_1)
        self.publish_goal_1.publish(msg_bot_1)
        self.publish_goal_2.publish(msg_bot_2)


        # self.publish_goal_3.publish(msg_bot_3)




def main(args=None):
    rclpy.init(args=args)
    service_node = ServiceNode()

    rclpy.spin(service_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
