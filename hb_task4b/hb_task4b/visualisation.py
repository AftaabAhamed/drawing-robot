import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import numpy as np
from my_robot_interfaces.msg import Goal    


class camera_node(Node):
    def __init__(self):
        super().__init__("visualisation")
        self.subscription = self.create_subscription(
            Image,
            '/Visualisation',
            self.image_callback,
            10)
        self.cv_bridge = cv_bridge.CvBridge()

        self.subscription1 = self.create_subscription(
            Goal,  
            f'hb_bot_1/goal',  
            self.goalCallBack_bot1,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        )  

        self.subscription2 = self.create_subscription(
            Goal,  
            f'hb_bot_2/goal',  
            self.goalCallBack_bot1,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        )  

        self.subscription3 = self.create_subscription(
            Goal,  
            f'hb_bot_3/goal',  
            self.goalCallBack_bot1,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        )  

        self.goal1_x = []
        self.goal1_y = []

        self.goal2_x = []
        self.goal2_y = []

        self.goal3_x = []
        self.goal3_y = []

    def goalCallBack_bot1(self, msg: Goal):
        self.goal1_x = msg.x
        self.goal1_y = msg.y
        self.goal1_theta = msg.theta
        self.goal1_len = len(self.goal1_x)
        
    def goalCallBack_bot2(self, msg: Goal):
        self.goal2_x = msg.x
        self.goal2_y = msg.y
        self.goal2_theta = msg.theta
        self.goal2_len = len(self.goal2_x)        
    
    def goalCallBack_bot3(self, msg: Goal):
        self.goal3_x = msg.x
        self.goal3_y = msg.y
        self.goal3_theta = msg.theta
        self.goal3_len = len(self.goal2_x)

    def image_callback(self,msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error('Error converting ROS Image to OpenCV image: %s' % str(e))
            return

        goal1_x  = self.goal1_x 
        goal1_y  = self.goal1_y 

        goal2_x  = self.goal2_x 
        goal2_y  = self.goal2_y 

        goal3_x  = self.goal3_x 
        goal3_y  = self.goal3_y 



        for x,y in zip(goal1_x,goal1_y):
            cv_image = cv2.circle(cv_image, (int(x),int(y)), 
                        radius = 1, color=(255, 0, 0), thickness = -1)
        for x,y in zip(goal2_x,goal2_y):
            cv_image = cv2.circle(cv_image, (int(x),int(y)), 
                        radius = 1, color=(255, 0, 0), thickness = -1)
        for x,y in zip(goal3_x,goal3_y):
            cv_image = cv2.circle(cv_image, (int(x),int(y)), 
                        radius = 1, color=(255, 0, 0), thickness = -1)
            print(x)

        cv2.imshow('output distorted' , cv_image)
     
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    output = camera_node()

    rclpy.spin(output)

    output.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
