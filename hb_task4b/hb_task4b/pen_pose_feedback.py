import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
import cv2
import cv_bridge
import numpy as np
import math 
import matplotlib.pyplot as plt

class camera_node(Node):
    def __init__(self):
        super().__init__("Feedback_node")
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10)
        
        self.cv_bridge = cv_bridge.CvBridge()

        self.pen1_pub = self.create_publisher(Pose2D, '/pen1_pose', 10)
        self.pen2_pub = self.create_publisher(Pose2D, '/pen2_pose', 10)
        self.pen3_pub = self.create_publisher(Pose2D, '/pen3_pose', 10)
        
        self.vis_pub = self.create_publisher(Image, '/Visualisation', 10)

        self.pose_msg = Pose2D()
        # with open('/home/e-yanthra/eyrc_hb/hb_task4_ws/warp.pkl','rb') as f:
        #     self.warp_matrix = pkl.load(f)
        self.warp_matrix = \
              np.load('/home/e-yanthra/eyrc_hb/hb_task5_ws/warp_mtx.npy')
    
    def publish_pen_pose(self,key,pose_msg):
        if key == "bot1":
            self.pen1_pub.publish(pose_msg)
        elif key == "bot2":
            self.pen2_pub.publish(pose_msg)
        elif key == "bot3":
            self.pen3_pub.publish(pose_msg)
        else:
            print("error in publishing")
        pass

    def get_pos(self, bot_corners , img):

        # bot_corners=list(all_coordinates[key]) 
        
        delx = bot_corners[0][0] - bot_corners[1][0]
        dely = bot_corners[0][1] - bot_corners[1][1]

        bot_theta = math.atan2(dely,delx)

        bot_x = (bot_corners[0][0]+bot_corners[2][0])/2
        bot_y = (bot_corners[0][1]+bot_corners[2][1])/2
        penx  = (bot_x - (25*math.sin(-bot_theta))) 
        peny  = (bot_y - (25*math.cos(-bot_theta)))

        print(penx,peny,bot_theta)
        
        pose_msg = Pose2D()

        pose_msg.x = penx
        pose_msg.y = peny
        pose_msg.theta = bot_theta

        image = cv2.circle(img.copy(), (int(bot_x),int(bot_y)),
                    radius = 2, color=(250, 0, 0), thickness = -1)

        image = cv2.circle(image, (int(penx),int(peny)), 
                    radius = 5, color=(0, 0, 0), thickness = -1)

        return  pose_msg ,image
        


    def image_callback(self,msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error('Error converting ROS Image to OpenCV image: %s' % str(e))
            return
        
        # cam_mtx = np.asarray([[1257.54957,    0.     , 1115.10304,],       #1080p vidsteam
        #                       [0.     , 1265.15677,  476.10181,],     
        #                       [0.     ,    0.     ,    1.     ]]) 
        

        # dist_mtx = np.asarray([-0.540714, 0.142376, 0.009046, -0.062507, 0.000000])


        cam_mtx = np.asarray([[438.07737,   0.     , 304.54829,],       #480p vidsteam
                              [  0.     , 436.95667, 216.15402,],     
                              [ 0.     ,   0.     ,   1.     ]]) 
        


        dist_mtx = np.asarray([-0.371341, 0.130125, 0.002396, -0.001402, 0.000000])


        h,  w = cv_image.shape[:2]

        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam_mtx, dist_mtx, (w,h), 1, (w,h))


        dst = cv2.undistort(cv_image, cam_mtx, dist_mtx, None, newcameramtx)

        x, y, w, h = roi
        image = dst[y:y+h, x:x+w] 
        image = cv2.warpPerspective(image, self.warp_matrix, (500, 500))  
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
        
        ret, thresh = cv2.threshold(image, 120, 255, cv2.THRESH_BINARY) 


        ARUCO_DICT = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50  
            }
        key_ids = {"arena_tl": 8,"arena_tr": 10,"arena_bl": 4, "arena_br": 12 ,"bot1": 1, "bot2": 2, "bot3": 3}

        all_coordinates = {}

        arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT["DICT_4X4_50"])

        arucoParams = cv2.aruco.DetectorParameters()

        corners,ids,rejected = cv2.aruco.detectMarkers(thresh,arucoDict,parameters=arucoParams)
        print(len(corners))
        
        # cv2.imshow("vis" , image)

        
        if len(corners) > 0:


            try:

                ids = ids.flatten()
                for (markerCorner, markerId) in zip(corners ,ids):
                    corners = markerCorner.reshape((4,2))
                    # print(corners)
                    topleft ,topright,bottomright,bottomleft = corners

                    cx = (topleft[0] + topright[0])/2.0
                    cy = (topleft[1] + bottomleft[1])/2.0
                    cv2.putText(image,str(markerId),(int(cx),int(cy)),
                                cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
                
                    if markerId == key_ids["bot1"]:
                        all_coordinates.update({"bot1":corners})
                    elif markerId == key_ids["bot2"]:
                        all_coordinates.update({"bot2":corners})
                    elif markerId == key_ids["bot3"]:
                        all_coordinates.update({"bot3":corners})

                for key in all_coordinates:
                    # print(key)
                    bot_corners=list(all_coordinates[key]) 
                    
                    pose_msg , image = self.get_pos(bot_corners=bot_corners, img=image)

                    self.publish_pen_pose(key , pose_msg=pose_msg)

                self.vis_pub.publish(self.cv_bridge.cv2_to_imgmsg(image, "passthrough"))


            except Exception as e:
                print(f"somethings qrong:{e}")
                pass

      

def main(args=None):
    rclpy.init(args=args)

    output = camera_node()

    rclpy.spin(output)

    output.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
