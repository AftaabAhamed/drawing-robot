import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import numpy as np
import pickle as pkl


class camera_node(Node):
    def __init__(self):
        super().__init__("Camera_output_node")

        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10)
        self.cv_bridge = cv_bridge.CvBridge()

    def get_perspective_warp_matrix(self,image):

        ARUCO_DICT = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        }

        key_ids = {"arena_tl": 8,"arena_tr": 10,"arena_bl": 4,"arena_br": 12,"bot1": 1}

        all_coordinates = {}

        arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT["DICT_4X4_50"])

        arucoParams = cv2.aruco.DetectorParameters()
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
        
        ret, thresh = cv2.threshold(image, 110, 255, cv2.THRESH_BINARY) 

        corners = []
        while (len(corners) < 4):
            corners,ids,rejected = cv2.aruco.detectMarkers(thresh,arucoDict,
                                                        parameters=arucoParams)
            print(len(corners))
            # corners = corners.reshape((4,2))

            # cv2.circle(thresh,(corners[0]), 63, (0,0,255), -1)
            cv2.imshow("bkb",thresh)



        if len(corners) > 0:
            print(ids)

            ids = ids.flatten()
            for (markerCorner, markerId) in zip(corners ,ids):
                corners = markerCorner.reshape((4,2))
                # print(corners)
                topleft ,topright,bottomright,bottomleft = corners

                cx = (topleft[0] + topright[0])/2.0
                cy = (topleft[1]+bottomleft[1])/2.0
                cv2.putText(image,str(markerId),(int(cx),int(cy)),
                            cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)

                if markerId == key_ids["arena_tl"]:
                    all_coordinates.update({"tl":corners})
                    all_coordinates.update({"arena_tl_c":(cx,cy)})

                elif markerId == key_ids["arena_tr"]:
                    all_coordinates.update({"tr":corners})
                    all_coordinates.update({"arena_tr_c":(cx,cy)})

                elif markerId == key_ids["arena_bl"]:
                    all_coordinates.update({"bl":corners}) 
                    all_coordinates.update({"arena_bl_c":(cx,cy)})

                elif markerId == key_ids["arena_br"]:
                    all_coordinates.update({"br":corners})
                    all_coordinates.update({"arena_br_c":(cx,cy)})

            arena_tl = all_coordinates["tl"]  
            arena_tr = all_coordinates["tr"]
            arena_bl = all_coordinates["bl"]  
            arena_br = all_coordinates["br"]

            tl = arena_tl[0]
            tr = arena_tr[1]
            br = arena_br[2]
            bl = arena_bl[3]

            pts1 = np.float32([tl, tr,
                            bl, br])
            pts2 = np.float32([[0, 0], [500, 0],
                            [0, 500], [500, 500]])
            
            matrix = cv2.getPerspectiveTransform(pts1, pts2)
            result = cv2.warpPerspective(image, matrix, (500, 500))


            return (matrix,result)
    
    


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

        warp_matrix, image_unwarp = self.get_perspective_warp_matrix(image)
       
        np.save('/home/e-yanthra/eyrc_hb/hb_task5_ws/warp_mtx.npy', warp_matrix)
            
        cv2.imshow('output undistorted' , image_unwarp)
        cv2.imshow('output distorted' , cv_image)
     
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    output = camera_node()

    for i in range(10):
        rclpy.spin_once(output)

    output.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
