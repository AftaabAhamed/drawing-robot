import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3
import numpy as np

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

        return float(pwm1+1) ,float(pwm2+1), float(pwm3+1)




class Publisher(Node):
    def __init__(self):
        super().__init__('Publisher_node')
       # Initialze Publisher with the "/Integer" topic
        self.publisher1_ = self.create_publisher(Vector3, '/cmd_vel/bot1', 10)
        self.publisher2_ = self.create_publisher(Vector3, '/cmd_vel/bot2', 10)
        self.publisher3_ = self.create_publisher(Vector3, '/cmd_vel/bot3', 10)

        self.timer = self.create_timer(0.5,self.timer_callback)

        self.x = 90
        self.y = 90
        self.z = 90 

        bot2_path_list = ['/home/e-yanthra/eyrc_hb/hb_task4_ws/src/hb_task4b/hb_task4b/motor_cal/21.txt',
                          '/home/e-yanthra/eyrc_hb/hb_task4_ws/src/hb_task4b/hb_task4b/motor_cal/22.txt',
                          '/home/e-yanthra/eyrc_hb/hb_task4_ws/src/hb_task4b/hb_task4b/motor_cal/23.txt']
        
        bot1_path_list = ['/home/e-yanthra/eyrc_hb/hb_task4_ws/src/hb_task4b/hb_task4b/motor_cal/11.txt',
                            '/home/e-yanthra/eyrc_hb/hb_task4_ws/src/hb_task4b/hb_task4b/motor_cal/12.txt',
                            '/home/e-yanthra/eyrc_hb/hb_task4_ws/src/hb_task4b/hb_task4b/motor_cal/13.txt']

        bot3_path_list = ['/home/e-yanthra/eyrc_hb/hb_task4_ws/src/hb_task4b/hb_task4b/motor_cal/31.txt',
                            '/home/e-yanthra/eyrc_hb/hb_task4_ws/src/hb_task4b/hb_task4b/motor_cal/32.txt',
                            '/home/e-yanthra/eyrc_hb/hb_task4_ws/src/hb_task4b/hb_task4b/motor_cal/33.txt']
        
        self.pwm1 = pwm_provider(bot1_path_list)
        self.pwm2 = pwm_provider(bot2_path_list)
        self.pwm3 = pwm_provider(bot3_path_list)        

    def inverse_kinematics(self,vx,vy,w):


        transform_matrix=np.array([ [-0.33, 0.58, 0.33],
                                    [-0.33, -0.58, 0.33],
                                    [ 0.67, 0, 0.33]])

        vel_chassis_array=np.array([[vx],
                                    [vy],
                                    [w]])
        wheel_vel=np.transpose(np.dot(transform_matrix, vel_chassis_array))
        wheel_vel.flatten()

        return (wheel_vel[0][0]) , ((wheel_vel[0][1])), (wheel_vel[0][2])


    def motor(self , x, y,z):

        msg1 = Vector3()
        msg2 = Vector3()
        msg3 = Vector3()
        msg1.z ,msg1.y ,msg1.x  = self.pwm1.get_pwm(self.inverse_kinematics(x,y,z))
        msg2.z ,msg2.y ,msg2.x  = self.pwm2.get_pwm(self.inverse_kinematics(x,y,z))
        msg3.z ,msg3.y ,msg3.x  = self.pwm3.get_pwm(self.inverse_kinematics(x,y,z))

 
        self.publisher1_.publish(msg1)
        self.publisher2_.publish(msg2)
        self.publisher3_.publish(msg3)

    def timer_callback(self):
        dir = input("enter xyz: ")
        x , y ,z = dir.strip().split()
        self.motor(float(x),float(y),float(z)) 
         


        

def main(args = None):
    rclpy.init(args=args)
    Publisher_node = Publisher()
    rclpy.spin(Publisher_node)
    Publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

