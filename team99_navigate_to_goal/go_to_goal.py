import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from time import sleep
# etc
import numpy as np
import math
#Waypoints
#1.5 0
#1.5 1.4
#0 1.4

class odom_go_to_goal(Node):
    def __init__(self):
        super().__init__('odom_go_to_goal')
        # State (for the update_Odometry code)
        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalPos = Point()
        self.waypoint_flag = 0
        self.vel = Twist()
        self.vel.linear.x = 0.1
        self.vel.angular.z = 0.0

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            1)
        self.odom_sub  # prevent unused variable warning

        self.move_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move_pub

    def odom_callback(self, data):
        self.c_PosX, self.c_PosY, self.c_Ang = self.update_Odometry(data)
        if (float(self.c_PosX) < 1.5 and self.waypoint_flag == 0):
            #Implement a P controller here
            self.move_pub.publish(self.vel)
        elif(self.waypoint_flag == 0):
            self.vel.linear.x = 0.0
            #Rotation
            self.vel.angular.z = 1.0
            if (float(self.c_Ang) < 1.5):
                #Implement a P controller here
                self.move_pub.publish(self.vel)
                print("I enter the if statement")
            else:
                sleep(10) #Pause in waypoint 1
                print("I enter the else statement")
                self.vel.angular.z = 0.0
                self.move_pub.publish(self.vel)
                self.waypoint_flag = 1
        if (float(self.c_PosY) < 1.4 and self.waypoint_flag == 1):
            #Implement a p controller here
            self.vel.linear.x = 0.1
            self.move_pub.publish(self.vel)
        elif(self.waypoint_flag == 1):
            self.vel.linear.x = 0.0
            #Rotation
            self.vel.angular.z = 1.0
            if (float(self.c_Ang) < 3.0):
                #Implement a P controller here
                self.move_pub.publish(self.vel)
                print("I enter the if statement")
            else:
                sleep(10) #Pause in waypoint 2
                print("I enter the else statement")
                self.vel.angular.z = 0.0
                self.move_pub.publish(self.vel)
                self.waypoint_flag = 2
                self.vel.linear.x = 0.1 
        if(float(self.c_PosX ) < 2.0 and float(self.c_PosX) > 0 and self.waypoint_flag == 2):
            self.vel.linear.x = 0.1
            self.move_pub.publish(self.vel)
        elif(self.waypoint_flag == 2):
            self.vel.linear.x = 0.0
            self.move_pub.publish(self.vel)
            print("Goal Reached!")


    def update_Odometry(self,Odom):
        position = Odom.pose.pose.position #Access linear position
        
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation #Access angular position
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        #We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang
        self.get_logger().info('Transformed global pose is x:{}, y:{}, a:{}'.format(self.globalPos.x,self.globalPos.y,self.globalAng))
        return((self.globalPos.x, self.globalPos.y, self.globalAng))
    
def main(args=None):
    rclpy.init(args=args)
    print_odom = odom_go_to_goal()
    rclpy.spin(print_odom)
    print_odom.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()