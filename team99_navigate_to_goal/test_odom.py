import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point

# etc
import numpy as np
import math


class print_position_odom(Node):
    def __init__(self):
        super().__init__('print_position_odom')
        # State (for the update_Odometry code)
        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        #self.Init_ang = 0.0
        self.globalPos = Point()

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            1)
        self.odom_sub  # prevent unused variable warning

    def odom_callback(self, data):
        self.update_Odometry(data)

    def update_Odometry(self,Odom):
        position = Odom.pose.pose.position #Access linear position
        print(position)
    
def main(args=None):
    rclpy.init(args=args)
    print_odom = print_position_odom()
    rclpy.spin(print_odom)
    print_odom.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()