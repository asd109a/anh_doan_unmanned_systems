#!/usr/bin/env python3
from re import S
import rclpy
import math 
import numpy as np

from rclpy.node import Node
from rclpy.duration  import Duration

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from unmanned_systems_ros2_pkg import PIDTemplate


def get_time_in_secs(some_node:Node) -> float:
    return some_node.get_clock().now().nanoseconds /1E9 
    
def euler_from_quaternion(x:float, y:float, z:float, w:float) -> tuple:
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

class TurtleBotNode(Node):
    def __init__(self, ns=''):
        super().__init__('minimial_turtlebot')
        
        if ns != '':
            self.ns = ns
        else:
            self.ns = ns
                
        #create vel and odom pub and subscribers
        self.vel_publisher = self.create_publisher(
            Twist, self.ns+ "/cmd_vel" ,  10) 
        
        self.odom_subscriber = self.create_subscription(
            Odometry, self.ns +"/odom", self.odom_callback, 10)
        
        self.current_position = [None,None]
        self.orientation_quat = [0,0,0,0] #x,y,z,w
        self.orientation_euler = [0,0,0] #roll, pitch, yaw

    def odom_callback(self,msg:Odometry) -> None:
        """subscribe to odometry"""
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        roll,pitch,yaw = euler_from_quaternion(qx, qy, qz, qw)
        
        self.orientation_euler[0] = roll
        self.orientation_euler[1] = pitch 
        self.orientation_euler[2] = yaw
        
        #Wrap yaw  from to 0 to 2pi
        # if self.orientation_euler[2] < 0:
        #     self.orientation_euler[2] += 2*np.pi
        # else:
        #     self.orientation_euler[2] = self.orientation_euler[2]

        # print("yaw is", np.degrees(self.orientation_euler[2]))
        
    def move_turtle(self, linear_vel:float, angular_vel:float) -> None:
        """Moves turtlebot"""
        twist = Twist()
        twist.linear.x = linear_vel 
        twist.angular.z = angular_vel
        self.vel_publisher.publish(twist)
    
def main()->None:
    rclpy.init(args=None)
    print("starting")

    namespace = ''
    rate_val = 5
    turtlebot_node = TurtleBotNode(namespace)
    rate = turtlebot_node.create_rate(rate_val)
    
    cmd_vel = 1.0 #m/s
    ang_vel = 0.5 #rad/s
    stop_vel = 0.0
    time_duration = 5
    
    # time intilization ref 
    time_origin = get_time_in_secs(turtlebot_node)
    print("time now is", time_origin)

    kp_angular = 1
    ki_angular = 0.0
    kd_angular  = 1
    dt_angular = 1/20

    pid_angular = PIDTemplate.PID(
        kp = kp_angular,
        ki = ki_angular,
        kd = kd_angular,
        dt = dt_angular)

    MAX_ANG_SPEED_RAD = 2.84 #rad/s

    # waypoint list 
    wp_list =[ 
        [1.00, 1.00],[2.00, 0.93],[3.00, 0.88],[3.90, 0.46],[4.90, 0.55],[5.89, 0.67],[6.87, 0.90],
        [6.58, 1.86],[6.38, 2.84],[7.00, 3.62],[6.26, 4.30],[6.86, 5.10],[6.33, 5.95],[5.33, 5.98],[4.33, 6.00],[3.33, 6.03],[2.33, 5.98],[1.33, 6.00],
        [0.52, 6.59],[0.76, 7.56],[0.98, 8.53],[0.94, 9.53],[0.97, 10.53],[0.81, 11.52],[0.11, 12.23],[0.81, 12.95],[1.11, 13.91],[2.09, 14.09],[3.06, 14.33],
        [3.69, 13.55],[3.12, 12.73],[3.89, 12.09],[3.89, 11.09],[4.00, 10.10],[3.28, 9.41],[4.02, 8.75],[4.50, 7.87],[3.69, 8.46],
        [3.41, 9.42],[3.75, 8.48],[4.66, 8.05],[5.65, 7.95],[6.29, 8.73],[7.06, 9.36],
        [7.62, 10.19],[8.40, 10.82],[9.40, 10.89],[10.15, 10.23],[10.70, 9.39],[10.34, 8.46],[10.87, 7.61],[11.60, 6.93],[12.59, 7.08],
        [13.58, 6.92],[14.10, 7.77],[13.83, 8.73],[14.04, 9.71],[13.45, 10.52],[13.33, 11.51],[14.25, 11.92],[14.45, 12.90],[14.15, 13.86],[13.79, 14.79],
        [12.79, 14.82],[11.79, 14.71],[10.85, 14.37],[9.88, 14.59],
        [8.89, 14.73],[7.90, 14.54],[8.10, 13.56],[7.14, 13.86],[7.00,13.00]
            ]


    heading_error_tol_rad = np.deg2rad(1)
    distance_error_tolerance_m = 0.15#m
    num_waypoints = len(wp_list)
    wp_counter = 0

    # try:
    try: 
        rclpy.spin_once(turtlebot_node)

        while rclpy.ok():
            if wp_counter >= num_waypoints:
                print ("xong roi")
                turtlebot_node.move_turtle (0.0,0.0)
                continue         
        
            for current_wp in wp_list:

                # get current waypoint
                # current_wp = wp_list[0]

                dx = current_wp[0] - turtlebot_node.current_position[0]
                dy = current_wp[1] -  turtlebot_node.current_position[1]
                desired_heading_rad = np.arctan2(dy,dx)

                current_heading_error_rad = pid_angular.compute_error(
                    desired_heading_rad,
                    turtlebot_node.orientation_euler[2]                
                )

                current_distance_error = np.sqrt(dx**2 + dy**2)
            
                ### SET CORRECT HEADING ------------
                while abs(current_heading_error_rad) >= heading_error_tol_rad:
                    
                    current_heading_error_rad = pid_angular.compute_error(
                        desired_heading_rad,
                        turtlebot_node.orientation_euler[2]                
                    )
                
                    if (abs(current_heading_error_rad) <= heading_error_tol_rad):
                        print("I'm done")
                        break

                    angular_gains = pid_angular.get_gains(
                        desired_heading_rad,
                        turtlebot_node.orientation_euler[2]
                    )

                    print("my heading error is", 
                        np.rad2deg(pid_angular.error[0]))

                    if angular_gains >= MAX_ANG_SPEED_RAD:
                        angular_gains = MAX_ANG_SPEED_RAD
                    elif angular_gains <= -MAX_ANG_SPEED_RAD:
                        angular_gains = -MAX_ANG_SPEED_RAD
                    
                    turtlebot_node.move_turtle(0.0, angular_gains)

                    rclpy.spin_once(turtlebot_node)

                ### ONCE HEADING IS CORRECT SEND FORWARD ---- 

                while current_distance_error >= distance_error_tolerance_m:
                    
                    current_heading_error_rad = pid_angular.compute_error(
                        desired_heading_rad,
                        turtlebot_node.orientation_euler[2]                
                    )
                
                    angular_gains = pid_angular.get_gains(
                        desired_heading_rad,
                        turtlebot_node.orientation_euler[2]
                    )

                    print("my heading error is", 
                        np.rad2deg(pid_angular.error[0]))

                    if angular_gains >= MAX_ANG_SPEED_RAD:
                        angular_gains = MAX_ANG_SPEED_RAD
                    elif angular_gains <= -MAX_ANG_SPEED_RAD:
                        angular_gains = -MAX_ANG_SPEED_RAD
                    
                    dx = current_wp[0] - turtlebot_node.current_position[0]
                    dy = current_wp[1] -  turtlebot_node.current_position[1]
                    current_distance_error = np.sqrt(dx**2 + dy**2)

                    if (current_distance_error <= distance_error_tolerance_m):
                        print("converged to wp")
                        turtlebot_node.move_turtle(0.0, 0.0)
                        break

                    turtlebot_node.move_turtle(0.15, angular_gains)

                    rclpy.spin_once(turtlebot_node)

                wp_counter = wp_counter + 1

    except KeyboardInterrupt:
        turtlebot_node.move_turtle(0.0, 0.0)

    

if __name__ == '__main__':
    """apply imported function"""
    main()