#! /usr/bin/env python

import rospy 
import math
from my_rb1_ros.srv import Rotate, RotateResponse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class RotateService(object):
    def __init__(self):
        rospy.init_node("rotate_service_node")
        self.service = rospy.Service("/rotate_robot", Rotate, self.callback)    
        self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)   
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.r = rospy.Rate(20)
        self.yaw = None
        rospy.loginfo("Service Ready.")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        
        while angle < -math.pi:
            angle += 2 * math.pi
        
        return angle

    def callback(self, request):
        rospy.loginfo("Service Requested.")
        rotate_response = RotateResponse()
        initial_yaw = self.yaw
        target_angle = math.radians(request.degrees)

        if self.yaw is None:
            rospy.logerr("Yaw data is not available...")
            rotate_response.result = "Failed - No yaw data..."
            return rotate_response
        try:
            current_angle = 0.0 
            tolerance = math.radians(1)
            cmd = Twist()
            angular_speed = 0.3 if target_angle >= 0 else -0.3

            while abs(current_angle) < abs(target_angle) - tolerance and not rospy.is_shutdown():
                cmd.angular.z = angular_speed
                self.pub.publish(cmd)
                self.r.sleep()
                delta_yaw = self.normalize_angle(self.yaw - initial_yaw)
                current_angle = delta_yaw

            cmd.angular.z = 0
            self.pub.publish(cmd)     
        
            rospy.loginfo("Service Completed.")
            rotate_response.result = "success"
            return rotate_response
        except:
            rospy.logerr("Unexpected error has occurred...")
            rotate_response.result = "Failed..."
            return rotate_response


    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation 
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.yaw = yaw

if __name__ == "__main__":
    RotateService()
    rospy.spin()