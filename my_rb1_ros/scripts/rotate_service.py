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

    def callback(self, request):
        rospy.loginfo(f"Service Requested - Rotating {request.degrees}°")
        rotate_response = RotateResponse()
        initial_yaw = self.yaw
        target_angle = math.radians(request.degrees)

        if self.yaw is None:
            rospy.logerr("Yaw data is not available...")
            rotate_response.result = "Failed - No yaw data..."
            return rotate_response
        try:
            accumulated_angle = 0.0 
            tolerance = math.radians(1)
            cmd = Twist()
            angular_speed = -1 if target_angle >= 0 else 1
            previous_yaw = initial_yaw

            while abs(accumulated_angle) < abs(target_angle) - tolerance and not rospy.is_shutdown():
                cmd.angular.z = angular_speed
                self.pub.publish(cmd)
                self.r.sleep()

                current_yaw = self.yaw
                delta_yaw = current_yaw - previous_yaw

                if delta_yaw > math.pi:
                    delta_yaw -= 2 * math.pi
                elif delta_yaw < -math.pi:
                    delta_yaw += 2 * math.pi

                accumulated_angle += delta_yaw
                previous_yaw = current_yaw

            cmd.angular.z = 0
            self.pub.publish(cmd)     
        
            rospy.loginfo(f"Service Completed - Rotated {request.degrees}°")
            rotate_response.result = "RB1 Completed Rotation Successfully"
            return rotate_response
        except:
            rospy.logerr("Unexpected error has occurred...")
            rotate_response.result = "RB Failed to Complete Rotation"
            return rotate_response


    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation 
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.yaw = yaw

if __name__ == "__main__":
    RotateService()
    rospy.spin()