#!/usr/bin/env python
import rospy
import rospkg
from volta_msgs.msg import RPM
from nav_msgs.msg import Odometry
import math
import tf
from geometry_msgs.msg import TransformStamped
import yaml

class Odompublisher():
    def __init__(self):
        self.rpm_left = None
        self.rpm_right = None
        self.linear_velocity = None
        self.angular_velocity = None
        self.heading_old = 0
        self.heading_ = 0
        self.x_=0
        self.y_=0
        self.timestamp = rospy.get_time()
        print("self.timestamp:",self.timestamp)
        self.rospack = rospkg.RosPack()
        self.get_params()
        self.rpm_subscriber = rospy.Subscriber('rpm_sub',RPM,self.rpm_callback)
        self.odom_pub = rospy.Publisher('wheel_odom',Odometry,queue_size=10)
        self.br = tf.TransformBroadcaster()
        
    
    def get_params(self):
        with open(self.rospack.get_path('multivolta_exploration') + "/config/control2.yaml",'r') as stream:
            # data = yaml.load(stream, Loader=yaml.FullLoader)
            data = yaml.load(stream)

            self.wheel_radius = data['volta_base_controller']['wheel_radius'] 
            # * \
            # data['volta_base_controller']['left_wheel_radius_multiplier']
            self.wheel_seperation = data['volta_base_controller']['wheel_separation'] 
            # * \
            # data['volta_base_controller']['wheel_separation_multiplier']
            self.odomframe = data['volta_base_controller']['odom_frame_id']
            self.childframe = data['volta_base_controller']['base_frame_id']
            print("self.wheel_radius:",self.wheel_radius)
            print("self.wheel_seperation:",self.wheel_seperation)
            print("self.odomframe:",self.odomframe)
            print("self.childframe:",self.childframe)
    
    def rpm_callback(self,rpm):
        # self.rpm_left = rpm.left / 24.0
        # self.rpm_right = rpm.right / 24.0
        #This was calculated experimentally and needs to be improved
        self.rpm_left = rpm.left / 24.0
        self.rpm_right = rpm.right / 24.0
        self.computespeed()
        self.publish_odom()
    
    def computespeed(self):
        avg_rps = (self.rpm_left + self.rpm_right)/(2 * 60)
        self.linear_velocity = avg_rps * (2 * self.wheel_radius * math.pi)
        avg_rps_angular = (self.rpm_right - self.rpm_left) / (60)
        self.angular_velocity = (avg_rps_angular * (2 * self.wheel_radius * math.pi))/(self.wheel_seperation)
        # print("self.linear_velocity:",self.linear_velocity)
        # print("self.angular_velocity:",self.angular_velocity)
        self.updateOpenLoop(self.linear_velocity,self.angular_velocity,rospy.get_time())
#-----------------------------Utility functions for Odometry calculation-----------------------------
    def updateOpenLoop(self,linear,angular,curr_time):
        # print("self.wheel_seperation:",self.wheel_seperation)
        # print("self.wheel_radius:",self.wheel_radius)
        # curr_time_new = curr_time.sec + (curr_time.nanosec * 0.000000001)
        # self.timestamp_new = self.timestamp.sec + (self.timestamp.nanosec * 0.000000001)
        dt = curr_time - self.timestamp
        # print("dt:",dt)
        self.timestamp = curr_time
        self.integrateExact(linear * dt, angular * dt)
        #Correct pose: Clockwise is from 0 to -pi, counter clockwise is from 0 to +pi
        # print("self.heading_",self.heading_)
        if(self.heading_ < -3.1415):
            self.heading_ = 3.141 + (self.heading_ % (-3.141))
        
        elif(self.heading_ > 3.1415):
            self.heading_ = -3.141 + (self.heading_ % (3.141))
        
    
    def integrateExact(self,linear,angular):
        if (abs(angular) < 1e-6):
            self.integrateRungeKutta2(linear, angular)
        else:
            self.heading_old = self.heading_
            r = linear / angular
            self.heading_ += angular
            self.x_ += r * (math.sin(self.heading_) - math.sin(self.heading_old))
            self.y_ += -r * (math.cos(self.heading_) - math.cos(self.heading_old))
            # print("xnew,ynew,he ad",self.x_,self.y_)
    

    def integrateRungeKutta2(self,linear,angular):
        direction = self.heading_ + angular * 0.5
        self.x_ += linear * math.cos(direction)
        self.y_ += linear * math.sin(direction)
        self.heading_ += angular
        # print("x,y,head",self.x_,self.y_)
#--------------------------------------------------------------------------------------------------#
    def publish_odom(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        #Adding tf prefix to the Odometry publisher
        odom.header.frame_id = str(self.odomframe)
        odom.child_frame_id = str(self.childframe)
        odom.pose.pose.position.x = self.x_
        odom.pose.pose.position.y = self.y_
        #Convert orientation into quaternion
        # print("prev_x,prev_y,prev_heading:",self.x_,self.y_,self.heading_)
        # print("x,y,heading:",self.x_,self.y_,self.heading_)
        # q = tf_transformations.quaternion_from_euler(0.0,0.0,self.heading_)
        q = tf.transformations.quaternion_from_euler(0.0,0.0,self.heading_)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity
        self.odom_pub.publish(odom)
        #Publish transform from odom to base_link
        #NOTE: The transform needs to be published only after combining with IMU data
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = str(self.odomframe)
        t.child_frame_id = str(self.childframe)
        t.transform.translation.x = self.x_
        t.transform.translation.y = self.y_
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # #Send the transform
        self.br.sendTransformMessage(t)
        





if __name__ == '__main__':
    rospy.init_node('odompublisher')
    Odompublisher_obj = Odompublisher()
    rospy.spin()
    # Odompublisher_obj.get_params()

        
