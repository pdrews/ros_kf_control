#!/usr/bin/env python
import roslib
roslib.load_manifest('cb_tracker')

import rospy
import math
import tf
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from axis_camera.msg import Axis
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from cb_tracker.cfg import CameraControlConfig

def sat(x,v):
    if x>v:
        return v
    if x<-v:
        return -v
    return x

class cbTracker:
    def __init__(self):
        rospy.init_node('cb_track')
        self.kp_x = rospy.get_param('~kp_x', 1)
        self.nx = rospy.get_param('~corners_x', 4)
        self.ny = rospy.get_param('~corners_y', 4)
        self.widthx = rospy.get_param('~spacing_x', 0.2)
        self.widthy = rospy.get_param('~spacing_y', 0.2)
        self.simmode = rospy.get_param('~sim_mode', True)
        self.sizesetpt = rospy.get_param('~size_setpt', 0.2)
        self.sizekp = rospy.get_param('~size_kp', 1.0)
        self.baseframe = rospy.get_param('~baseframe', "/world")
        self.cb_timeout = rospy.get_param('~timeout', 2.0)
        self.imuScale = rospy.get_param('~imu_scale', 0.0)
        self.pub = rospy.Publisher("~twistOut",Twist)
        self.pubsim = rospy.Publisher("~rateOut",Float64)
        rospy.Subscriber("~cbSize",Float32,self.store_cbsize)
        rospy.Subscriber("~IMU",Imu,self.store_imu)
        self.tfl = tf.TransformListener()
        self.cbsize = 0.2
        self.dyn_reconf_server = DynamicReconfigureServer(CameraControlConfig, self.reconfigure)
        self.imuz = 0.0

    def reconfigure(self, config, level):
        rospy.loginfo("Got reconfigure request")
        self.kp_x = config["kp_x"]
        self.sizesetpt = config["zoom_setpt"]
        self.sizekp = config["kp_zoom"]
        return(config)

    def store_cbsize(self, data):
        #cbSize is actually the distance from the edge of the board to the edge of the camera frame
        self.cbsize = data.data

        def store_imu(self, data):
        self.imuz = data.angular_velocity.z

    def run(self):
        rospy.loginfo("Waiting for checkerboard...")
        t = Twist()
        t.linear.x = -1
        rate = rospy.Rate(5)
        while (not rospy.is_shutdown()) and (not (self.tfl.frameExists(self.baseframe) and self.tfl.frameExists('/checkerboard'))):
            self.pub.publish(t)
            rate.sleep()
        while not rospy.is_shutdown():
            # Get the transform from the camera to the cb and servo the camera
            # Figure out when our last checkerboard frame happened
            lastTime = self.tfl.getLatestCommonTime('/checkerboard', self.baseframe)
            out = Twist()
            pt = PointStamped()
            pt.header.stamp = lastTime
            pt.header.frame_id = '/checkerboard'
            if (self.simmode):
                pt.point.y = -(self.nx / 2.0) * self.widthx
                pt.point.z = -(self.ny / 2.0) * self.widthy
            else:
                pt.point.x =  -(self.nx / 2.0) * self.widthx
                pt.point.y =  -(self.ny / 2.0) * self.widthy
            pt = self.tfl.transformPoint(self.baseframe, pt)
            rospy.loginfo("cbsize %f sizesetpt %f sizekp %f", self.cbsize, self.sizesetpt, self.sizekp)
            out.linear.x = (self.cbsize - self.sizesetpt) * self.sizekp 
            if (self.simmode):
                out.angular.z = self.kp_x * math.atan2(pt.point.x, pt.point.z)
                #rospy.loginfo("Rate: %f", out.angular.z)
            else:
                out.angular.z = (self.kp_x * math.atan2(pt.point.y, pt.point.x)) + (imu_scale * self.imuz)
                #rospy.loginfo("Rate: %f", out.angular.z)

            if (lastTime < (rospy.Time.now() - rospy.Duration(self.cb_timeout))):
                # If we loose the checkerboard, keep slewing but zoom out
                rospy.loginfo("Lost the checkerboard, now %f lastTime %f", rospy.Time.now().to_sec(), lastTime.to_sec())
                out.linear.x = -1.0
            rospy.loginfo("zoom = %f", out.linear.x)
            # For now, don't zoom anywhere
            out.linear.x = 0.0
            self.pub.publish(out)
            self.pubsim.publish(out.angular.z)
            rate.sleep()


if __name__=="__main__":
    demo = cbTracker()
    demo.run()
