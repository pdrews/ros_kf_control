#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('sim_tasks')
import rospy
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.05)
tc = TaskClient(server_node,default_period)
rospy.loginfo("Mission connected to server: " + server_node)

tc.SetPTZ(pan=0.0,tilt=0.0)
tc.AlignWithShore(angle=0.0, ang_velocity=0.5)
tc.Constant(linear=1.0, angular=0.0 duration=2.0)

rospy.loginfo("Mission completed")

