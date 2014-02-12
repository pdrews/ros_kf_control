#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('sim_tasks')
import rospy
from math import *
from task_manager_lib.TaskClient import *
from optparse import OptionParser

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.05)
tc = TaskClient(server_node,default_period)

parser = OptionParser()
parser.add_option("-s",dest="speed", default=0.1
                  help="Constant angular speed")

(options, args) = parser.parse_args()

tc.WaitForAuto()
try:
    tc.Constant(linear=0.0,angular=options.speed,duration=5.0)

except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()


rospy.loginfo("Mission completed")


