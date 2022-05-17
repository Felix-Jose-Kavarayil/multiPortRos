#!/usr/bin/env python3
#

import rospy
from time import sleep
from std_msgs.msg import Int32
from std_msgs.msg import Header


fileBase=rospy.get_param("file_base","default_file_base")

fileName= fileBase+ ".log"



print("\033[44m" + ">>>>>>>> autopi_logger saving into " + fileName + " <<<<<<<<" + "\033[0m")

f = open(fileName, "w")

# file header
f.write("event time param\n")



def callbackTaskEvent(data):
    f.write("%s %10d.%09d NA\n" % (data.frame_id, data.stamp.secs, data.stamp.nsecs))
    f.flush()
def callbackMultiportControl(data):
    stamp=rospy.get_rostime()
    if data.data == 0:
        event = "light"
        param = "off"
    if data.data == 1:
        event = "light"
        param = "on"
    if data.data == 2:
        event = "reward"
        param = "NA"
    f.write("%s %10d.%09d %s\n" % (event ,stamp.secs, stamp.nsecs, param))
    f.flush()
def callbackMultiportIRReport(data):
    f.write("IRBeam %10d.%09d %s\n" % (data.stamp.secs,
                                       data.stamp.nsecs,
                                       data.frame_id))
    f.flush()


rospy.init_node('multiport_logger')
rospy.Subscriber("multi_port_ir_report", Header, callbackMultiportIRReport)
rospy.Subscriber("multi_port_control", Int32, callbackMultiportControl)
rospy.Subscriber("task_event", Header, callbackTaskEvent)
pubTaskEvent = rospy.Publisher('task_event',Header,queue_size=1)


rospy.spin()
f.close()

print("multiport_logger saved into %s" % fileName)
