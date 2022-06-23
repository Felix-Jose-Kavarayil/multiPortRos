#!/usr/bin/env python3
#

import rospy
from time import sleep
from std_msgs.msg import Int32
from std_msgs.msg import Header


fileBase=rospy.get_param("file_base","default_file_base")

fileName= fileBase+ ".log"


def decodeCommand(cmd):
    """
    Decode the command number into a command and port selection
    """
    
    # deal with the command
    commandPart = cmd >> 8 # shift to the right by 8 bits
    #print("Command part:", commandPart, "{:b}".format(commandPart))
    
    
    # deal with the port selection
    portPart = cmd & 255 # AND binary operation
    #print("Port part:", portPart, "{:b}".format(portPart))
    port = portPart
    i = 0
    firstSelectedPort = -1
    while port > 0: 
        selected = port & 1 # get the least significant bit (the last one to the right)
        if selected :
           # print("port {} selected".format(i))
            if firstSelectedPort == -1:
                firstSelectedPort = i
            #    print("first selected port {}".format(i))
        port = port >> 1 # shift to the right to check the next port, get rid of the one just checked
        i+=1
    return commandPart, portPart, firstSelectedPort
    


print("\033[44m" + ">>>>>>>> autopi_logger saving into " + fileName + " <<<<<<<<" + "\033[0m")

f = open(fileName, "w")

# file header
f.write("event time param\n")



def callbackTaskEvent(data):
    f.write("%s %10d.%09d NA\n" % (data.frame_id, data.stamp.secs, data.stamp.nsecs))
    f.flush()
def callbackMultiportControl(data):

    command, selection, firstSelectedPort = decodeCommand(data.data)
    
    stamp=rospy.get_rostime()
    if command == 0:
        event = "light"
        param = "off"
    if command == 1:
        event = "light"
        param = "on"
    if command == 2:
        event = "reward"
        param = firstSelectedPort
    f.write("%s %10d.%09d %s\n" % (event ,stamp.secs, stamp.nsecs, param))
    f.flush()
def callbackMultiportIRReport(data):

    message= data.frame_id.split()[0]
    messagePortNo = int(data.frame_id.split()[1])
    print(message)
    if message == "broken":
        f.write("IRBeamBroken %10d.%09d %s\n" % (data.stamp.secs,
                                                 data.stamp.nsecs,
                                                messagePortNo))
    if message == "notBroken":
        f.write("IRBeamNotBroken %10d.%09d %s\n" % (data.stamp.secs,
                                                    data.stamp.nsecs,
                                                    messagePortNo))
    f.flush()


rospy.init_node('multiport_logger')
rospy.Subscriber("multi_port_ir_report", Header, callbackMultiportIRReport)
rospy.Subscriber("multi_port_control", Int32, callbackMultiportControl)
rospy.Subscriber("task_event", Header, callbackTaskEvent)
pubTaskEvent = rospy.Publisher('task_event',Header,queue_size=1)


rospy.spin()
f.close()

print("multiport_logger saved into %s" % fileName)
