#!/usr/bin/env python
# python code to run the multi-port task, with one port!
#
# make sure that the rosserial node communicating with the Arduino is running
# rosrun rosserial_python serial_node.py /dev/ttyACM0
#
#

import rospy
from time import sleep
from std_msgs.msg import Int32,Header
import sys
import time
import datetime
import ntpath
import argparse
import os
import random
import numpy as np
import getpass


lastRewardTime=time.time()
lightStatus=0 # flag to know if the light is on or off

nPorts=2
myCmd = {"allLightsOn": int('0000000111111111',2),
        "allLightsOff": int('0000000011111111',2),
        "allReward" :   int('0000001011111111',2)}

rewardedPort = np.random.randint(0,nPorts) # choose a random port to be rewarded


def rewardCommand(port):
    """
    Function to calculate the integer that will request reward delivery on a single port
    
    If the port is out of range it returns 0, which has no port selected and does nothing.
    
    Argument: 
    port: number of the port, index starts at port 0 up to 7.
    """
    if port < 0 or port > 7:
        print("port out of range")
        return int('0',2)
    
    command = 2 << 8 # command is 2, and we shift by 8 bits
    port = 1 << port # shift the one to the bit representing this port
    return command+port

def callbackIRBeam(data):
    """
    Deliver water if the beam is broken while the light is on
    """
    global lastRewardTime
    global nextLightChange
    global lightStatus
    
    now = time.time()
    #print("{} {} {}".format(data.frame_id,lightStatus,isProbeTrial))
    message= data.frame_id.split()[0]
    messagePortNo = int(data.frame_id.split()[1])
    #print(message,messagePortNo)

    # if a port is not in use, don't do anything
    if messagePortNo >= nPorts:
        return

    
    if message == "broken" and lightStatus == 1 and lastRewardTime+refractoryDurationSec < now and isProbeTrial == False:

        if  messagePortNo == rewardedPort:
            pubCommand.publish(rewardCommand(rewardedPort)) # give reward
            lastRewardTime=time.time()
            sleep(0.1)
        
        # switch off the light after a reward
        pubCommand.publish(myCmd["allLightsOff"])
        lightStatus=0
        nextLightChange = time.time()+lightOffDurationSec+np.random.randint(low=-5, high=5, size=1)[0]


        

defaultDrive="/ext_drives/d62/data/electro/" ## some drives have /data/electro and other /data/processing
defaultDatabase="/adata/electro/"
userName= getpass.getuser()
localData=os.getenv("HOME") + '/'



# parse arguments
parser = argparse.ArgumentParser()
parser.add_argument("mouse",help="set the mouse name")
parser.add_argument("-s", "--sessionDuration",help="set the session duration in seconds",type=int, action="store",default = "10")
parser.add_argument("-l", "--lightOnDuration",help="set the light on duration in seconds",type=int, action="store",default = "20")
parser.add_argument("-o", "--lightOffDuration",help="set the light off duration in seconds",type=int, action="store",default = "40")

parser.add_argument("-r", "--refractoryDuration",help="set the refractory period in seconds between two rewards",type=float, action="store",default = "2")
parser.add_argument("-p", "--probeTrialProportion",help="set the proportion of probe trials (from 0 to 1)",type=float, action="store",default = "0.0")
parser.add_argument("-d","--directory", help="create the direcotry for the data in "+defaultDatabase,action="store_true")
parser.add_argument("-t","--transfer", help="transfer the data files to the data directory in " + defaultDatabase,action="store_true")

## get the time
now = datetime.datetime.now()
# set some variables for this trials
args = parser.parse_args()
task=ntpath.basename(sys.argv[0])
mouseName=args.mouse
sessionDurationSec=args.sessionDuration
lightOnDurationSec = args.lightOnDuration
lightOffDurationSec = args.lightOffDuration
refractoryDurationSec = args.refractoryDuration
probeTrialProportion = args.probeTrialProportion
isProbeTrial = False



now = datetime.datetime.now()
sessionName=mouseName + now.strftime("-%d%m%Y-%H%M")
fileBase= localData + mouseName + now.strftime("-%d%m%Y-%H%M")
rospy.set_param("file_base", fileBase) # used by logger



print("sessionName:",sessionName,"Session duration:",sessionDurationSec,"Light on duration:",lightOnDurationSec,"Light off duration:", lightOffDurationSec ,"Refractory on reward:",refractoryDurationSec, "Probe trial proportion:",probeTrialProportion)
print("file:",fileBase)
print("rewarded port: {}".format(rewardedPort))

rospy.init_node('multiport_task')


pubCommand = rospy.Publisher('multi_port_control',Int32,queue_size=2)
pubTaskEvent = rospy.Publisher('task_event',Header,queue_size=1)
rospy.Subscriber("multi_port_ir_report", Header, callbackIRBeam)

sleep(5)
sleep(1) # wait until this node is up and running


msg=Header()
msg.frame_id="start"
msg.stamp=rospy.get_rostime()
pubTaskEvent.publish(msg)


timeout = time.time() + sessionDurationSec ## time at which to stop


# make sure we start with light off
pubCommand.publish(myCmd["allLightsOff"])
lightStatus=0
lastLightChange = time.time()
nextLightChange = lastLightChange+lightOffDurationSec + np.random.randint(low=-5, high=5, size=1)[0]


while True:

    # check if it is time to change the light
    if time.time() > nextLightChange:
        if lightStatus == 0: # light will turn on
            pubCommand.publish(myCmd["allLightsOn"])
            
            # decide if this is a light trial, get a random number from 0 to 1, compare to our proportion of probe trials
            myRand = np.random.uniform()
            #print("rand:",myRand, "probe prop.:",probeTrialProportion)
            if myRand < probeTrialProportion:
                isProbeTrial=True
                msg.frame_id="probe"
                msg.stamp=rospy.get_rostime()+ rospy.Duration(lightOnDurationSec/2)  # add time so it is in the middle of the light interval
                pubTaskEvent.publish(msg)
            else:
                isProbeTrial=False
            lightStatus=1
            nextLightChange = time.time()+lightOnDurationSec
        else: # light will turn off
            pubCommand.publish(myCmd["allLightsOff"])
            lightStatus=0
            nextLightChange = time.time()+lightOffDurationSec+np.random.randint(low=-5, high=5, size=1)[0]
    
    sleep(0.1)
    # stop the loop when session is done
    if time.time() > timeout :
        break

# switch off the light when we are done
if lightStatus==1:
    pubCommand.publish(myCmd["allLightsOff"])
    lightStatus=0

    
msg=Header()
msg.frame_id="end"
msg.stamp=rospy.get_rostime()
pubTaskEvent.publish(msg)

sleep(1)
rospy.signal_shutdown("task is over")
