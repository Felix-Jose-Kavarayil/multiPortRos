#!/usr/bin/env python
# python code to run the multi-port task, with one port!
#
# make sure that the rosserial node communicating with the Arduino is running
# rosrun rosserial_python serial_node.py /dev/ttyACM0
#
#
import rostopic
import roslaunch
import rospkg
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
from multiport_common_funct import checkNodesTopicServices,enoughDiskSpace,createDirectory,copyDataFilesToDatabase

lastRewardTime=time.time()
lightStatus=0 # flag to know if the light is on or off
arenaState=None # state of arena

nPorts=4
myCmd = {"allLightsOn": int('0000000111111111',2),
        "allLightsOff": int('0000000011111111',2),
        "allReward" :   int('0000001011111111',2)}
        
#rewardedport = 1 #np.randomranint(o,nports) #choose a random port to be rewarded


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

def switchOffLightCommand(port):
    """
    Function to calculate the integer that will request turning off the light of one port
    
    If the port is out of range it returns 0, which has no port selected and does nothing.
    
    Argument: 
    port: number of the port, index starts at port 0 up to 7.
    """
    if port < 0 or port > 7:
        print("port out of range")
        return int('0',2)
    
    command = 0 << 8 # command is 0, and we shift by 8 bits
    port = 1 << port # shift the one to the bit representing this port
    return command+port
    

def start_dark_period():
    """
    start the dark period: switch off all lights and set the next light change to the current time + some random time & rotate the arena
    """
    rospy.loginfo("start dark period")
    global lightStatus
    global nextLightChange
    pubCommand.publish(myCmd["allLightsOff"])
    lightStatus=0
    lightOffDurationSec_random = lightOffDurationSec+np.random.randint(low=-5, high=5, size=1)[0]
    nextLightChange = time.time() + lightOffDurationSec_random

    sleep(2)
    possible_angles = [0,90,180]
    angle_to_rotate = np.random.choice(possible_angles)
    lightOffDurationSec_random_round = int(lightOffDurationSec_random)-4
    rospy.loginfo("light off seconds %s rounded %s:",lightOffDurationSec_random, lightOffDurationSec_random_round)
    pubArenaDuration.publish(lightOffDurationSec_random_round)
    sleep(0.1)
    pubArenaControl.publish(angle_to_rotate)
    sleep(0.1)
    
    

def callbackIRBeam(data):
    """
    Deliver water if the beam is broken while the light is on
    """
    global lastRewardTime
    global nextLightChange
    global lightStatus
    global rewardPortList # ports that have been already rewarded during this trial
    
    now = time.time()
    #print("{} {} {}".format(data.frame_id,lightStatus,isProbeTrial))
    message= data.frame_id.split()[0]
    messagePortNo = int(data.frame_id.split()[1])
    #print(message,messagePortNo)
    
    #start_dark_period = False

    # if a port is not in use, don't do anything
    if messagePortNo >= nPorts:
        return
       
    if message == "broken" and lightStatus == 1 and lastRewardTime+refractoryDurationSec < now and isProbeTrial == False:

        if  messagePortNo in rewardedPorts and messagePortNo not in rewardPortList: # is a port that we reward, but has not been depleated yet

            pubCommand.publish(rewardCommand(messagePortNo)) # give reward to the broken port
            rewardPortList.append(messagePortNo) # add to the list of rewarded port within this trial
            lastRewardTime=time.time()
            sleep(0.1)
             #
            pubCommand.publish(switchOffLightCommand(messagePortNo))

            # if all ports have been depleated
            if len(rewardPortList) == len(rewardedPorts):
            	# start_dark_period = True
            	start_dark_period()


        if messagePortNo not in rewardedPorts: # the animal poke a wrong port, end this trial there
            # start_dark_period = True
            start_dark_period()
            
    #if start_dark_period:
    #	# start the dark period either because all ports have been depleated or the animal poke a wrong port
    #	pubCommand.publish(myCmd["allLightsOff"])
    #	lightStatus=0
    #	lightOffDurationSec_random = lightOffDurationSec+np.random.randint(low=-5, high=5, size=1)[0]
    #	nextLightChange = time.time() + lightOffDurationSec_random

            

def callbackArenaInfo(data):
	"""
	callback function to update the current arena state as it is sent by the arena
	"""
	#print("debug: arena,",data.data)
	global arenaState
	arenaState = data.data

defaultDrive="/ext_drives/d69/data/electro/" ## some drives have /data/electro and other /data/processing
defaultDatabase="/adata/electro/"
userName= getpass.getuser()
localData=os.getenv("HOME") + '/'

rospack = rospkg.RosPack()# get an instance of RosPack with the default search paths
autopi_rosPackagePath=rospack.get_path('multiport_ros')
print("multiPortRos path: {}".format(autopi_rosPackagePath))


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
parser.add_argument("-P", "--rewardedPorts",help="set the rewarded ports, using 0001, 0010, 0011, etc. One is rewarded, 0 is non-rewarded", action="store",default = "0011")

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

if len(args.rewardedPorts) != nPorts:
    raise ValueError("rewardedPorts should have a length of nPorts {}".format(nPorts))

# get the rewarded ports
rewardedPorts= []
for i,v in enumerate((args.rewardedPorts[::-1])): # reverse order to read from right to left (as bit operations)
    if v == "1":
        rewardedPorts.append(i)
print("Rewarded ports:", rewardedPorts)

isProbeTrial = False



now = datetime.datetime.now()
sessionName=mouseName + now.strftime("-%d%m%Y-%H%M")
fileBase= localData + mouseName + now.strftime("-%d%m%Y-%H%M")
rospy.set_param("file_base", fileBase) # used by logger
mouseDir= defaultDatabase+mouseName

# set ros parameters to set the file name for the video created by the jetson_camera_node.py
rospy.set_param("cv_camera_arena_top/output_path",fileBase+".arena_top.avi") # used to record the video


print("sessionName:",sessionName,"Session duration:",sessionDurationSec,"Light on duration:",lightOnDurationSec,"Light off duration:", lightOffDurationSec ,"Refractory on reward:",refractoryDurationSec, "Probe trial proportion:",probeTrialProportion)
print("file:",fileBase)





try:
    # Checkif rosmaster is running or not.
    rostopic.get_topic_class('/rosout')
except rostopic.ROSTopicIOException as e:
    print("Could not find ros master")
    print("Is roscore running?")
    sys.exit()


###########################
## run the launch file  ###
###########################
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
taskNameNoPy=task.split(".",1)[0]

# the name of the launch file is "task_master_slave.launch"
launchFileName=autopi_rosPackagePath+"/src/launch_files/"+ "{}.launch".format(taskNameNoPy)
print("task name:" + taskNameNoPy)
print("Launch file name: " + launchFileName)
launch = roslaunch.parent.ROSLaunchParent(uuid,[launchFileName])
launch.start()
rospy.loginfo("launch file started")
sleep(5) # wait so that all nodes are up and running


rospy.init_node('multiport_task')

# topics related to multiport ports
pubCommand = rospy.Publisher('multi_port_control',Int32,queue_size=2)
# topic for arena related
pubArenaMode = rospy.Publisher('arena_mode', Int32, queue_size=1)
pubArenaDuration = rospy.Publisher('arena_duration', Int32, queue_size=1)
pubArenaControl = rospy.Publisher('arena_control', Int32, queue_size=1)
# topic for arena feedback
rospy.Subscriber("arena_info", Int32, callbackArenaInfo)
# topic for logging
pubTaskEvent = rospy.Publisher('task_event',Header,queue_size=2)
# topic for callback on IR break
rospy.Subscriber("multi_port_ir_report", Header, callbackIRBeam)


sleep(1) # wait until this node is up and running



## check that the nodes, topics and services needed are running
nodeList=["/node_beams","/node_arena","/multiport_task_logger","/cv_camera_arena_top"]
topicList=["/multi_port_control","/multi_port_ir_report","/task_event","/cv_camera_arena_top/image_raw","/arena_control", "/arena_duration", "/arena_mode"]
serviceList=[]

if not checkNodesTopicServices(nodeList,topicList,serviceList):
  sys.exit()
print("All nodes and topics are there")


if not enoughDiskSpace(directory = localData) :
    print("Make space on the hard drive before trying to run this program")
    sys.exit()


if args.directory:
    if not createDirectory(mouseName=mouseName,mouseDir=mouseDir,sessionName=sessionName,defaultDrive=defaultDrive) :
        print("Unable to create the session directory. Please fix this before continuing.")
        sys.exit()

master="a230-pc89"
arenaCameraHost="a230-pc005"
        
msg=Header()
msg.frame_id="start"
msg.stamp=rospy.get_rostime()
pubTaskEvent.publish(msg)


rpInt = int(args.rewardedPorts,2)
msg.frame_id="rewardedPort_{0:08b}".format(rpInt)
msg.stamp=rospy.get_rostime()
pubTaskEvent.publish(msg)

# zero the arena
pubArenaMode.publish(0)
sleep(0.1)
pubArenaControl.publish(-1)
print("zeroing...")
sleep(1)

while True:
	# wait for arena to be zeroed
	if arenaState==-1:
		print("zeroing done")
		break
	sleep(0.1)

sleep(2.0)
pubArenaMode.publish(1)
print("ready")


timeout = time.time() + sessionDurationSec ## time at which to stop


# make sure we start with light off
pubCommand.publish(myCmd["allLightsOff"])
lightStatus=0
lastLightChange = time.time()
print("Wait 5 seconds until light on")
nextLightChange = lastLightChange+5   #lightOffDurationSec + np.random.randint(low=-5, high=5, size=1)[0]


while True:

    # check if it is time to change the light
    if time.time() > nextLightChange:

        rospy.loginfo("light change by time")
        rewardPortList=[]
        
        if lightStatus == 0: # light will turn on
            rospy.loginfo("light ON")
            sleep(0.1)
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
            rospy.loginfo("light OFF")
            sleep(0.1)
            start_dark_period()
    
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



launch.shutdown()
sleep(10)


if args.transfer:
    copyDataFilesToDatabase(fileBase,mouseDir,sessionName,
                            master, arenaCameraHost,
                            userName,
                            arenaCamera=True)

sleep(3)
rospy.signal_shutdown("task is over")