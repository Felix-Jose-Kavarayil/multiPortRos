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
from std_msgs.msg import Int32,Header,String
import sys
import time
import datetime
import ntpath
import argparse
import os
import random
import numpy as np
import json
import getpass
from multiport_common_funct import checkNodesTopicServices,enoughDiskSpace,createDirectory,copyDataFilesToDatabase
from collections import deque



lastRewardTime=time.time()
lightStatus=0 # flag to know if the light is on or off
arenaState=None # state of arena
nRewards=0 # number of reward in the current trial
nChoices=0 # number of choices in the current trial


nPorts=4
myCmd = {"allLightsOn": int('0000000111111111',2),
        "allLightsOff": int('0000000011111111',2),
        "allReward" :   int('0000001011111111',2)}
        
#rewardedport = 1 #np.randomranint(o,nports) #choose a random port to be rewarded


# dictionary to keep track of performance

n_trials_history=10
perfo = {"n_trials_history" : n_trials_history,
          "mean_reward" : np.nan, # for each trial it gets a 0,1,2 and this is the average over the last x trials
          "mean_choice" : np.nan,
          "percentage_correct" : np.nan, # percentage of correct choice
          "n_trials_done" : 0,
          "reward_history" : np.empty(n_trials_history),
          "choice_history" : np.empty(n_trials_history)}
perfo["reward_history"][:]=np.nan
perfo["choice_history"][:]=np.nan



def rewardedPorts_to_byteRepresentation(rewardedPorts):
    """
    Transform of list of rewarded port (0to7) to a bit representation
    Arguments
    rewardedPorts: list of rewarded ports
    
    Return bit string
    """
    mySum = 0
    for i in rewardedPorts:
        mySum= mySum + (1<<i)

    return "{0:08b}".format(mySum)

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
    
def start_trial():
    """
    Function to start a trial
    """
    global isProbeTrial
    global msg
    global lightStatus
    global nextLightChange
    global nRewards # number of reward in the current trial
    global nChoices # number of chioces in the current trial
    
    # reset to 0
    nRewards=0 
    nChoices=0
    
    
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
    
def read_config_file():
    """
    Function to read the config file with the rewarded ports and images on screens.
    """
    directory="/ext_drives/d69/data/electro/config_files"
    fn =  directory+"/"+mouseName+".config"
    
    if not os.path.exists(fn):
        raise IOError("File {} is missing".format(fn))
    
    with open(fn, 'r') as f:
        config = json.load(f)
    return config

def rotateAngle(angle,rotation):
    """
    Rotate an angle in degree by a certain rotation in degree
    
    The output range is from 0 to 359 and will be an integer
    """
    initRad = ((angle+rotation)/360*2*np.pi)
    x = np.cos(initRad)
    y = np.sin(initRad)
    newRad = np.arctan2(y,x)
    newAngle = np.round((newRad)/(2*np.pi)*360).astype(int)
    if newAngle < 0:
        newAngleCorrected = 360+newAngle
    else:
        newAngleCorrected = newAngle
    return newAngleCorrected
    
    
def rotateVisualCues(cueList,visualCueRotation):
    """
    Rotate the visual cues (assuming that the rotation are by multiple of 90 degrees only)
    """
    visualCueRotation = rotateAngle(0,visualCueRotation) # to restrict the range from 0 to 359
    myShift = int(visualCueRotation / 90)
    myList = deque(cueList)
    myList.rotate(myShift)
    myList = list(myList)
    return myList 



def display_images():
    """
    Display images listed in the config["window_files"] in the windows
    """
    #for i,wf in enumerate(config["window_files"]):
        #print("{} {}".format(wf,i+1))
       # pubMonitorControl.publish("{} {}".format(wf,i+1))
       # sleep(0.5)
    
    myString1 = ""
    for i in config["window_files"]:
        myString1= myString1+i+","
    myString1=myString1[:-1]
    myString1
    myString2 = ""
    for i,j in enumerate(config["window_files"]):
        myString2= myString2+"{}".format(i)+","
    myString2= myString2[:-1]
    aCmd = myString1+  " " + myString2
    print(aCmd)
    pubMonitorControl.publish(aCmd)
    

    
def start_dark_period():
    """
    start the dark period: switch off all lights and set the next light change to the current time + some random time & rotate the arena
    """
    rospy.loginfo("start dark period")
    global lightStatus
    global nextLightChange
    global arenaOrientation
    global portAngle
    
    pubCommand.publish(myCmd["allLightsOff"])
    lightStatus=0
    lightOffDurationSec_random = lightOffDurationSec+np.random.randint(low=-5, high=5, size=1)[0]
    nextLightChange = time.time() + lightOffDurationSec_random

    sleep(2) # add some time before the arena starts rotating (allow reward collection)
    
    
    
    possible_angles = [0, 90, -90]
    angle_to_rotate = np.random.choice(possible_angles)
    lightOffDurationSec_random_round = int(lightOffDurationSec_random)-4
    rospy.loginfo("light off seconds %s rounded %s:",lightOffDurationSec_random, lightOffDurationSec_random_round)
    
    
    pubArenaDuration.publish(lightOffDurationSec_random_round) # set the time it will take to rotate
    sleep(0.1)
    pubArenaControl.publish(angle_to_rotate) # starts rotating the arena
    sleep(0.1)
    
    ## 
    arenaOrientation = arenaOrientation+angle_to_rotate
    print("New arena orientation:",arenaOrientation)
    
   # Apply rotation to arenaOrientation
    

    print("New arena orientation:",arenaOrientation)

    for key in portAngle:
        portAngle[key] = rotateAngle(portAngle[key],angle_to_rotate)
    
    print("portAngle:",portAngle)
    
    
    
def update_performance(n_rewards,n_choices):
    """
    update the performance of the mouse at the end of a trial
    """
    
    global perfo
    
        
    
    if n_rewards < 0 or n_rewards > 2:
        raise ValueError("n_rewards should range from 0 to 2 but was {}".format(n_rewards))
    if n_choices < 0 or n_choices > 2:
        raise ValueError("n_choices should range from 0 to 2 but was {}".format(n_rewards))
    
    
    
    if perfo["n_trials_done"] < perfo["n_trials_history"]:
        perfo["reward_history"][perfo["n_trials_done"]]=n_rewards
        perfo["choice_history"][perfo["n_trials_done"]]=n_choices
    else:
        perfo["reward_history"] = np.roll(perfo["reward_history"],-1)
        perfo["reward_history"][perfo["n_trials_history"]-1] = n_rewards
        perfo["choice_history"] = np.roll(perfo["choice_history"],-1)
        perfo["choice_history"][perfo["n_trials_history"]-1] = n_choices
        
        
    perfo["n_trials_done"]+=1
    
    # Calculate mean reward and mean choice over the last n_trials_history trials
    perfo["mean_reward"] = np.nanmean(perfo["reward_history"])
    perfo["mean_choice"] = np.nanmean(perfo["choice_history"])
    
    
    indices = np.logical_and(perfo["choice_history"] != 0,~np.isnan(perfo["choice_history"]))
    if indices.sum() == 0:
        perfo["percentage_correct"] = 0
    else:
        perfo["percentage_correct"] = np.sum(perfo["reward_history"][indices]) / np.sum(perfo["choice_history"][indices])
    
   
    
    print("trial done:", perfo["n_trials_done"])
    print("reward history:",perfo["reward_history"])
    print("choice history:",perfo["choice_history"])
    print("mean reward/trial:", perfo["mean_reward"])
    print("mean choice/trial:", perfo["mean_choice"])
    print("percentage correct:", perfo["percentage_correct"])
    
#global_function = update_performance(n_rewards,n_choices)


def reset_performance():
    """
    Reset the performance variables to their initial values
    """
    global perfo
    
    n_trials_history=10
    perfo = {"n_trials_history" : n_trials_history,
          "mean_reward" : np.nan, # for each trial it gets a 0,1,2 and this is the average over the last x trials
          "mean_choice" : np.nan,
          "percentage_correct" : np.nan, # percentage of correct choice
          "n_trials_done" : 0,
          "reward_history" : np.empty(n_trials_history),
          "choice_history" : np.empty(n_trials_history)}
    perfo["reward_history"][:]=np.nan
    perfo["choice_history"][:]=np.nan
    """
    perfo = {
        "n_trials_done": 0,
        "n_trials_history": 10,  # Initial value for n_trials_history
        "reward_history": np.zeros(10),  # Initial value for reward_history
        "choice_history": np.zeros(10),  # Initial value for choice_history
        "mean_reward": 0.0,  # Initial value for mean_reward
        "mean_choice": 0.0,  # Initial value for mean_choice
        "percentage_correct": 0.0  # Initial value for percentage_correct
    }
    """
# Call the reset_performance function when needed to reset the performance
#reset_performance()



def end_trial():
    """
    Function run at the end of a trial
    """
    
    global config
    global rewardedAngles
    
    update_performance(nRewards,nChoices)
    start_dark_period()
    
    sleep(0.2)
    
    #to change the image location
    
    # Check if the number of trials done is greater than or equal to num_trials
    
   
    # Perform any necessary updates to perfo["n_trials_done"] and perfo["percentage_correct"]
    
    
     
    if perfo["n_trials_done"] >= perfo["n_trials_history"] and perfo["n_trials_done"] % 1 == 0 and perfo["percentage_correct"] > 0.66 and perfo["mean_reward"] > 1.3:
        
        
        print("image shift")
        visualCueRotation=90
        config["window_files"] = rotateVisualCues(config["window_files"],visualCueRotation)
        display_images()
        
        print("update rewardedAngles")
        rewardedAngles = [ rotateAngle(angle,visualCueRotation) for angle in rewardedAngles ]
        
        print("reset performance")
        reset_performance()
        
        
        

    

def callbackIRBeam(data):
    """
    Deliver water if the beam is broken while the light is on
    """
    global lastRewardTime
    global nextLightChange
    global lightStatus
    global trialRewardPortList # ports that have been already rewarded during this trial
    global nRewards
    global nChoices

    
    
    now = time.time()
    #print("{} {} {}".format(data.frame_id,lightStatus,isProbeTrial))
    message= data.frame_id.split()[0]
    messagePortNo = int(data.frame_id.split()[1])
    #print(message,messagePortNo)
    

    # if a port is not in use, don't do anything
    if messagePortNo >= nPorts:
        return
    
    
    # get the angle of the port, see if the angle is rewarded, if so reward the broken port
    
    
    
    if message == "broken" and lightStatus == 1 and lastRewardTime+refractoryDurationSec < now and isProbeTrial == False:
        
        
        
        if portAngle[messagePortNo] in rewardedAngles and messagePortNo not in trialRewardPortList: # is a port that we reward, but has not been depleated yet


            pubCommand.publish(rewardCommand(messagePortNo)) # give reward to the broken port
            trialRewardPortList.append(messagePortNo) # add to the list of rewarded port within this trial
            lastRewardTime=time.time()
            sleep(0.1)
             #
            pubCommand.publish(switchOffLightCommand(messagePortNo))
            
            
            nRewards= nRewards+1 
            nChoices= nChoices+1
    

            # if all ports have been depleated
            if len(trialRewardPortList) == len(config["rewarded_ports"]): #if len(trialRewardPortList) == len(config["rewarded_ports"]):
            	end_trial()


        if  portAngle[messagePortNo] not in rewardedAngles: # the animal poke a wrong port, end this trial there if messagePortNo not in config["rewarded_ports"]:
            nChoices= nChoices+1
            end_trial()
            
            

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
#parser.add_argument("-P", "--rewardedPorts",help="set the rewarded ports, using 0001, 0010, 0011, etc. One is rewarded, 0 is non-rewarded", action="store",default = "0011")

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

# read the configuration file
config = read_config_file()
print(config)


print("Rewarded ports:", config["rewarded_ports"])
arenaOrientation = 0
print("ArenaOrientation", arenaOrientation)
portAngle = {0:0,1:90,2:180,3:270}
print("portAngle:",portAngle)
rewardedAngles = [portAngle[port] for port in config["rewarded_ports"]]
print("rewardedAngles:",rewardedAngles)


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

#topic for monitor control

pubMonitorControl = rospy.Publisher('monitor_control', String, queue_size=1)






# topic for arena feedback
rospy.Subscriber("arena_info", Int32, callbackArenaInfo)
# topic for logging
pubTaskEvent = rospy.Publisher('task_event',Header,queue_size=2)
# topic for callback on IR break
rospy.Subscriber("multi_port_ir_report", Header, callbackIRBeam)
# topic for monitor control
#rospy.Subscriber("monitor_control", String, callbackMonitorControl)



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


#msg.frame_id="rewardedPort_{}".format(rewardedPorts_to_byteRepresentation(config["rewarded_ports"]))
msg.frame_id="rewardedPort_{}".format(rewardedPorts_to_byteRepresentation(config["rewarded_ports"]))
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
pubArenaMode.publish(0)
print("ready")

display_images()

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
        trialRewardPortList=[]
        
        if lightStatus == 0: # light will turn on, beginning of a trial
            rospy.loginfo("light ON")
            start_trial()

            
        else: # default end of the trial if not caused by a beam break
            rospy.loginfo("light OFF")
            sleep(0.1)
            end_trial()
    
    
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
