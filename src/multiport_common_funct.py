import rosnode
import rospy
import sys
import os
from shutil import copyfile
import netifaces
import subprocess
import numpy as np
import pandas as pd
import getpass
import socket
import rosservice
import psutil
from os import path
import random



## function to check if all the needed nodes are running
def checkNodesTopicServices(neededNodes, neededTopics, neededServices):
    return(all([checkIfNodesRunning(neededNodes),checkPublishedTopics(neededTopics),checkIfServicesRunning(neededServices)]))

def checkIfNodesRunning(neededNodes):
    if(len(neededNodes)==0):
        return(True)
    
    runningNodes=rosnode.get_node_names()
    result =  all(elem in runningNodes  for elem in neededNodes)
    if not result:
        print("Running nodes:")
        print(runningNodes)
        print("Needed nodes:")
        print(neededNodes)
        for elem in neededNodes:
            if elem not in runningNodes:
                print("{} not running".format(elem))     
    return(result)

## function to check if all the needed services are running
def checkIfServicesRunning(neededServices):
    if(len(neededServices)==0):
        return(True)
    runningServices=rosservice.get_service_list()
    result =  all(elem in runningServices  for elem in neededServices)  
    if not result:
        print("Running services:")
        print(runningServices)
        print("Needed services:")
        print(neededServices)
        for elem in neededServices:
            if elem not in runningServices:
                print("{} not running".format(elem))     
    return(result)

## function to check if the needed topics have been published
def checkPublishedTopics(neededTopics):
    if(len(neededTopics)==0):
        return(True)

    publishedTopics=rospy.get_published_topics() 

    # we have a list of list, get only the first item of each sublist
    flat_list = []
    for sublist in publishedTopics:
        flat_list.append(sublist[0])
    publishedTopics=flat_list
    
    result =  all(elem in publishedTopics  for elem in neededTopics)
    if not result:
        print("Published topics:")
        print(publishedTopics)
        print("Needed topics:")
        print(neededTopics)
        for elem in neededTopics:
            if elem not in publishedTopics:
                print("{} not running".format(elem))     
    return(result)

def enoughDiskSpace(directory="/home/kevin",minSpace=4000000000):
    hdd = psutil.disk_usage(directory)
    if hdd.free < minSpace:
        print("Not enough disc space associated with {}".format(directory))
        print("Minimum is {} Mb".format(minSpace/1000000))
        return False
    else:
        return True

def createDirectory(mouseName,mouseDir,sessionName,defaultDrive):

    print("Create data directories in the database")
    #check if the mouse directory is there, if not create
    if not defaultDrive.startswith("/") or not defaultDrive.endswith("/"):
        print(defaultDrive + " should start and end with /")
        return False

    if os.path.islink(mouseDir) and not os.path.exists(mouseDir):
        print(mouseDir + " is most likely a broken symbolic link")
        print("Try to fix this first. Maybe try rm " + mouseDir)
        return False
    
    if os.path.exists(mouseDir) and os.path.isdir(mouseDir):
        print(mouseDir + " exists")
               
    else:
        print(mouseDir + " mouse directory does not exist, creating it in " + defaultDrive)
        path=defaultDrive+mouseName
        print("Try to create "+ path)


        try:
            if os.path.isdir(defaultDrive):
                print(defaultDrive + " exists")
            else:
                print(defaultDrive + " does not exist")
                print("please give a valid defaultDrive as argument to createDirectory")
                return False
        except OSError:
            print ("Error while checking the defaultDrive path %s" % path)
            return False
        else:
            print("Valid defaultDrive path")
        
        try:
            if os.path.isdir(path):
                print(path + " exists")
            else:
                print(path + " does not exist, creating it")
                os.mkdir(path)
        except OSError:
            print ("Error while trying to create the mouse directory %s" % path)
            return False
        else:
            print ("%s mouse directory is now available" % path)

        # create a symbolic link in
        print ("Trying to create the symlink from "+path+" to "+mouseDir)
        try:
            os.symlink(path,mouseDir)
        except OSError:
            print("Creation of the symlink failed")
            return False
        else:
            print("Successfully created the symlink to %s" % mouseDir)

    # create a session directory within the mouse directory
    path=mouseDir+"/"+sessionName
    if os.path.isdir(path):
        print(path + " session directory exists")
        return True
    else:
        print("Trying to create "+path)
        try:
            os.mkdir(path)
        except OSError:
            print ("Creation of the directory %s failed" % path)
            return False
        else:
            print ("Successfully created the directory %s " % path)
            return True
     
def transferFile(src,dest):
    print("Copy %s to %s" % (src,dest))
    try:
        copyfile(src, dest)
    except :
        print("Copying the file failed")
    else:
        print("%s created" % dest)

def scpFile(user,host,sourceFile,destination):
    mySource=user+"@"+host+":"+sourceFile
    print("source: {}".format(mySource))
    print("destination: {}".format(destination))
    argu=["scp", mySource, destination]
    p = subprocess.Popen(["scp", mySource, destination])
    sts = os.waitpid(p.pid, 0)

        
def get_ip():
    interfaces = netifaces.interfaces()
    for i in interfaces:
        if i.startswith('eth') or i.startswith('enp') or i.startswith('eno'):
            ip = netifaces.ifaddresses(i)[netifaces.AF_INET][0]['addr']
            # iface = netifaces.ifaddresses(i).get(netifaces.AF_INET)
            break
    return ip


def copyDataFilesToDatabase(fileBase, # example "/home/kevin/testMouse-31082020-1306"
                            mouseDir, # example "/ext_drives/d56/data/electro/testMouse"
                            sessionName, # example "testMouse-31082020-1306"
                            master,arenaCameraHost, # computer IPs of master, computer with homebase camera and computer with arena camera, check /autopi_ros/data/setups
                            userName, # example "kevin"
                            arenaCamera=True): # these logicals indicate whether to copy these files

    for ext in ['log']: # this is on master
        src= fileBase+ "."+ ext
        dest=mouseDir+"/"+sessionName+"/"+sessionName+"."+ ext
        transferFile(src,dest)

    if arenaCamera:
        if arenaCameraHost==master:
            for ext in ['arena_top.avi', 'arena_top.log']:
                src= fileBase+ "."+ ext
                dest=mouseDir+"/"+sessionName+"/"+sessionName+"."+ ext
                transferFile(src,dest)
        else :
            for ext in ['arena_top.avi', 'arena_top.log']:
                src=fileBase + "." + ext
                dest=mouseDir+"/"+sessionName+"/"+sessionName+"."+ ext
                scpFile(userName,arenaCameraHost,src,dest)
