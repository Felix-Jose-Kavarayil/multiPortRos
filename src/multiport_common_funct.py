import rosnode
import rospy

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
