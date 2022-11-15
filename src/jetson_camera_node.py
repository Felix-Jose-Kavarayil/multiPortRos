#!/usr/bin/env python3
from __future__ import print_function
from __future__ import division

import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Time
import time

def opencv_version():
    return int(cv2.__version__.split('.')[0])

def gstreamer_video_encoding_h265_pipeline(fileName):
    return ("appsrc ! "
            "video/x-raw, format=BGR ! "
            "queue ! "
            "videoconvert ! "
            "omxh265enc ! "
            "matroskamux ! "
            "filesink location=%s" % (fileName))
def gstreamer_pihdsrc_pipeline(
        sensorId=0,
        captureWidth=1920, 
        captureHeight=1080,
        displayWidth=640,
        displayHeight=480,
        top=0,
        bottom=1080,
        left=258,
        right=1662,
        frameRate=60,
        flipMethod=0):
    return (
        "nvarguscamerasrc sensor-id=%d wbmode=8 tnr-mode=1 tnr-strength=-1  exposurecompensation=0.7 ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv top=%d bottom=%d left=%d right=%d flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "videobox ! " 
        "video/x-raw, format=(string)BGR ! "
        "videobalance saturation=0.0 contrast=1.3 ! "
        "appsink drop=True"
        % (
            sensorId,
            captureWidth,
            captureHeight,
            frameRate,
            top,
            bottom,
            left,
            right,
            flipMethod,
            displayWidth,
            displayHeight,
        )
    )

def gstreamer_webcam_pipeline(
        sensorId=0,
        displayWidth=640,
        displayHeight=480,
        frameRate=30):
    if displayWidth !=640 or displayHeight!=480:
        print("Error: Code with webcam not tested with image size different than 640x480")
        return
    s1="v4l2src device=/dev/video{} ! video/x-raw, width={}, height={}, framerate=30/1 ! videoconvert ! video/x-raw,format=BGR ! appsink".format(sensorId,displayWidth,displayHeight,frameRate)
   # s2="v4l2src device=/dev/video0 ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! video/x-raw,format=BGR ! appsink"
   # print(s1)
    print(s1)
    return (s1) 


# this class does most of the work besides getting frames from
class CameraNode:
    def __init__(self):
        # Init node
        rospy.init_node('jetson_camera_node')

        self.nodeName=rospy.get_name()
        # parameters
        camera_topic = rospy.get_param('~camera_topic', '/jetson_cam')
        sensor_id = int(rospy.get_param('~sensor_id', '0'))
        image_width = int(rospy.get_param('~image_width', '640')) # for the webcam, you need width of 640 and height of 480
        image_height = int(rospy.get_param('~image_height', '640'))
        camera_type = rospy.get_param('~camera_type', 'pi')
        captureWidth= int(rospy.get_param('~capture_width', '1920')) #1920
        captureHeight= int(rospy.get_param('~capture_height', '1080')) #1080
        top=int(rospy.get_param('~crop_top', '0'))
        bottom=int(rospy.get_param('~crop_bottom', '1080'))
        left=int(rospy.get_param('~crop_left', '420')) # 258
        right=int(rospy.get_param('~crop_right', '1500')) # 1662
        flipMethod = 1 # vertical flip

        fps = int(rospy.get_param('~fps', '30'))
        queue_size = int(rospy.get_param('~queue_size', '1'))   # queue for the publishers
        camera_frame_id = rospy.get_param('~camera_frame_id', 'jetson_cam')
        output_path = rospy.get_param('~output_path', '')   # if not empty, record a video and save time_stamps
        timeStampVideo= rospy.get_param('~time_stamp_video', '')   # if not empty, time stamp the video

         
        # ratio of frame captured to frame published
        # to reduce cpu usage of node
        self.publish_every=2
        
        if len(output_path):
            self.record_video = True
            rospy.loginfo("[{}] video recorded to {}".format(self.nodeName, output_path))
        else:
            self.record_video = False
            rospy.loginfo("[{}] no video recorded".format(self.nodeName))
            rospy.loginfo("[{}] set {}/output_path ROS parameter if you want video".format(self.nodeName,self.nodeName))

        image_topic = camera_topic + '/image_raw'
        camera_info_topic = camera_topic + '/camera_info'

        rospy.loginfo("[{}] topic: {}".format(self.nodeName, camera_topic))
        rospy.loginfo("[{}] publishing to {} and {}".format(self.nodeName,image_topic, camera_info_topic))

               
        # Create topics publishers
        self.pubImage = rospy.Publisher(image_topic, Image, queue_size=queue_size)
        self.pubCameraInfo = rospy.Publisher(camera_info_topic, CameraInfo, queue_size=queue_size)

        # Messages
        self.image_msg = Image()
        self.camera_info_msg = CameraInfo()

        self.camera_info_msg.header.frame_id = camera_frame_id
        self.camera_info_msg.height = image_height
        self.camera_info_msg.width = image_width

        
        if camera_type == "pi":
            rospy.loginfo(gstreamer_pihdsrc_pipeline(sensorId = sensor_id,
                                                     frameRate=fps,
                                                     captureWidth=captureWidth,
                                                     captureHeight=captureHeight,
                                                     top=top,bottom=bottom,left=left,right=right,
                                                     displayWidth=image_width,
                                                     displayHeight=image_height))
                    
            self.cap = cv2.VideoCapture(gstreamer_pihdsrc_pipeline(sensorId = sensor_id,
                                                                   frameRate=fps,
                                                                   captureWidth=captureWidth,
                                                                   captureHeight=captureHeight,
                                                                   top=top,bottom=bottom,left=left,right=right,
                                                                   displayWidth=image_width,
                                                                   displayHeight=image_height,
                                                                   flipMethod=flipMethod),
                                        cv2.CAP_GSTREAMER)

        if camera_type == "webcam" :
            rospy.loginfo(gstreamer_webcam_pipeline(sensorId = sensor_id,
                                                    displayWidth=image_width,
                                                    displayHeight=image_height,
                                                    frameRate=fps))
            self.cap = cv2.VideoCapture(gstreamer_webcam_pipeline(sensorId = sensor_id,
                                                                  displayWidth=image_width,
                                                                  displayHeight=image_height,
                                                                  frameRate=fps))

        if not self.cap.isOpened():
            rospy.logerr("Failed to open the video capture in {}".format(self.nodeName))
            exit()

           
        # Create a video writer and a timestam writer
        if output_path:
            self.video_output_path = output_path

            self.video_writer = cv2.VideoWriter(gstreamer_video_encoding_h265_pipeline(fileName=self.video_output_path),
                             cv2.CAP_GSTREAMER,
                             0,
                             float(fps),
                             (int(image_width),int(image_height)))
            if not self.video_writer.isOpened():
                rospy.logerr("Failed to open video writer in {}".format(self.nodeName))
                self.cap.release()
                exit()

            temp = output_path.split('.')
            temp[-1] = 'log'
            self.timestamps_output_path = str.join('.', temp)
            temp[-1] = 'errors'
            self.error_log_path = str.join('.', temp)
            del temp
            self.timestamp_writer = open(self.timestamps_output_path, "w")
            self.timestamp_writer.write("frame_number time\n")
            rospy.loginfo("[{}] Timestamps will be saved in {}".format(self.nodeName,self.timestamps_output_path))
            rospy.loginfo("[{}] Video will be saved in {}".format(self.nodeName,self.video_output_path))
        else:
            self.video_writer = None
            self.timestamp_writer = None
        self.errors_writer = None

        self.bridge = CvBridge()
      
        #
        # start the camera loop
        #
        start = time.time()
        self.frame_num = 0
        while not rospy.is_shutdown() :

            ret_val, Frame = self.cap.read()
            if ret_val == True:                
                self.camera_info_msg.header.stamp = rospy.get_rostime() # get timestamp for the frame
                self.camera_info_msg.header.seq = self.frame_num

                
                # if timeStampVideo is not length of 0, add the frame number and time to image
                if len(timeStampVideo):
                    frameTime = time.time()    
                    seconds = frameTime - start                
                    cv2.putText(Frame,str(self.frame_num)+" " + str(round(seconds,2)), 
                                (10,30),
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                0.7,
                                (50,50,255),
                                2)            
                
                # Save video and timestamps
                if self.video_writer:
                    self.video_writer.write(Frame)
                    
                if self.timestamp_writer:
                    self.timestamp_writer.write("%d %10d.%09d\n" % (self.frame_num, self.camera_info_msg.header.stamp.secs, self.camera_info_msg.header.stamp.nsecs))
               
                
                # Publish frame and info to image_raw and camera_info topics
                # This if statement is there to reduce CPU usage of this node
                if self.frame_num % self.publish_every == 0 :
                    # convert the cv image to ROS image message
                    try:
                        try:
                            #rospy.loginfo('[{}] publish frame_num {}'.format(self.nodeName,self.frame_num) )
                            self.image_msg = self.bridge.cv2_to_imgmsg(Frame, encoding="bgr8")
                        except CvBridgeError as e:
                            rospy.logerr('[{}][CameraNode][run_camera] Converting Image Error. '.format(self.nodeName) + str(e)) 
                        self.pubImage.publish(self.image_msg)
                        self.pubCameraInfo.publish(self.camera_info_msg)
                    except:
                        pass
            else:
                break

            ## move to next frame
            self.frame_num+=1
            
        self.terminate()
        
    def terminate(self):
        
        rospy.loginfo("[{}] leaving run_camera()".format(self.nodeName))
        self.cap.release()
        
        if self.record_video:
            self.timestamp_writer.close()
            self.video_writer.release()

            
            # get the number of frames (lines - header) in log file
            with open(self.timestamps_output_path) as f:
                for nFrameLog, l in enumerate(f):
                    pass
            rospy.loginfo("[{}] number of frames in ".format(self.nodeName) + self.timestamps_output_path + " {}".format(nFrameLog))
                
            ## get the number of frames in video
            cap = cv2.VideoCapture(self.video_output_path)
            nFrameVideo = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            rospy.loginfo("[{}] number of frames in ".format(self.nodeName) + self.video_output_path + " {}".format(nFrameVideo))

            ## throw an error if the syncrhonization failed
            if nFrameVideo != nFrameLog:
                rospy.logerr("[{}] video synchronization failed for {} and {}".format(self.nodeName,self.video_output_path,self.timestamps_output_path))
                
               
if __name__ == '__main__':
    try:
        camera_node = CameraNode()
    except KeyboardInterrupt:
        rospy.logerr("[{}] Shutting down...".format(self.nodeName))
        camera_node.terminate()
