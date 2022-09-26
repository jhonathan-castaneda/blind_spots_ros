#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import argparse
import os
#-----------------------------------------------------------------------------------------------------------------------------------------------NODE
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#----------------------------------------------------------------------------------------------------------------------------------------BROADCASTER    
def bct():
       
    rospy.init_node('imgBroadcasterSONY', anonymous = False)                 # 1 -- INIT IM BROADCASTER
    rate   = rospy.Rate(30)                                                  # 2 -- SET REFRESH RATE
    rospy.set_param('nodeimBroadcasterSONY', 1)                              # 3 -- ENABLE FLAG FOR THE "imBroadcaster" NODE
    rospy.loginfo('imageBroadcasterSONYNodeInitialized')                     # 4 -- REGISTER THE INITIALIZATION OF THE NODE
    
    p     = rospy.Publisher('/dImSONY',  Image, queue_size=0.00005)          # DISTORTED IMAGE PUBLISHER    
    while not rospy.is_shutdown():   # MAIN SEQUENCE------------------------------------------
         
        rf, ff = f.read()       # 8 -- READ FRONT FRAME
        rb, fb = b.read()       # 9 -- READ BACK FRAME  
        
        ff = cv2.cvtColor(ff, cv2.COLOR_BGR2GRAY)
        fb = cv2.cvtColor(fb, cv2.COLOR_BGR2GRAY)
                
        ff = cv2.flip(ff,1)     ## CORRECT THE MIRROR OF SONY B        
        fb = cv2.flip(fb,1)     ## CORRECT THE MIRROR OF SONY B
        c3 = np.zeros_like(ff)
        
        im  = cv2.merge((fl,fr,c3))                # JOIN FRAMES 
        m   = bdg.cv2_to_imgmsg(im, "bgr8")  
        
        p.publish(m)
        pcs.publish(mcs)                           #  13 -- BROADCAST CAMS STATUS    
        
        if rospy.is_shutdown():
            f.release()
            b.release()
#--------------------------------------------------------------------------------------------------------------------------------------MAIN SEQUENCE  
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------------------------------------------------------ GENERAL V4L SETTINGS
if __name__ == "__main__":
    parser  =  argparse.ArgumentParser()
    parser.add_argument("--pixelFormat",      type=str,  default="M,J,P,G",        help='"M,J,P,G" or "Y,U,Y,V", default="M,J,P,G"') #pixel_format
    parser.add_argument("--fps",               type=int,  default=30,               help="default=30")   
    parser.add_argument("--field",             type=str,  default="top",            help="top, bottom, any or none default=top=30fps") #general
    parser.add_argument("--imageWidth", type=int,  default=720,                   help="default=1280")
    parser.add_argument("--imageHeight",type=int,  default=480,                    help="default=720")   
    args, unknown = parser.parse_known_args()
    print(args)
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-----------------------------------------------------------------------------------------------------------------------------CAPTURE INITIALIZATION
    f,b   = cv2.VideoCapture(0) , cv2.VideoCapture(1) 
#------------------------------------------------------------------------------------------------------------------SET RESOLUTION, VIDEO FORMAT, FPS
#----------------------------THIS IS CONFIGURED HERE SINCE AFTER INITIALIZING THE VIDEOCAPTURE OBJECTS OF CV2 THE VIDEO FORMAT CONFIG IS OVERWRITTEN
    vf    = (args.pixelFormat).split(",")
    fcc   = cv2.VideoWriter_fourcc(vf[0], vf[1], vf[2], vf[3])
    
    f.set(cv2.CAP_PROP_FOURCC, fcc)               
    f.set(cv2.CAP_PROP_FRAME_WIDTH, args.imageWidth)
    f.set(cv2.CAP_PROP_FRAME_HEIGHT, args.imageHeight)
    f.set(cv2.CAP_PROP_FPS, args.fps)
    
    b.set(cv2.CAP_PROP_FOURCC, fcc)               
    b.set(cv2.CAP_PROP_FRAME_WIDTH, args.imageWidth)
    b.set(cv2.CAP_PROP_FRAME_HEIGHT, args.imageHeight)
    b.set(cv2.CAP_PROP_FPS, args.fps)
    
    rospy.loginfo('DefaultSettingsLoadedToSensors')  # -- REGISTER THE INITIALIZATION OF THE SENSORS
    bdg = CvBridge()                                 # -- INITIALIZE ROS-CV2 BRIDGE 

    try:
        bct()                                        # -- DEPLOY BROADCASTER NODE
    except rospy.ROSInterruptException:
        pass
