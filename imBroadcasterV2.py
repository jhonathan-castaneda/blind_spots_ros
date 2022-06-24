#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from oni_pkg.msg import cams_status
from oni_pkg.msg import fuzzy_corr
import argparse
import os
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#----------------------------------------------------------------------------------------------------------------INCOMING_DATA_HANDLER--SUB-CALLBACK
#SUB-CALLBACK TO SET FUZZY CORRECTIONS (AUX FUNCTION)
def ctrl_limiter(corr_e, corr_g, cv_cap):    

    if corr_e >= 8187:  
        pass
    elif corr_e <=0:
        pass   
    elif corr_e > 0 & corr_e <8187:
        cv_cap.set(cv2.CAP_PROP_EXPOSURE, corr_e)
    if corr_g >=255:  
        pass  
    elif corr_g <=0:
        pass 
    elif corr_g > 0 & corr_g <255:
        cv_cap.set(cv2.CAP_PROP_GAIN, corr_g)
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------------------------------------------CALLBACK TO SET FUZZY CORRECTIONS
def set_ctrls(fuzzy_msg):
    
    corr_e_l,  corr_g_l  = fuzzy_msg.corr_e_l,  fuzzy_msg.corr_g_l   
    corr_e_r,  corr_g_r  = fuzzy_msg.corr_e_r,  fuzzy_msg.corr_g_r
    corr_e_f,  corr_g_f  = fuzzy_msg.corr_e_f,  fuzzy_msg.corr_g_f      
   #corr_e_b,  corr_g_b  = fuzzy_msg.corr_e_b,  fuzzy_msg.corr_g_b
    ctrl_limiter(corr_e_l, corr_g_l, left)
    ctrl_limiter(corr_e_r, corr_g_r, right)
    ctrl_limiter(corr_e_f, corr_g_f, front)
   #ctrl_limiter(corr_e_b, corr_g_r, back)
#-----------------------------------------------------------------------------------------------------------------------------------------------NODE
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#----------------------------------------------------------------------------------------------------------------------------------------BROADCASTER    
def im_broadcaster():
       
    rospy.init_node('img_broadcaster', anonymous = False)                # 1 -- INIT IM BROADCASTER
    rate   = rospy.Rate(30)                                              # 2 -- SET REFRESH RATE
    rospy.set_param('nodeimBroadcaster', 1)                              # 3 -- ENABLE FLAG FOR THE "imBroadcaster" NODE
    rospy.loginfo('imageBroadcasterNodeInitialized')                     # 4 -- REGISTER THE INITIALIZATION OF THE NODE
    
    pub_l              = rospy.Publisher('/d_img_l', Image, queue_size=0.0005)    # LEFT PUBLISHER
    pub_r              = rospy.Publisher('/d_img_r', Image, queue_size=0.0005)    # RIGHT PUBLISHER
    pub_f              = rospy.Publisher('/d_img_f', Image, queue_size=0.0005)    # FRONT PUBLISHER
   #pub_b              = rospy.Publisher('/d_img_b', Image, queue_size=0.04)    # BACK PUBLISHER
    
    pub_cams_status    = rospy.Publisher('/pub_cams_status', cams_status, queue_size=0.0005) # CONTROL STATUS PUBLISHER
    
    ctrls_sub          = rospy.Subscriber('/fuzzy_ctrl', fuzzy_corr, set_ctrls)            # FUZZY CORRECTION RETRIEVER    

    msg_cs = cams_status()                                                                 # 5 -- INITIALIZE STATUS MSG OBJECT 
    
    while not rospy.is_shutdown():                                                         # MANI SEQUENCE------------------------------------------
         
        ret_L, FRAME_L = left.read()                                                       # 6 -- READ LEFT FRAME
        ret_R, FRAME_R = right.read()                                                      # 7 -- READ RIGHT FRAME
        ret_F, FRAME_F = front.read()                                                      # 8 -- READ FRONT FRAME
       #ret_B, FRAME_B = back.read()                                                       #   -- READ BACK FRAME  
        
        FRAME_L = cv2.resize(FRAME_L, (640,480), interpolation= cv2.INTER_LINEAR)          # RESIZE FRAMES:
        FRAME_R = cv2.resize(FRAME_R, (640,480), interpolation= cv2.INTER_LINEAR)
        FRAME_F = cv2.resize(FRAME_F, (640,480), interpolation= cv2.INTER_LINEAR)
        #FRAME_B = cv2.resize(FRAME_B, (640,480), interpolation= cv2.INTER_LINEAR)
        
        
        msg_l = bridge.cv2_to_imgmsg(FRAME_L, "bgr8")  #  9 -- CONVERT FRAMES TO ROS TO PUBLISH
        msg_r = bridge.cv2_to_imgmsg(FRAME_R, "bgr8")     
        msg_f = bridge.cv2_to_imgmsg(FRAME_F, "bgr8") 
       #msg_b = bridge.cv2_to_imgmsg(FRAME_B, "bgr8")

        msg_cs.gain_l = int(left.get(cv2.CAP_PROP_GAIN)  )       ;   msg_cs.exp_l  = int(left.get(cv2.CAP_PROP_EXPOSURE) )    #   10 -- ORGANIZE LEFT STATUS IN MSG
        msg_cs.gain_r = int(right.get(cv2.CAP_PROP_GAIN) )       ;   msg_cs.exp_r  = int(right.get(cv2.CAP_PROP_EXPOSURE) )   #   11 -- ORGANIZE RIGHT STATUS IN MSG
        msg_cs.gain_f = int(front.get(cv2.CAP_PROP_GAIN) )       ;   msg_cs.exp_f  = int(front.get(cv2.CAP_PROP_EXPOSURE) )   #   12 -- ORGANIZE FRONT STATUS IN MSG
        
        pub_l.publish(msg_l)      ;  pub_r.publish(msg_r)   ;  pub_f.publish(msg_f) #  12 -- BROADCAST FRAMES

        pub_cams_status.publish(msg_cs)                                             #  13 -- BROADCAST CAMS STATUS    
        
        if rospy.is_shutdown():
            
            left.release()
            right.release()
            front.release()

#--------------------------------------------------------------------------------------------------------------------------------------MAIN SEQUENCE  
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------------------------------------------------------ GENERAL V4L SETTINGS
if __name__ == "__main__":
    parser  =  argparse.ArgumentParser()
    parser.add_argument("--pixel_format",      type=str,  default="M,J,P,G",        help='"M,J,P,G" or "Y,U,Y,V", default="M,J,P,G"') #pixel_format
    parser.add_argument("--fps",               type=int,  default=30,               help="default=30")   
    parser.add_argument("--field",             type=str,  default="top",            help="top, bottom, any or none default=top=30fps") #general
    parser.add_argument("--image_width", type=int,  default=1280,                   help="default=1280")
    parser.add_argument("--image_height",type=int,  default=720,                    help="default=720")   
    parser.add_argument("--exposure_auto_priority", action="store_true", default=0, help="(bool) default=0")             # FLAG_ONLY_FOR_THE A30330
    parser.add_argument("--exposure_auto",          action="store_true", default=1, help="(1=manual,3=auto) default=1")  # flag
    parser.add_argument("--exposure_absolute",      type=int,  default=3,           help="min=3 max=2047 step=1 default=500")  
    parser.add_argument("--gain",                   type=int,  default=10,          help="min=0 max=255 step=1 default=250")   
    parser.add_argument("--backlight_compensation", type=int,  default=0,           help="min=0 max=1 step=1 default=0")#general
    parser.add_argument("--brightness", type=int,  default=4,                       help="min=-64 max=64 step=1 default=4")#general
    parser.add_argument("--contrast",   type=int,  default=7,                       help="min=0 max=95 step=1 default=20")#general
    parser.add_argument("--saturation", type=int,  default=1,                       help="min=0 max=100 step=1 default=0")#general
   
    #args = parser.parse_args()
    args, unknown = parser.parse_known_args()
    print(args)
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------------------IDENTIFIERS OF THE CAMERAS
    cams_ids = { 1:"left" , "right":2, "front":3} #, "back":4 }
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-----------------------------------------------------------------------------------------------------------LOAD THE INITIAL SETTINGS TO THE CAMERAS
    for i in range (len(cams_ids)):

        ctr="v4l2-ctl -d " +str(i+1)+" --set-ctrl="

        os.system(ctr+'exposure_auto='+str(args.exposure_auto))
        os.system(ctr+'exposure_absolute='+str(args.exposure_absolute))
        os.system(ctr+'gain='+str(args.gain))
        os.system(ctr+'backlight_compensation='+str(args.backlight_compensation))
        os.system(ctr+'brightness='+str(args.brightness))
        os.system(ctr+'contrast='+str(args.contrast))
        os.system(ctr+'saturation='+str(args.saturation)) 
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-----------------------------------------------------------------------------------------------------------------------------CAPTURE INITIALIZATION
    left  = cv2.VideoCapture(1)
    right = cv2.VideoCapture(2) 
    front = cv2.VideoCapture(3) 
#------------------------------------------------------------------------------------------------------------------SET RESOLUTION, VIDEO FORMAT, FPS
#----------------------------THIS IS CONFIGURED HERE SINCE AFTER INITIALIZING THE VIDEOCAPTURE OBJECTS OF CV2 THE VIDEO FORMAT CONFIG IS OVERWRITTEN

    vf       = (args.pixel_format).split(",")
    fourcc   = cv2.VideoWriter_fourcc(vf[0], vf[1], vf[2], vf[3])

    left.set(cv2.CAP_PROP_FOURCC, fourcc)               
    left.set(cv2.CAP_PROP_FRAME_WIDTH, args.image_width)
    left.set(cv2.CAP_PROP_FRAME_HEIGHT, args.image_height)
    left.set(cv2.CAP_PROP_FPS, args.fps)

    right.set(cv2.CAP_PROP_FOURCC, fourcc)               
    right.set(cv2.CAP_PROP_FRAME_WIDTH, args.image_width)
    right.set(cv2.CAP_PROP_FRAME_HEIGHT, args.image_height)
    right.set(cv2.CAP_PROP_FPS, args.fps)

    front.set(cv2.CAP_PROP_FOURCC, fourcc)               
    front.set(cv2.CAP_PROP_FRAME_WIDTH, args.image_width)
    front.set(cv2.CAP_PROP_FRAME_HEIGHT, args.image_height)
    front.set(cv2.CAP_PROP_FPS, args.fps)

    rospy.loginfo('DefaultSettingsLoadedToSensors')  # -- REGISTER THE INITIALIZATION OF THE SENSORS

    bridge   = CvBridge()                            # -- INITIALIZE ROS-CV2 BRIDGE 

    try:

        im_broadcaster()                             # -- DEPLOY BROADCASTER NODE

    except rospy.ROSInterruptException:
        pass
