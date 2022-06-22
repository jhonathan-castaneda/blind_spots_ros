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
cams_ids = { 1:"left" , "right":2} #, "front":3, "back":4 }
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
#-------------------------------------------------------------------------------------------------------------------------------------INITIALIZATION
vf       = (args.pixel_format).split(",")
fourcc   = cv2.VideoWriter_fourcc(vf[0], vf[1], vf[2], vf[3])
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------------------------------------------SET RESOLUTION, VIDEO FORMAT, FPS
#----------------------------THIS IS CONFIGURED HERE SINCE AFTER INITIALIZING THE VIDEOCAPTURE OBJECTS OF CV2 THE VIDEO FORMAT CONFIG IS OVERWRITTEN
left  = cv2.VideoCapture(1)
right = cv2.VideoCapture(2) 

left.set(cv2.CAP_PROP_FOURCC, fourcc)               
left.set(cv2.CAP_PROP_FRAME_WIDTH, args.image_width)
left.set(cv2.CAP_PROP_FRAME_HEIGHT, args.image_height)
left.set(cv2.CAP_PROP_FPS, args.fps)

right.set(cv2.CAP_PROP_FOURCC, fourcc)               
right.set(cv2.CAP_PROP_FRAME_WIDTH, args.image_width)
right.set(cv2.CAP_PROP_FRAME_HEIGHT, args.image_height)
right.set(cv2.CAP_PROP_FPS, args.fps)

bridge   = CvBridge() 
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------------------------------------------------------INCOMING_DATA_HANDLER
#CALLBACK TO SET FUZZY CORRECTIONS (AUX FUNCTION)
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
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#CALLBACK TO SET FUZZY CORRECTIONS
def set_ctrls(fuzzy_msg):
    
    corr_e_l = fuzzy_msg.corr_e_l
    corr_e_r = fuzzy_msg.corr_e_r
    corr_g_l = fuzzy_msg.corr_g_l
    corr_g_r = fuzzy_msg.corr_g_r
    
    ctrl_limiter(corr_e_l, corr_g_l, left)
    ctrl_limiter(corr_e_r, corr_g_r, right)
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#----------------------------------------------------------------------------------------------------------------------------------------BROADCASTER    
def im_broadcaster():
    
    pub_l              = rospy.Publisher('/d_img_l', Image, queue_size=0.04)    
    pub_r              = rospy.Publisher('/d_img_r', Image, queue_size=0.04)
    pub_cams_status    = rospy.Publisher('/pub_cams_status', cams_status, queue_size=0.04)
    
    ctrls_sub          = rospy.Subscriber('/fuzzy_ctrl', fuzzy_corr, set_ctrls)
    
    rospy.init_node('img_broadcaster', anonymous = False)    
    rate   = rospy.Rate(30) 
    
    msg_cs = cams_status()
    
    while not rospy.is_shutdown():
         
        ret_L, FRAME_L = left.read()                    
        ret_R, FRAME_R = right.read()		
        
        gain_l = left.get(cv2.CAP_PROP_GAIN)          ;  gain_r = right.get(cv2.CAP_PROP_GAIN)
        exp_l = left.get(cv2.CAP_PROP_EXPOSURE)       ;  exp_r = right.get(cv2.CAP_PROP_EXPOSURE)
         
        msg_l = bridge.cv2_to_imgmsg(FRAME_L, "bgr8") ;  msg_r = bridge.cv2_to_imgmsg(FRAME_R, "bgr8")

        pub_l.publish(msg_l)                          ; pub_r.publish(msg_r)

        msg_cs.gain_l = int(gain_l)
        msg_cs.gain_r = int(gain_r)
        msg_cs.exp_l  = int(exp_l)
        msg_cs.exp_r  = int(exp_r)
        
        pub_cams_status.publish(msg_cs)    
        
        if rospy.is_shutdown():
            
            left.release()
            right.release()
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#--------------------------------------------------------------------------------------------------------------------------------------MAIN SEQUENCE  
try:
    im_broadcaster()
    
except rospy.ROSInterruptException:
    pass
