#!/usr/bin/env python3

#VERSION_3 VARIABLES OPTIMIZED

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from oni_pkg.msg import cStat
from oni_pkg.msg import fuzzC
import argparse
import os


#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------------------IDENTIFIERS OF THE CAMERAS
cid = { 1:"l" , "r":2, "f":3} #, "back":4 }
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------INCOMING_DATA_HANDLER--SUB-CALLBACK (CORRECTION LIMITER)
#SUB-CALLBACK TO SET FUZZY CORRECTIONS (AUX FUNCTION)
def lim(ce, cg, cv):  #(INPUTS: ECORRECTION_EXPOSURE, CORRECTION_GAIN, CV2_VIDCAP_OBJECT)    

    if ce >= 8187:  
        pass
    elif ce <=0:
        pass   
    elif ce > 0 & ce <8187:
        cv.set(cv2.CAP_PROP_EXPOSURE, ce)
    
    if cg >=255:  
        pass  
    elif cg <=0:
        pass 
    elif cg > 0 & cg <255:
        cv.set(cv2.CAP_PROP_GAIN, cg)
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------------------------------------------CALLBACK TO SET FUZZY CORRECTIONS
def corr(ms):
    
    cel,  cgl  = ms.cel,  ms.cgl  #CORRECTION_EXPOSURE_L,  CORRECTION_GAIN_L   
    cer,  cgr  = ms.cer,  ms.cgr  #CORRECTION_EXPOSURE_R,  CORRECTION_GAIN_R
    cef,  cgf  = ms.cef,  ms.cgf  #CORRECTION_EXPOSURE_F,  CORRECTION_GAIN_F    
    
   #corr_e_b,  corr_g_b  = fuzzy_msg.corr_e_b,  fuzzy_msg.corr_g_b
    lim(cel, cgl, l)
    lim(cer, cgr, r)
    lim(cef, cgf, f)
   #ctrl_limiter(corr_e_b, corr_g_r, back)
#-----------------------------------------------------------------------------------------------------------------------------------------------NODE
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#----------------------------------------------------------------------------------------------------------------------------------------BROADCASTER    
def bct():
       
    rospy.init_node('imgBroadcaster', anonymous = False)                 # 1 -- INIT IM BROADCASTER
    rate   = rospy.Rate(30)                                              # 2 -- SET REFRESH RATE
    rospy.set_param('nodeimBroadcaster', 1)                              # 3 -- ENABLE FLAG FOR THE "imBroadcaster" NODE
    rospy.loginfo('imageBroadcasterNodeInitialized')                     # 4 -- REGISTER THE INITIALIZATION OF THE NODE
    
    pl   = rospy.Publisher('/dImL', Image, queue_size=0.00005)    # LEFT PUBLISHER
    pr   = rospy.Publisher('/dImR', Image, queue_size=0.00005)    # RIGHT PUBLISHER
    pf   = rospy.Publisher('/dImF', Image, queue_size=0.00005)    # FRONT PUBLISHER
   #pb   = rospy.Publisher('/dImB', Image, queue_size=0.04)       # BACK PUBLISHER
    
    pcs  = rospy.Publisher('/cStat',  cStat, queue_size=0.00005)   # (PUBLISHER OF CAMERAS STATUS)
    cs   = rospy.Subscriber('/fCtrl', fuzzC, corr)                # (FUZZY CORRECTION SUBSCRIBER)   
    mcs  = cStat()                                                     # 5 -- INITIALIZE CAMS STATUS MSG (CAMS_STATUS OBJECT) 
    
    while not rospy.is_shutdown():   # MAIN SEQUENCE------------------------------------------
         
        rl, fl = l.read()       # 6 -- READ LEFT FRAME
        rr, fr = r.read()       # 7 -- READ RIGHT FRAME
        rf, ff = f.read()       # 8 -- READ FRONT FRAME
       #fb = back.read()                                                       #   -- READ BACK FRAME  
        
        ffl = cv2.resize(fl, (640,480), interpolation= cv2.INTER_LINEAR)       # RESIZE FRAMES:
        ffr = cv2.resize(fr, (640,480), interpolation= cv2.INTER_LINEAR)
        fff = cv2.resize(ff, (640,480), interpolation= cv2.INTER_LINEAR)
        #ffb = cv2.resize(fb, (640,480), interpolation= cv2.INTER_LINEAR)
        
        
        ml = bdg.cv2_to_imgmsg(ffl, "bgr8")   #MSG LEFT #  9 -- CONVERT FRAMES TO ROS TO PUBLISH
        mr = bdg.cv2_to_imgmsg(ffr, "bgr8")   #MSG RIGHT  
        mf = bdg.cv2_to_imgmsg(fff, "bgr8")   #MSG FRONT
       #mb = bdg.cv2_to_imgmsg(ffb, "bgr8")   #MSG BACK

        mcs.gl = int(l.get(cv2.CAP_PROP_GAIN) )       ;   mcs.el  = int(l.get(cv2.CAP_PROP_EXPOSURE) )   #   10 -- ORGANIZE LEFT STATUS IN MSG
        mcs.gr = int(r.get(cv2.CAP_PROP_GAIN) )       ;   mcs.er  = int(r.get(cv2.CAP_PROP_EXPOSURE) )   #   11 -- ORGANIZE RIGHT STATUS IN MSG
        mcs.gf = int(f.get(cv2.CAP_PROP_GAIN) )       ;   mcs.ef  = int(f.get(cv2.CAP_PROP_EXPOSURE) )   #   12 -- ORGANIZE FRONT STATUS IN MSG
        
        pl.publish(ml)      ;  pr.publish(mr)   ;  pf.publish(mf)          #  12 -- BROADCAST FRAMES (MSGS L, R AND F)

        pcs.publish(mcs)                                                   #  13 -- BROADCAST CAMS STATUS    
        
        if rospy.is_shutdown():
            
            l.release()
            r.release()
            f.release()

#--------------------------------------------------------------------------------------------------------------------------------------MAIN SEQUENCE  
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------------------------------------------------------ GENERAL V4L SETTINGS
if __name__ == "__main__":
    parser  =  argparse.ArgumentParser()
    parser.add_argument("--pixelFormat",      type=str,  default="M,J,P,G",        help='"M,J,P,G" or "Y,U,Y,V", default="M,J,P,G"') #pixel_format
    parser.add_argument("--fps",               type=int,  default=30,               help="default=30")   
    parser.add_argument("--field",             type=str,  default="top",            help="top, bottom, any or none default=top=30fps") #general
    parser.add_argument("--imageWidth", type=int,  default=1280,                   help="default=1280")
    parser.add_argument("--imageHeight",type=int,  default=720,                    help="default=720")   
    parser.add_argument("--exposureAutoPriority", action="store_true", default=0, help="(bool) default=0")             # FLAG_ONLY_FOR_THE A30330
    parser.add_argument("--exposureAuto",          action="store_true", default=1, help="(1=manual,3=auto) default=1")  # flag
    parser.add_argument("--exposureAbsolute",      type=int,  default=3,           help="min=3 max=2047 step=1 default=500")  
    parser.add_argument("--gain",                   type=int,  default=10,          help="min=0 max=255 step=1 default=250")   
    parser.add_argument("--backlightCompensation", type=int,  default=0,           help="min=0 max=1 step=1 default=0")#general
    parser.add_argument("--brightness", type=int,  default=4,                       help="min=-64 max=64 step=1 default=4")#general
    parser.add_argument("--contrast",   type=int,  default=7,                       help="min=0 max=95 step=1 default=20")#general
    parser.add_argument("--saturation", type=int,  default=1,                       help="min=0 max=100 step=1 default=0")#general
   
    #args = parser.parse_args()
    args, unknown = parser.parse_known_args()
    print(args)

#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-----------------------------------------------------------------------------------------------------------LOAD THE INITIAL SETTINGS TO THE CAMERAS
    for i in range (len(cid)):

        c="v4l2-ctl -d "+str(i+1)+" --set-ctrl="

        os.system(c+'exposure_auto='+str(args.exposureAuto))
        os.system(c+'exposure_absolute='+str(args.exposureAbsolute))
        os.system(c+'gain='+str(args.gain))
        os.system(c+'backlight_compensation='+str(args.backlightCompensation))
        os.system(c+'brightness='+str(args.brightness))
        os.system(c+'contrast='+str(args.contrast))
        os.system(c+'saturation='+str(args.saturation)) 
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-----------------------------------------------------------------------------------------------------------------------------CAPTURE INITIALIZATION
    l = cv2.VideoCapture(1)
    r = cv2.VideoCapture(2) 
    f = cv2.VideoCapture(3) 
#------------------------------------------------------------------------------------------------------------------SET RESOLUTION, VIDEO FORMAT, FPS
#----------------------------THIS IS CONFIGURED HERE SINCE AFTER INITIALIZING THE VIDEOCAPTURE OBJECTS OF CV2 THE VIDEO FORMAT CONFIG IS OVERWRITTEN

    vf    = (args.pixelFormat).split(",")
    fcc   = cv2.VideoWriter_fourcc(vf[0], vf[1], vf[2], vf[3])

    l.set(cv2.CAP_PROP_FOURCC, fcc)               
    l.set(cv2.CAP_PROP_FRAME_WIDTH, args.imageWidth)
    l.set(cv2.CAP_PROP_FRAME_HEIGHT, args.imageHeight)
    l.set(cv2.CAP_PROP_FPS, args.fps)

    r.set(cv2.CAP_PROP_FOURCC, fcc)               
    r.set(cv2.CAP_PROP_FRAME_WIDTH, args.imageWidth)
    r.set(cv2.CAP_PROP_FRAME_HEIGHT, args.imageHeight)
    r.set(cv2.CAP_PROP_FPS, args.fps)

    f.set(cv2.CAP_PROP_FOURCC, fcc)               
    f.set(cv2.CAP_PROP_FRAME_WIDTH, args.imageWidth)
    f.set(cv2.CAP_PROP_FRAME_HEIGHT, args.imageHeight)
    f.set(cv2.CAP_PROP_FPS, args.fps)

    rospy.loginfo('DefaultSettingsLoadedToSensors')  # -- REGISTER THE INITIALIZATION OF THE SENSORS

    bdg = CvBridge()                     # -- INITIALIZE ROS-CV2 BRIDGE 

    try:

        bct()                            # -- DEPLOY BROADCASTER NODE

    except rospy.ROSInterruptException:
        pass
