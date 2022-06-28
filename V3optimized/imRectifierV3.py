#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import message_filters
import argparse
import yaml
#import time
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------IDENTIFIERS OF THE CAMERAS(DICTIONARY)
cid =  { 1:"l" , 2:"r", 3:"f"}#, 4:"back"}
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#-----------------------------------------------------------------------------CONVERT LIST TO NO ARRAY TO IMPORT PINHOLE MTX FROM ROS PARAMS. SERVER
def list2mtx_k(data_list):  
    fax   = data_list[0]; fay = data_list[4]; ua0 = data_list[2]; va0 = data_list[5] 
    k_mtx = np.array([[fax, 0, ua0],[0, fay, va0],[0,0,1]])          
    return(k_mtx)
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#--------------------------------------------------------------------------CONVERT LIST TO NO ARRAY TO IMPORT DISTORTION MTX FROM ROS PARAMS. SERVER
def list2mtx_d(data_list):
    d_mtx = np.array( [ data_list[0],  data_list[1],  data_list[2],  data_list[3],  data_list[4] ])          
    return(d_mtx)
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#---------------------------------------------------------------------------------------IMPORT CALIB DATA FROM YAML AND UPLOAD TO ROS PARAMS. SERVER
def imp(cp):
    
    with open(cp) as c:  
    
        dt  = yaml.load(c, Loader=yaml.FullLoader) # "dt"---> data 
        
    for i in range (len(cid)):
        
        dic  = dt.get( cid[i+1] )  #AT THE 1ST ITERATION 0+1 == 1--> EQUAL TO "left" IN THE CAMS DIC OF ABOVE  # "dic" --> dictionary

        kl   = dic.get("k_mtx")  #EXTRACT THE CAMERA MATRIX                    "kl"  --> k matrix in list
        dl   = dic.get("d_mtx")  #EXTRACT THE DISTORTION MATRIX                "dl"  --> distortion matrix in list
        
        rospy.set_param( cid[i+1]+'MtxK', kl)
        rospy.set_param( cid[i+1]+'MtxD', dl)    
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------------------CALLBACK TO RECTIFY IMAGES             
def rec( ml, mr, mf): #, dimg_b):
    
    l = bdg.imgmsg_to_cv2( ml ,"bgr8")
    r = bdg.imgmsg_to_cv2( mr ,"bgr8")
    f = bdg.imgmsg_to_cv2( mf ,"bgr8")
   #b = bdg.imgmsg_to_cv2( dimg_b ,"bgr8")
       
    #update this to retrieve the dist and k matrix from each camera instead using the same one for all of them
    
    ll = cv2.undistort(l, km, dm, None, nmt)
    rr = cv2.undistort(r, km, dm, None, nmt)
    ff = cv2.undistort(f, km, dm, None, nmt)
   #b = cv2.undistort(b, k_mtx, d_mtx, None, nmt)
    
    ml = bdg.cv2_to_imgmsg(ll, "bgr8")
    mr = bdg.cv2_to_imgmsg(rr, "bgr8")
    mf = bdg.cv2_to_imgmsg(ff, "bgr8")
   #mb = bdg.cv2_to_imgmsg(b, "bgr8")
    
    pl.publish(ml)
    pr.publish(mr)             
    pf.publish(mf)
   #pb.publish(mb)

    
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#---------------------------------------------------------------------------------------------------------------------MAIN_SEQUENCE

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='calibration data from cameras')
    
    parser.add_argument('-c','--calibDataPath',  type=str,  default='/home/sierra/oni_dir/oni_ws/src/oni_pkg/yaml_config/cams_data.yaml',  help='img calib data, file path')
        
    args, unknown = parser.parse_known_args()
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#--------------------------------------------------------------------------------------------------------------------------------------MAIN SEQUENCE
    try:

        bdg   = CvBridge()
        imp(args.calibDataPath)                     #IMPORT INTRINSIC CAMERA PARAMETERS
                                                    #PENDING TO IMPORT THE 4 MTX FOR THE 4 CAMERAS
        kl = rospy.get_param('lMtxK') # k matrix in list
        dl = rospy.get_param('lMtxD') #  distortion matrix in list

        km  = list2mtx_k(kl)  
        dm  = list2mtx_d(dl)  

        nmt, roi = cv2.getOptimalNewCameraMatrix(km, dm, (640 , 480), 0, (640 , 480)) # NEW CAMERA MATRIX
        #----------------------------------------------------------------------------------------------------------------------------
        rospy.init_node('imRectifier', anonymous = False)    
        rate   = rospy.Rate(30)    
        rospy.set_param('nodeimRectifier', 1)                              #  -- ENABLE FLAG FOR THE "imBroadcaster" NODE
        rospy.loginfo('imRectifierNodeInitialized')                        #  -- REGISTER THE INITIALIZATION OF THE NODE

        sl         = message_filters.Subscriber('/dImL', Image)
        sr         = message_filters.Subscriber('/dImR', Image)    
        sf         = message_filters.Subscriber('/dImF', Image)
        #sb        = message_filters.Subscriber('/dImB', Image)
        
        pl             = rospy.Publisher('/uImL', Image, queue_size=0.00005)    
        pr             = rospy.Publisher('/uImR', Image, queue_size=0.00005)    
        pf             = rospy.Publisher('/uImF', Image, queue_size=0.00005)
        #pb            = rospy.Publisher('//uImB', Image, queue_size=0.04)

        syn  = message_filters.ApproximateTimeSynchronizer([sl, sr, sf], queue_size=1, slop=0.00005) #7 SYNC RETRIEVED DATA      
        #IMAGE MESSAGES SHOULD ARRIVE EACH 0.033 S FOR 30 FPS, THEN THE MAX WAITING RANGE FOR SYNC IS 0.015 (HALF BROADCASTING PERIOD)

        syn.registerCallback(rec)  # DEFINE DATA HANDLER (RECTIFY)

        rospy.spin()    


    except rospy.ROSInterruptException:
        pass
