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
cid =  { 1:"l" , 2:"r", 3:"f", 4:"b"}
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
def rec(ms): #, dimg_b):
    
    im   = bdg.imgmsg_to_cv2(ms,"bgra8")
    
    l, r, f, b = cv2.split(im)
    
    #l = cv2.merge((l,l,l)) # JUST TO RUN UNDISTORT THEN... WE JUST USE ONE CHANNEL 
    #r = cv2.merge((r,r,r)) # JUST TO RUN UNDISTORT THEN... WE JUST USE ONE CHANNEL
    #f = cv2.merge((f,f,f)) # JUST TO RUN UNDISTORT THEN... WE JUST USE ONE CHANNEL
        
    #l = cv2.undistort(l, km, dm, None, nmt)
    #r = cv2.undistort(r, km, dm, None, nmt)
    #f = cv2.undistort(f, km, dm, None, nmt)
    
    dl          =  cv2.remap(l, mxl, myl, cv2.INTER_LINEAR)
    x, y, w, h  =  roil
    l2          =  dl[y:y+h, x:x+w]
    
    dr          =  cv2.remap(r, mxr, myr, cv2.INTER_LINEAR)
    x, y, w, h  =  roir
    r2          =  dr[y:y+h, x:x+w]

    
    f = cv2.resize(f, (720,480), interpolation= cv2.INTER_LINEAR)  # RESIZE FRONT FRAME ONLY FOR THE SONY CAM
    df          =  cv2.remap(f, mxf, myf, cv2.INTER_LINEAR)
    x, y, w, h  =  roif
    f2          =  df[y:y+h, x:x+w]
    f2 = cv2.resize(f2, (1279,719), interpolation= cv2.INTER_LINEAR)  # RESIZE FRONT FRAME ONLY FOR THE SONY CAM

    b = cv2.resize(b, (720,480), interpolation= cv2.INTER_LINEAR)  # RESIZE BACK FRAME ONLY FOR THE SONY CAM
    db          =  cv2.remap(b, mxb, myb, cv2.INTER_LINEAR)
    x, y, w, h  =  roib
    b2          =  db[y:y+h, x:x+w]
    b2 = cv2.resize(b2, (1279,719), interpolation= cv2.INTER_LINEAR)  # RESIZE BACK FRAME ONLY FOR THE SONY CAM

    #print(f2.shape)
    #bb = cv2.undistort(b, k_mtx, d_mtx, None, nmt)
    #o = cv2.merge(( l[:,:,0], r[:,:,0], f[:,:,0]))
    
    o = cv2.merge(( l2, r2, f2, b2))
    
    m = bdg.cv2_to_imgmsg(o, "bgra8")
    
    p.publish(m)
    
    
    #l2= bdg.cv2_to_imgmsg(l2, "mono8") 
    #r2= bdg.cv2_to_imgmsg(r2, "mono8") 
    #f2= bdg.cv2_to_imgmsg(f2, "mono8") 
    #b2= bdg.cv2_to_imgmsg(b2, "mono8")
    #PUT THIS IN A SERVICE THEN
    #pl.publish(l2)
    #pr.publish(r2)
    #pf.publish(f2)
    #pb.publish(b2)    
    
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#---------------------------------------------------------------------------------------------------------------------MAIN_SEQUENCE

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='calibration data from cameras')
    
    parser.add_argument('-c','--calibDataPath',  type=str,  default='/home/sierra/oni_dir/oni_ws/src/oni_pkg/yaml_config/intrinsicData.yaml',  help='iintrinsic calib data, file path')
        
    args, unknown = parser.parse_known_args()
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#--------------------------------------------------------------------------------------------------------------------------------------MAIN SEQUENCE
    try:
        
        bdg   = CvBridge()
        imp(args.calibDataPath)                     #IMPORT INTRINSIC CAMERA PARAMETERS
                                                    #PENDING TO IMPORT THE 4 MTX FOR THE 4 CAMERAS
        ##################################################################################################### LEFT CHANNEL
        kl = rospy.get_param('lMtxK') # k matrix in list
        dl = rospy.get_param('lMtxD') #  distortion matrix in list

        kml  = list2mtx_k(kl)  
        dml  = list2mtx_d(dl)  

        nmtl, roil = cv2.getOptimalNewCameraMatrix(kml, dml, (1280 , 720), 0, (1280 , 720)) # NEW CAMERA MATRIX
        mxl, myl   = cv2.initUndistortRectifyMap(kml, dml, None, nmtl, (1280 , 720), 5)
        #####################################################################################################
        ##################################################################################################### RIGHT CHANNEL
        kr = rospy.get_param('rMtxK') # k matrix in list
        dr = rospy.get_param('rMtxD') #  distortion matrix in list

        kmr  = list2mtx_k(kr)  
        dmr  = list2mtx_d(dr)  

        nmtr, roir = cv2.getOptimalNewCameraMatrix(kmr, dmr, (1280 , 720), 0, (1280 , 720)) # NEW CAMERA MATRIX
        mxr, myr   = cv2.initUndistortRectifyMap(kmr, dmr, None, nmtr, (1280 , 720), 5)
        #####################################################################################################
        ##################################################################################################### FRONT CHANNEL
        kf = rospy.get_param('fMtxK') # k matrix in list
        df = rospy.get_param('fMtxD') #  distortion matrix in list

        kmf  = list2mtx_k(kf)  
        dmf  = list2mtx_d(df)  

        nmtf, roif = cv2.getOptimalNewCameraMatrix(kmf, dmf, (720 , 480), 0, (720 , 480)) # NEW CAMERA MATRIX
        mxf, myf   = cv2.initUndistortRectifyMap(kmf, dmf, None, nmtf, (720 , 480), 5)
        #####################################################################################################
        ##################################################################################################### BACK CHANNEL
        kb = rospy.get_param('bMtxK') # k matrix in list
        db = rospy.get_param('bMtxD') #  distortion matrix in list

        kmb  = list2mtx_k(kb)  
        dmb  = list2mtx_d(db)  

        nmtb, roib = cv2.getOptimalNewCameraMatrix(kmb, dmb, (720 , 480), 0, (720 , 480)) # NEW CAMERA MATRIX
        mxb, myb   = cv2.initUndistortRectifyMap(kmb, dmb, None, nmtb, (720 , 480), 5)
        #####################################################################################################
        
        #----------------------------------------------------------------------------------------------------------------------------
        rospy.init_node('imRectifier', anonymous = False)    
        rate   = rospy.Rate(30)    
        rospy.set_param('nodeimRectifier', 1)                              #  -- ENABLE FLAG FOR THE "imBroadcaster" NODE
        rospy.loginfo('imRectifierNodeInitialized')                        #  -- REGISTER THE INITIALIZATION OF THE NODE
        
        s              = rospy.Subscriber('/dIm', Image, rec)                 # (FUZZY CORRECTION SUBSCRIBER)   
        p              = rospy.Publisher('/uIm',  Image, queue_size=0.00005)
        
        #PASS THIS BELOW TO A SERVICE
        pl              = rospy.Publisher('/uImL',  Image, queue_size=0.00005)
        pr              = rospy.Publisher('/uImR',  Image, queue_size=0.00005)
        pf              = rospy.Publisher('/uImF',  Image, queue_size=0.00005)
        pb              = rospy.Publisher('/uImB',  Image, queue_size=0.00005)

        
        rospy.spin()    


    except rospy.ROSInterruptException:
        pass
