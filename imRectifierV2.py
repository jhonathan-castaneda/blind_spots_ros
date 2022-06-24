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
def import_im_calib(calib_data_path):
    
    with open(calib_data_path) as c:  
    
        calib_file  = yaml.load(c, Loader=yaml.FullLoader) 
        
    for i in range (len(cams_ids)):
        
        cam_dic  = calib_file.get( cams_ids[i+1] ) #AT THE 1ST ITERATION 0+1 == 1--> EQUAL TO "left" IN THE CAMS DIC OF ABOVE

        k_list   = cam_dic.get("k_mtx")  #EXTRACT THE CAMERA MATRIX
        d_list   = cam_dic.get("d_mtx")  #EXTRACT THE DISTORTION MATRIX   
        rt_list  = cam_dic.get("rt_mtx") #EXTRACT THE RT MATRIX 
        
        rospy.set_param( cams_ids[i+1]+'_mtx_k',  k_list)
        rospy.set_param( cams_ids[i+1]+'_mtx_d',  d_list)    
        rospy.set_param( cams_ids[i+1]+'_mtx_rt', rt_list)        
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------------------CALLBACK TO RECTIFY IMAGES             
def rectify( dimg_l, dimg_r, dimg_f): #, dimg_b):
    
    l = bridge.imgmsg_to_cv2( dimg_l ,"bgr8")
    r = bridge.imgmsg_to_cv2( dimg_r ,"bgr8")
    f = bridge.imgmsg_to_cv2( dimg_f ,"bgr8")
   #b = bridge.imgmsg_to_cv2( dimg_b ,"bgr8")
       
    #update this to retrieve the dist and k matrix from each camera instead using the same one for all of them
    
    l = cv2.undistort(l, k_mtx, d_mtx, None, new_mtx)
    r = cv2.undistort(r, k_mtx, d_mtx, None, new_mtx)
    f = cv2.undistort(f, k_mtx, d_mtx, None, new_mtx)
   #b = cv2.undistort(b, k_mtx, d_mtx, None, new_mtx)
    
    msg_l = bridge.cv2_to_imgmsg(l, "bgr8")
    msg_r = bridge.cv2_to_imgmsg(r, "bgr8")
    msg_f = bridge.cv2_to_imgmsg(f, "bgr8")
   #msg_b = bridge.cv2_to_imgmsg(b, "bgr8")
    
    pub_l.publish(msg_l)
    pub_r.publish(msg_r)             
    pub_f.publish(msg_f)
   #pub_b.publish(msg_b)

    
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#---------------------------------------------------------------------------------------------------------------------MAIN_SEQUENCE

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='calibration data from cameras')
    
    parser.add_argument('-c','--calib_data_path',  type=str,  default='/home/sierra/oni_dir/oni_ws/src/oni_pkg/yaml_config/cams_data.yaml',  help='img calib data, file path')
        
    args, unknown = parser.parse_known_args()
      
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------IDENTIFIERS OF THE CAMERAS(DICTIONARY)
    cams_ids =  { 1:"left" , 2:"right", 3:"front"}#, 4:"back"}


#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#--------------------------------------------------------------------------------------------------------------------------------------MAIN SEQUENCE
    try:

        bridge     = CvBridge()
        import_im_calib(args.calib_data_path)

        w, h  = 1280 , 720

        k_list = rospy.get_param('left_mtx_k') 
        d_list = rospy.get_param('left_mtx_d') 

        k_mtx  = list2mtx_k(k_list)
        d_mtx  = list2mtx_d(d_list)    

        new_mtx, roi = cv2.getOptimalNewCameraMatrix(k_mtx, d_mtx, (w,h), 0, (w,h))

        #----------------------------------------------------------------------------------------------------------------------------

        rospy.init_node('img_rectifier', anonymous = False)    
        rate   = rospy.Rate(30)    
        rospy.set_param('nodeimRectifier', 1)                              #  -- ENABLE FLAG FOR THE "imBroadcaster" NODE
        rospy.loginfo('imageRectifierNodeInitialized')                     #  -- REGISTER THE INITIALIZATION OF THE NODE

        sub_left          = message_filters.Subscriber('/d_img_l', Image)
        sub_right         = message_filters.Subscriber('/d_img_r', Image)    
        sub_front         = message_filters.Subscriber('/d_img_f', Image)
        #sub_back         = message_filters.Subscriber('/d_img_b', Image)

        pub_l             = rospy.Publisher('/u_img_l', Image, queue_size=0.0005)    
        pub_r             = rospy.Publisher('/u_img_r', Image, queue_size=0.0005)    
        pub_f             = rospy.Publisher('/u_img_f', Image, queue_size=0.0005)
        #pub_b             = rospy.Publisher('/u_img_b', Image, queue_size=0.04)

        sync  = message_filters.ApproximateTimeSynchronizer([sub_left, sub_right, sub_front], queue_size=1, slop=0.0005) #7 SYNC RETRIEVED DATA      
        #IMAGE MESSAGES SHOULD ARRIVE EACH 0.033 S FOR 30 FPS, THEN THE MAX WAITING RANGE FOR SYNC IS 0.015 (HALF BROADCASTING PERIOD)

        sync.registerCallback(rectify)  # DEFINE DATA HANDLER

        rospy.spin()    


    except rospy.ROSInterruptException:
        pass
