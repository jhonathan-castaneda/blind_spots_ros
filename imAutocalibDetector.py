#!/usr/bin/env python3


# THIS WILL DETECT THE LANDMARKS AND REGISTER THE LOCATION FOR THEM 

# THEN STORE THE REFERENCE POINTS WITH THE KNOWN .YAML FORMAT 

##|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

#calibration_units: pixels 

#left:

#  in_img: [95, 319, 42, 398,   20, 23, 86, 105]                                  #FORMAT: x1 x2 x3 x4; y1 y2 y3 y4
#  vir_img: [55.5, 83.33, 388.05, 415.83,   916.66, 55.5, 704.88, 267.33   ]  #FORMAT: x1 x2 x3 x4; y1 y2 y3 y4

#right:

#  in_img: [95, 319, 43, 397,   20, 23, 86, 104]                                  #FORMAT: x1 x2 x3 x4; y1 y2 y3 y4
#  vir_img: [944.44, 916.66, 616.94, 584.17,   83.33, 944.44, 295.11, 732.667   ]  #FORMAT: x1 x2 x3 x4; y1 y2 y3 y4 

#front:

#  in_img: [60, 359, 114, 335,   30, 34, 89, 115]                                  #FORMAT: x1 x2 x3 x4; y1 y2 y3 y4
#  vir_img: [83.3, 944.4, 415.83,611.94,   55.55, 83.3, 267.33, 295,05  ]  #FORMAT: x1 x2 x3 x4; y1 y2 y3 y4

#back:

#  in_img: [50, 370, 92, 373,   23, 28, 78, 110]                                  #FORMAT: x1 x2 x3 x4; y1 y2 y3 y4
#  vir_img: [916.66, 55.5, 584.17, 388.05,   944.44, 916.66, 732.667, 704.88  ]  #FORMAT: x1 x2 x3 x4; y1 y2 y3 y4

##|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
##|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import message_filters
import argparse
import yaml
import cv2.aruco as aruco
#-----------------------------------------------------------------------------------------------------------------------------------------DICTIONARY
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------------------IDENTIFIERS OF THE CAMERAS
cams_ids =  { 1:"left" , 2:"right", 3:"front"}#, 4:"back"}
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#---------------------------------------------------------------------------------------------------------------------------------FIND THE LANDMARKS
def findArucoMarkers(img, markerSize=4, totalMarkers = 250, draw=True ):
 
    imGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  
    key    = getattr(aruco,f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict  = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()

    bbox, ids, rejected = aruco.detectMarkers(imGray, arucoDict, parameters=arucoParam)
   
    #if draw:  
    #    aruco.drawDetectedMarkers(img, bbox)
    return(bbox, ids)
    
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD             
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------------------------------------CLASS TO STORE THE UNDISTORTED IMG MSGS  
class aruco_server:

    def __init__(self):#, dimg_f, dimg_b):
        
        self.left   = None
        self.right  = None
        self.front  = None        
        #self.back = None#dimg_b         
        
        self.points_left   = ''
        self.points_right  = ''
        self.points_front  = ''
        #self.points_back   = ''
        
        self.flag_l = 0
        self.flag_r = 0
        self.flag_f = 0
        
        self.ext_status = 0   #FLAG TO NOTIFY IF THE PIXELS FROM LANDMARKS ARE ALREADY EXTRACTED AND EXPORTED TO .YAML CALIB FILE
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#---------------------------------------------------------------------------EXTRACT THE CENTROIDS OF THE DETECTIONS AND ORGANIZE THEM TO BE EXPORTED         
def extract_ref(frame, target):
#frame      = cv2.imread('/home/sierra/oni_dir/oni_ws/src/oni_pkg/nodes/l3l7l5l1.png')
#target    = 1 #or left camera according to the dict

#                       ID P1 P2 P3 P4
    id_targets = np.array([[3, 7, 5, 1],    #  CAM ID(1=RIGHT) , [P1, P2, P3, P4]     ---> P ARE THE POINTS TO BE LOCATED FROM LEFT TO RIGHT IN THE IMAGE
                           [2, 6, 8, 4],    #  CAM ID(2=RIGHT) , [P1, P2, P3, P4]     ---> P ARE THE POINTS TO BE LOCATED FROM LEFT TO RIGHT IN THE IMAGE
                           [1, 5, 6, 2]])   #  CAM ID(3=RIGHT) , [P1, P2, P3, P4]     ---> P ARE THE POINTS TO BE LOCATED FROM LEFT TO RIGHT IN THE IMAGE

    out_str_x   =''
    out_str_y   =''
    centroids   = np.zeros((4,3)) 
    #-----------------------------------------------------------------------------------------------------------------------------------------------------
    bbox, ids  =  findArucoMarkers(frame)  # 1 --- SEARCH FOR TARGETS 

    if (len(ids)) == 4:                    # 2 --- IF ALL THE REFERENCE LANDMARKS ARE DETECTED (AND 4 ID ITEMS ARE AVAILABLE)

        for i in range (len(ids)):         # 3 --- OBTAIN THE CENTROID FOR EACH BOUNDING BOX AND FILL THE CENTROIDS ARRAY (COORDS. AND LANDMARK ID)

            points =  bbox[i]
            p      =  points[0]

            x,y  = p[:,0], p[:,1]  # [:,0] FOR FIRST COLUMN OF X VALUES  &  [:,1] FOR SECOND COLUMN OF Y VALUES

            len_p = len(x)

            centroids[i,0] =  round(sum(x)/len_p)   # CENTROID COORD X 
            centroids[i,1] =  round(sum(y)/len_p)   # CENTROID COORD Y
            centroids[i,2] =  ids[i]                # CENTROID ARUCO ID


        sequence = id_targets[target-1,:]  # 4 --- LOAD THE SCANNING SEQUENCE TO EXTRACT THE COORDS FROM EACH LANDMARK


        for i in range ( len(sequence) ):  # 5 --- EXTRACT IN THE SEQUENTIAL ORDER THE LIST OF COORDS. X AND Y AND STORE THEM AT "out_str_x" & "out_str_y"

        # ITERATE ON THE SEQUENCE, (CHANGE ARUCO TARGET)... 3, 7, 5, 1 ...

            key = sequence[i]                        # THIS WILL BE 3, 7, 5, 1 .... 2, 6, 8, 4   ....  1, 5, 6, 2 

            for j in range ( len(sequence) ):    # SEARCH IN THE BBOXES

                if centroids[j,2] == key: 

                    if i ==0: #if the KEY IS THE FIRST ONE OF THE SEQUENCE

                        out_str_x   = str(centroids[j,0]) # GET THE X COORD 
                        out_str_y   = str(centroids[j,1]) # GET THE Y COORD

                    else:
                        out_str_x   = out_str_x + ',' + str(centroids[j,0]) #STACK THE X COORD WITH THE ONE OF THE FIRST KEY
                        out_str_y   = out_str_y + ',' + str(centroids[j,1]) #STACK THE Y COORD WITH THE ONE OF THE FIRST KEY
                else:
                    pass
        
        flag = 1
        #print(out_str_x)  #ONLY4TEST-----------__!!!!!!!!!!!!
        #print(out_str_y)  #ONLY4TEST-----------__!!!!!!!!!!!!
    
    
    else:
        flag = 0
        pass
    
    return(bbox, ids, out_str_x, out_str_y, flag)  # RETURN ORDERED DATA x1 x2 x3 x4; y1 y2 y3 y4 

    #THE PAIR OF OUT STRINGS WILL BE ASSIGNED TO THE ATTRIBUTES OF THE OBJECT ARUCO_SERVER
    
    #THEN THE STRINGS WILL BE USED TO EXPORT THE REFERENCE POINS FILE TO THE .YAML FILE USED IN THE EXTRINSIC CALIB NODE     
    
    
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#----------------------------------------------------------------------------------------BROADCAST THE CURRENT DETECTIONS-- FOR VISUAL FEEDBACK ONLY     
def broadcast_aruco(l,r,f):    
    
    bbox, ids = findArucoMarkers(l)
    pub_l.publish(bridge.cv2_to_imgmsg(aruco.drawDetectedMarkers(l, bbox), "bgr8"))
    
    bbox, ids = findArucoMarkers(r)
    pub_r.publish(bridge.cv2_to_imgmsg(aruco.drawDetectedMarkers(r, bbox), "bgr8"))

    bbox, ids = findArucoMarkers(f)
    pub_f.publish(bridge.cv2_to_imgmsg(aruco.drawDetectedMarkers(f, bbox), "bgr8"))

#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#--------------------------------------------------------------------------------------------------CALLBACK TO EXTRACT THE LOCATION OF THE LANDMARKS             
def extract_landmarks(uimg_l, uimg_r, uimg_f): #, uimg_b):
    
    l = bridge.imgmsg_to_cv2( uimg_l ,"bgr8")
    r = bridge.imgmsg_to_cv2( uimg_r ,"bgr8")
    f = bridge.imgmsg_to_cv2( uimg_f ,"bgr8")    
    
    flag_l = getattr(server, 'flag_l') 
    flag_r = getattr(server, 'flag_r)
    flag_f = getattr(server, 'flag_f')
    
    broadcast_aruco(l,r,f)
    
    if (getattr(server, 'ext_status') ) == 0:
        

        
        while flag_l == 0 && flag_r == 0 && flag_f ==0:
#-----------------------------------------------EXTRACT REFERENCE POINTS FROM LEFT IMAGE IF THEY ARE NOT READY--------------------------------------            
            if flag_l == 0:
            
                id_target =  1
                 
                bbox, ids, out_str_x, out_str_y, ext_flag = extract_ref(l, target)

                if ext_flag == 1:

                    setattr(server,  'points_left'  , {out_str_x + out_str_y}  )

                    im_ready =  aruco.drawDetectedMarkers(frame, bbox)

                    setattr(server,  'left'  ,  bridge.cv2_to_imgmsg(im_ready, "bgr8")  )
                    
                    rospy.loginfo('LANDMARKS FROM LEFT CHANNEL ACQUIRED AND STORED')
                    setattr(server, 'flag_l', 1 )
                    

                else:
                    pass                
 #-----------------------------------------------EXTRACT REFERENCE POINTS FROM LEFT IMAGE IF THEY ARE NOT READY-------------------------------------        
            if flag_r ==0:
            
                id_target =  2

                bbox, ids, out_str_x, out_str_y, ext_flag = extract_ref(r, target)

                if ext_flag == 1:

                    setattr(server,  'points_right'  , {out_str_x + out_str_y}  )

                    im_ready =  aruco.drawDetectedMarkers(frame, bbox)

                    setattr(server,  'right'  ,  bridge.cv2_to_imgmsg(im_ready, "bgr8")  )
                    
                    rospy.loginfo('LANDMARKS FROM RIGHT CHANNEL ACQUIRED AND STORED')
                    setattr(server, 'flag_r', 1 )

                else:
                    pass    
                
            
#------------------------------------------------EXTRACT REFERENCE POINTS FROM FRONT IMAGE IF THEY ARE NOT READY------------------------------------  
            if flag_f ==0:
            
                id_target =  3

                bbox, ids, out_str_x, out_str_y, ext_flag = extract_ref(r, target)

                if ext_flag == 1:

                    setattr(server,  'points_right'  , {out_str_x + out_str_y}  )

                    im_ready =  aruco.drawDetectedMarkers(frame, bbox)

                    setattr(server,  'right'  ,  bridge.cv2_to_imgmsg(im_ready, "bgr8")  )

                    rospy.loginfo('LANDMARKS FROM FRONT CHANNEL ACQUIRED AND STORED')
                    setattr(server, 'flag_f', 1 )

                else:
                    pass 
#-----------------------------------------------------------------BROADCAST CURRENT DETECTIONS------------------------------------------------------          
        #HERE, IF ALL THE REF POINTS ARE EXTRACTED THE STATUS OF THE ARUCO SERVER BECOMES 1     
        setattr(server,  'ext_status'  , 1 )
        #HERE, CALL FOR METHOD TO EXPORT IN .YAML FILE
        rospy.loginfo('Landmarks extracted and stored at: /home/sierra/oni_dir/oni_ws/src/oni_pkg/yaml_config/ext_cams_calib.yaml')
        #rospy.set_param('nodeimAutoCalibDetector', 0)
 
    else:
        pass
    
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#--------------------------------------------------------------------------------------------------------------------------------------MAIN SEQUENCE
if __name__ == "__main__":
    
    try:

        bridge  = CvBridge()
        server  = aruco_server()   #INITIALIZE SERVER
        #----------------------------------------------------------------------------------------------------------------------------

        rospy.init_node('img_AutocalibDetector', anonymous = False)    
        rate   = rospy.Rate(30)    
        rospy.set_param('nodeimAutoCalibDetector', 1)                              #  -- ENABLE FLAG FOR THE "imAutoCalibDetector" NODE
        rospy.loginfo('imAutoCalibDetectorNodeInitialized')                        #  -- REGISTER THE INITIALIZATION OF THE NODE

        sub_left          = message_filters.Subscriber('/u_img_l', Image)
        sub_right         = message_filters.Subscriber('/u_img_r', Image)    
        sub_front         = message_filters.Subscriber('/u_img_f', Image)
        #sub_back         = message_filters.Subscriber('/u_img_b', Image)

        pub_l               = rospy.Publisher('/aruco_img_l', Image, queue_size=0.0005) 
        pub_r               = rospy.Publisher('/aruco_img_r', Image, queue_size=0.0005)
        pub_f               = rospy.Publisher('/aruco_img_f', Image, queue_size=0.0005)
        
        sync  = message_filters.ApproximateTimeSynchronizer([sub_left, sub_right, sub_front], queue_size=1, slop=0.0005) #7 SYNC RETRIEVED DATA      
        #IMAGE MESSAGES SHOULD ARRIVE EACH 0.033 S FOR 30 FPS, THEN THE MAX WAITING RANGE FOR SYNC IS 0.015 (HALF BROADCASTING PERIOD)

        sync.registerCallback(extract_landmarks)  # DEFINE DATA HANDLER

        rospy.spin()    


    except rospy.ROSInterruptException:
        pass
