#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import message_filters
import argparse
import yaml
import cv2.aruco as aruco
import time as t
#-----------------------------------------------------------------------------------------------------------------------------------------DICTIONARY
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------------------IDENTIFIERS OF THE CAMERAS
c_id = { 1:'l' , 2:'r', 3:'f'}#, 4:"back"}
s_id = { 1: [3, 7, 5, 1] , 2 : [2, 6, 8, 4], 3 : [1, 5, 6, 2]} # SEQUENCES TO ORDER REFERENCE POINTS
#----------------------------------------------------------------------------------------------------------------------------------------------CLASS
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#-------------------------------------------------------------------------------------------------------------DEFINITION OF THE HANDLER CLASS SERVER
class server:

    def __init__(self):#, dimg_f, dimg_b):
        
        self.l  = None 
        self.r  = None
        self.f  = None        
        #self.b = None         
        
        self.l_p  = '' # LEFT REFERENCE POINTS
        self.r_p  = '' # RIGHT REFERENCE POINTS
        self.f_p  = '' # FRONT REFERENCE POINTS
        #self.b_p  = '' # BACK REFERENCE POINTS
        
        self.l_f = 0  # LEFT LOADING FLAG
        self.r_f = 0  # RIGHT LOADING FLAG
        self.f_f = 0  # FRONT LOADING FLAG
        #self.b_f=0   # BACK LOADING FLAG
        
        self.st  = 0   #FLAG TO NOTIFY IF THE PIXELS FROM LANDMARKS ARE ALREADY EXTRACTED AND EXPORTED TO .YAML CALIB FILE
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#---------------------------------------------------------------------------------------------------------------------------------FIND THE LANDMARKS
def detect(im, markerSize=4, totalMarkers = 250, draw=True ):
 
    imG   = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)  
    k     = getattr(aruco,f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    Dict  = aruco.Dictionary_get(k)
    Param = aruco.DetectorParameters_create()
    bbox, ids, rejected = aruco.detectMarkers(imG, Dict, parameters=Param)
    aruco.drawDetectedMarkers(im, bbox)
    
    return(bbox, ids)
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#-------------------------------------------------------------------------------------------------------------------------------LOAD DETECTIONS INFO
def load(cen, id):
    
    sq  = s_id[id]       # 4 --- LOAD THE SCANNING SEQUENCE TO EXTRACT THE COORDS FROM EACH LANDMARK
    x,y = '',''          # 5 --- INITIALIZE EMPTY STRINGS FOR OUTPUT
    for i in range (4):  # 6 --- EXTRACT IN THE SEQUENTIAL ORDER THE LIST OF COORDS. X AND Y AND STORE THEM AT "out_str_x" & "out_str_y"
        
        k = sq[i]        # THIS WILL BE 3, 7, 5, 1 .... 2, 6, 8, 4   ....  1, 5, 6, 2 ... x IS THE KEY TO COMPARE (ELEMENT OF THE SEQUENCE)
        for j in range(4): # SEARCH IN THE 4 BBOXES
                
            if cen[j,2] == k: 

                if i ==0: #if the KEY IS THE FIRST ONE OF THE SEQUENCE
                    x = str(cen[j,0]) # GET THE X COORD 
                    y = str(cen[j,1]) # GET THE Y COORD
                else:
                    x = x + ',' + str(cen[j,0]) #STACK THE X COORD WITH THE ONE OF THE FIRST KEY
                    y = y + ',' + str(cen[j,1]) #STACK THE Y COORD WITH THE ONE OF THE FIRST KEY
            else:
                pass
            
    print("GOOD")        
    setattr(srv, (c_id[id]+'_p') ,{ x +',' + y } ) #LOAD THE REFERENCE POINTS FOR THE CURRENT IMAGE SOURCE IN THE SERVER
    setattr(srv, (c_id[id]+'_f') ,1 )              #SET THE LOADING FLAG FOR THE CURRENT IMAGE SOURCE
    rospy.loginfo('LANDMARKS FROM CHANNEL '+c_id[id]+' ACQUIRED AND STORED')
    #THE SERVER OBJECT MUST BE NAMED "srv"
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#----------------------------------------------------------------------------------------CHECK IMG, TO DETECT LANDMARKS AND EXTRACT REFERENCE POINTS
def check(im, Id): #im is the frame and Id the image source identifier... 1, 2, 3 or 4 for l, r, f, or b camera     
    bbox, ids  =  detect(im)                              # 1 --- SEARCH FOR TARGETS    
    o = bdg.cv2_to_imgmsg(im, "bgr8")                     # 2 --- SET OUTPUT MSG (IMG WITH THE DETECTIONS ON IT)
    pub.publish(o)
    
    try:        
        d = [ids[0,0]  , ids[1,0] , ids[2,0] , ids[3,0]]
        
        if ((len(ids)) == 4) & (sum(s_id[Id]) == sum(d) ): # 4 --- IF ALL THE REFERENCE LANDMARKS ARE AVAILABLE AND DETECTED, CONTINUE            cen = np.zeros((4,3))
                                                           # WE USE THE SUM TO CHECK EVERY SINGLE SEQUENCE CAUSE THE SUME OF THE ELEMENTS OF EACH ONE IS DIFFERENT FROM THE OTHERS
            cen = np.zeros((4,3))
            for i in range(4):          # 4 --- OBTAIN THE CENTROID FOR EACH BOUNDING BOX AND FILL THE CENTROIDS ARRAY (COORDS. AND LANDMARK ID)

                pts =  bbox[i]
                p   =  pts[0]
                x,y =  p[:,0], p[:,1]  # [:,0] FOR FIRST COLUMN OF X VALUES  &  [:,1] FOR SECOND COLUMN OF Y VALUES
                lth = len(x)

                cen[i,0] = round(sum(x)/lth)   # CENTROID COORD X 
                cen[i,1] = round(sum(y)/lth)   # CENTROID COORD Y
                cen[i,2] = ids[i]              # CENTROID ARUCO ID
                
            load(cen, Id) # WE SEND 2 CAUSE 1 IS THE LEFT CAMERA ID           
        else:
            pass   
    except:        
        pass
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#-------------------------------------------------------------------------------------------------------------------------CALLBACK FOR INCOMING DATA    
def cback(l,r,f):
    
    if getattr(srv, 'l_f') == 0: 
        check(bdg.imgmsg_to_cv2(l,"bgr8") , 1)       #DETECT IN LEFT  (Id:1) IMAGE USING THE FUNCTIONS "check" AND "detect" AND ALSO "load"
        
    if getattr(srv, 'l_f') == 1:                     #IF L IS READY GO TO R DETECTION...
        if getattr(srv, 'r_f') == 0:
        
            check(bdg.imgmsg_to_cv2(r,"bgr8") , 2) #DETECT IN RIGHT (Id:2) IMAGE USING THE FUNCTIONS "check" AND "detect" AND ALSO "load")
    
    if getattr(srv, 'r_f') == 1:# & getattr(srv, 'f_f') == 0: #IF R IS READY GO TO F DETECTION...
        if getattr(srv, 'f_f') == 0:
            check(bdg.imgmsg_to_cv2(f,"bgr8") , 3) #DETECT IN RIGHT (Id:2) IMAGE USING THE FUNCTIONS "check" AND "detect" AND ALSO "load")
            
            #setattr(srv,'st' , 1 ) # IN THE LAST ITERATION, EXPORT THE REFERENCE POINTS TO .YAML FILE
            
    #if getattr(srv, 'st') == 1
        #exp()
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#--------------------------------------------------------------------------------------------------------------------------------------MAIN SEQUENCE
if __name__ == "__main__":
    
    try:

        bdg  = CvBridge()
        srv  = server()   #INITIALIZE SERVER
        #----------------------------------------------------------------------------------------------------------------------------

        rospy.init_node('imAutocalibDetector', anonymous = False)    
        rate   = rospy.Rate(30)    
        rospy.set_param('nodeimAutoCalibDetector', 1)                              #  -- ENABLE FLAG FOR THE "imAutoCalibDetector" NODE
        rospy.loginfo('imAutoCalibDetectorNodeInitialized')                        #  -- REGISTER THE INITIALIZATION OF THE NODE

        sl  =  message_filters.Subscriber('/u_img_l', Image) # LEFT  SUBSCRIBER
        sr  =  message_filters.Subscriber('/u_img_r', Image) # RIGHT SUBSCRIBER
        sf  =  message_filters.Subscriber('/u_img_f', Image) # FRONT SUBSCRIBER
        #sb  =  message_filters.Subscriber('/u_img_b', Image) # BACK SUBSCRIBER

        
        pub =  rospy.Publisher('/arucoIm', Image, queue_size=0.005) 
        
        syn = message_filters.ApproximateTimeSynchronizer([sl,sr, sf], queue_size=1, slop=0.0005) #7 SYNC RETRIEVED DATA      
        #IMAGE MESSAGES SHOULD ARRIVE EACH 0.033 S FOR 30 FPS, THEN THE MAX WAITING RANGE FOR SYNC IS 0.015 (HALF BROADCASTING PERIOD)

        syn.registerCallback(cback)   # DEFINE DATA HANDLER
        rospy.spin()    


    except rospy.ROSInterruptException:
        pass
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||      