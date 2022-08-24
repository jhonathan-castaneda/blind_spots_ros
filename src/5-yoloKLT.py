#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
import numpy as np
import cv2 
import argparse
import matplotlib.pyplot as plt
import time
from numba import jit
import argparse
from sensor_msgs.msg import Image
from oni_pkg.msg import kltData
from oni_pkg.srv import yoloSample
#-----------------------------------------------------------------------------------------------------------------------------METHOD [<< 3 >>]
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#--------------------------------------------------------------------------------------------------------------------------CALCULATE CENTROIDS
@jit 
def gc(l):                          #<<< IN: LIST OF FEATURES TO TRACK FOR EACH INDIVIDUAL DETECTION IN THE FRAME 
    a = np.sum(l,axis=0)            #<<< CALCULATE CENTROID FOR THE 
    a = a /len(l)                   #### REF. FEATURES
    return a
#-----------------------------------------------------------------------------------------------------------------------------METHOD [<< 2 >>]
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-----------------------------------------------------------------------------------------------------------------GET MASKS FOR ALL DETECTIONS
"""
#@jit
def gm2(bx,f):                        #<<< IN: BBOXES AND GRAYFRAME ("OLD GRAY FRAME") 
    m = np.zeros_like(f)              #<<< GET THE INDIVIDUAL MASK "m"
    
    for i in range(len(bx)):    
        
        b  = bx[i]
        x  = b[1]                       #<<< BBOX - UP LEFT CORNER (X) 
        y  = b[0]                       #<<< BBOX - UP LEFT CORNER (Y)
        dx = b[3]                       #<<< BBOX - DELTA WIDTH
        dy = b[2]                       #<<< BBOX - DELTA HEIGHT
        for u in range(dx):             #<<< GO THROUGH THE PIXELS OF THE BOUNDING BOX 
            for v in range(dy):         #### FROM X,Y TO X+dx , Y,dy (PIXELS SWEPT)  
                cx=x+u
                cy=y+v
                m[cx,cy] = 255          #<<< ENABLE THE ZONES IN THE INDIVIDUAL MASK GIVEN BY THE BOUNDING BOX             
    return m
"""
#---------------------------------------------------------------------------------------------------------------------------------------------
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------------------------------MASK FOR A SINGLE ROW OF THE DETECTIONS
@jit #
def gm(b,f):                        #<<< IN: BBOXES AND GRAYFRAME ("OLD GRAY FRAME") 
    m = np.zeros_like(f)            #<<< GET THE INDIVIDUAL MASK "m"
    x  = b[1]                       #<<< BBOX - UP LEFT CORNER (X) 
    y  = b[0]                       #<<< BBOX - UP LEFT CORNER (Y)
    dx = b[3]                       #<<< BBOX - DELTA WIDTH
    dy = b[2]                       #<<< BBOX - DELTA HEIGHT
    for u in range(dx):             #<<< GO THROUGH THE PIXELS OF THE BOUNDING BOX 
        for v in range(dy):         #### FROM X,Y TO X+dx , Y,dy (PIXELS SWEPT)  
            cx=x+u
            cy=y+v
            m[cx,cy] = 255          #<<< ENABLE THE ZONES IN THE INDIVIDUAL MASK GIVEN BY THE BOUNDING BOX
    return m
#-----------------------------------------------------------------------------------------------------------------------------METHOD [<< 1 >>]
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------------------------------------------------GET FEATURES TO TRACK
def gf(bx,fg):                       #<<< IN: BOUNDING BOXES AND GRAY FRAME
    
    f = []
    
    l  = len(bx)                     #<<< GET NUMBER OF DETECTIONS
    n  = np.zeros((l),   dtype=np.int16)    #<<< INIT FEATURES COUNTER
    c  = np.zeros((l,2), dtype=np.int16)  #<<< INIT CENTROIDS COUNTER
    
    for i in range(l) :              #<<< FOR EACH BOUNDING BOX::  
        m = gm(bx[i],fg)             #<<< GET MASK FOR THE INDIVIDUAL OBJECT.........................................................[<< 2 >>]

        if i == 0:                   #<<< first iteration            
            f       = cv2.goodFeaturesToTrack(fg, mask = m, **fp) # <<<<< EXTRACT FEATURES TO TRACK
            c[i,:]  = gc(f)          #<<< CALCULATE CENTROID FOR THE i DETECTION.....................................................[<< 3 >>]
            n[i]    = len(f)         #<<< REGISTER # OF FEATURES TO TRACK IN THE i DETECTION
            
        else:
            ff      = cv2.goodFeaturesToTrack(fg, mask = m, **fp) #<<< EXTRACT FEATURES TO TRACK
            f       = np.append(f,ff, axis=0)                     #<<< STACK LOCATION OF THE NEW FEATURES
            c[i,:]  = gc(ff)         #<<< CALCULATE CENTROID WITH THE ACTUAL FEATURES LOC............................................[<< 3 >>]
            n[i]    = len(ff)#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< REGISTER # OF FEATURES TO TRACK
            
    return n, f, c
#--------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| SERIALIZE p0 and BxINT16 TUPLES    
#@jit
def sp0(d,h,o):
    
    i = 0
    for c in range(2):
        for r in range(h):        
            o[i] =d[r,0,c]
            i=i+1    
    return o

#@jit
def sp3(d,o,w,h):
    
    i = 0
    for c in range(w):
        for r in range(h):
            o[i] =d[r,c]
            i=i+1    
    return o
#--------------------------------------------------------------------------------------------------------------------------------------------SERVICE
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| RETRIEVE FRAME TO PERFORM DETECTIONS
def rna(idx):
    rospy.wait_for_service('/rnaSrv')
    try:
        msg = rospy.ServiceProxy('/rnaSrv', yoloSample)
        ans = msg(idx)
        return(ans.im)
    except:
        rospy.loginfo("request for frame to perform detections failed")
#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='calibration data paths')
    parser.add_argument('-w','--weightsPath', type=str, default='/home/sierra/oni_dir/oni_ws/src/oni_pkg/data/rna/yolo.weights', help='rna weights path')
    parser.add_argument('-c','--rnaConfig',  type=str, default='/home/sierra/oni_dir/oni_ws/src/oni_pkg/data/rna/yolo.cfg',     help='rna config path')
    parser.add_argument("--confidenceThreshold",    type=float,  default=0.3,  help="confidence threshold for rna detections")   
    parser.add_argument("--nmSupressionThreshold",  type=float,  default=0.4,  help="non-max-supression for detections")   
    args, unknown = parser.parse_known_args()
    
    #RNA--INITIALIZATION
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------
    ct  =  args.confidenceThreshold   
    nt  =  args.nmSupressionThreshold 
    n   =  cv2.dnn.readNet(args.weightsPath, args.rnaConfig) #<<< READ CNN  
    n.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    n.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        
    m   = cv2.dnn_DetectionModel(n)                  #<<< INIT DETECTION MODEL OBJECT
    m.setInputParams(size=(416, 416), scale=1/255, swapRB=True) 
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------
    fp = dict( maxCorners = 5, qualityLevel = 0.3, minDistance = 7, blockSize = 7 )                                       #<<< ShiTomasi corner detection params.
    lp = dict( winSize  = (15, 15), maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)) #<<< lucas kanade optical flow params
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------
    #MAIN SEQUENCE------------------------------------------------------------------------------------------------------------------------------------------------
    
    try: 
        rospy.init_node('rnaDetector', anonymous = False)                    # <<< -- INIT rna node
        rate   = rospy.Rate(30)                                             # <<< -- SET REFRESH RATE
        rospy.set_param('nodernaDetector', 1)                                # <<< -- ENABLE FLAG FOR THE NODE
        rospy.loginfo('rnaDetectorNodeInitialized')                          # <<< -- REGISTER THE INITIALIZATION OF THE NODE

        pub = rospy.Publisher('/kltData', kltData,  queue_size=0.005)       # <<< -- DEFINE KLT DATA PUBLISHER
        msg = kltData()                                                     # <<< -- INITIALIZE KLT DATA MESSAGE
        bdg = CvBridge()                                                    # <<< -- INITIALIZE CVBRIDGE OBJECT
        
        while True:
                        
            ms   = rna(1)  # <<< -- REQUEST SERVICE, GET NEW IMAGE TO PERFORM DETECTIONSON IT (THE 1 DOES NOTHEING, PENDING TO UPDATE)
            im   = bdg.imgmsg_to_cv2(ms,"bgr8")

            of          =  im                                       #<<< OLD FRAME, TO EXTRACT FEATURES
            og          =  cv2.cvtColor(of, cv2.COLOR_BGR2GRAY)     #<<< OLD FRAME, TO EXTRACT FEATURES IN GRAY SCALE
            cl, sc, bx  =  m.detect(of, ct, nt)                     #<<< CLASSES, SCORES & BOXES IN THE DETECTIONS
            n,p0,oc     =  gf(bx,og)                                #<<< # OF DETECTIONS, LIST_OF_FEATURES_OLD & CENTROIDS_OLD (FOR REFERENCE THE LAST 2)...[<< 1 >>]
            #mk          =  gm2(bx,og)                               #<<< # GET MASK OF ALL DETECTIONS   #########!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!----------------------
            #///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            m1          =  bdg.cv2_to_imgmsg(of, "bgr8")            #<<< PREPARE GRAY FRAME FOR BROADCAST FORMAT
            #m5          =  bdg.cv2_to_imgmsg(mk, "mono8")            #<<< PREPARE DETECTIONS MASK FOR BROADCAST FORMAT  #########!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!----------------------
            
            if bx is not None:
                
                o2 = np.zeros( 2*(len(p0)) , dtype=np.float32)   #<< PREPARE p0 DATA TO BROADCAST; INITIALIZE MSG OBJECT 
                m2 = sp0(p0, len(p0), o2)                      #<< PREPARE p0 DATA TO BROADCAST; STACK X AND Y COLUMNS OF THE CENTROIDS FOR SERIALIZED MESSAGE
                
                m3  = np.append(oc[:,0], oc[:,1], axis = 0 )   #<< PREPARE oc DATA TO BROADCAST; STACK X AND Y COLUMNS OF THE CENTROIDS FOR SERIALIZED MESSAGE
                
                o4 = np.zeros( 4*(len(bx)) , dtype=np.int16)   #<< PREPARE Bx DATA TO BROADCAST; INITIALIZE MSG OBJECT
                m4 = sp3(bx, o4, 4, len(bx))                   #<< PREPARE BX DATA TO BROADCAST; STACK X, Y, W, AND H COLUMNS DATA IN THE SERIALIZED DATA

            else:
                m2,m3,m4 =[],[],[]
            #///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            msg.of   = m1
            msg.p0   = m2
            msg.oc   = m3
            msg.bx   = m4
            msg.cl   = cl
            msg.nbx  = len(bx)
            msg.np0  = len(p0)
            msg.npb  = n        #<<<-- NUMBER OF FEATURES PER BBOX

            #msg.mk   = m5 #########!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!----------------------
            
            pub.publish(msg)
              
    except rospy.ROSInterruptException:
        pass

