#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
import numpy as np
import cv2 
import argparse
import matplotlib.pyplot as plt
import time
from numba import jit
from sensor_msgs.msg import Image

from oni_pkg.srv import yoloSample
from oni_pkg.msg import kltData

#from std_msgs.msg import Int16MultiArray

ci  =  {0:'C', 1:'M', 2:'P'}                              #<<< CLASSES DICTIONARY
cc  =  {0:(204,102,155), 1:(0,102,153), 2:(102,255,102)}  #<<< COLOR4CLASSES DICTIONARY  
#----------------------------------------------------------------------------------------------------------------------------------------CLASS
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#------------------------------------------------------------------------------------------------------------------------------TRACKING SERVER
class server:

    def __init__(self):
        
        #TRACKING DATA---------------------------------------------------------------------
        self.og  = None #OLD GRAY FRAME                      <<<IMAGE>>>
        self.mk  = None #TRACKING MASK (is made from og)         
        self.p0  = None #OLD LIST OF FEATURES                <<<np.ndarray>>> int16   (26x 2)            
        self.oc  = None #OLD LIST OF FEATURES CENTROIDS      <<<np.ndarray>>> int16   (n x 2)
        self.nc  = None #NEW LIST OF CENTROIDS
        self.bx  = None #BOUNDING BOXES FROM DETECTIONS      <<<np.ndarray>>> int16   (n x 4)
        
        #self.bx2 = None #INERTIA bx                             #!!!!!!!!!!!!!!!!!!!!!!!
        #self.oc2 = None #INERTIA oc                             #!!!!!!!!!!!!!!!!!!!!!!!
        #self.nn  = 0                                            #!!!!!!!!!!!!!!!!!!!!!!!
        #self.mk0 = None #REFERENCE MASK OF DETECTIONS           #!!!!!!!!!!!!!!!!!!!!!!!
        self.po  = None #NEW LIST OF FEATURES FROM FLOW ESTIM   #!!!!!!!!!!!!!!!!!!!!!!!

        self.cl  = None #DETECTED CLASSES IN BBOXES          <<<np.ndarray>>> int8    (1 x n)
        self.npb = None #NUMBER OF P0 FEATURES PER BBOX      <<<np.ndarray>>> int8    (1 x n)
        self.nbx = 0  #OLD NUMBER OF DETECTIONS        
#-----------------------------------------------------------------------------------------------------------------------------METHOD [<< 6 >>]
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#---------------------------------------------------------------------------------------------------------------------------DRAW TRACKING PATH 
#def pts(co,cn,cf,msk):             #<<< IN: OLD_CENTROIDS, NEW_CENTROIDS, CURRENT_BGR_FRAME && MASK_FOR_TRACKING,  
def pts(co, cn, nf, mk, cl, bx):
    
    for i in range(len(cn)):
        z   = cc[cl[i]]                        #<<< ASSIGN COLOR FOR EACH BBOX, ACCORDING TO THE CLASS DETECTED  
        nf  = cv2.rectangle(nf, bx[i], z, 2)  #<<< DRAW BBOXES
        nf  = cv2.circle(nf, (int(cn[i,0]), int(cn[i,1])), 3, (0,0,255), -1)                          #<<< DRAW FEATURES CENTROIDS
        mk  = cv2.line(mk, (int(cn[i,0]), int(cn[i,1])), (int(co[i,0]), int(co[i,1])), (0,0,225), 2)  #<<< DRAW TRACKING PATH OF THE CENTROIDS
        nf  = cv2.add(nf, mk)
    return nf
#-----------------------------------------------------------------------------------------------------------------------------METHOD [<< 5 >>]
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------FIND DELTA OF BBOXES FOR NEXT ITERATION - UPDATE BOUNDING BOXES
@jit
def ub(bx, co, cn):

    for i in range(len(bx)):

        bx[i,0] = cn[i,0] - (co[i,0] - bx[i,0])
        bx[i,1] = cn[i,1] - (co[i,1] - bx[i,1])      
#-----------------------------------------------------------------------------------------------------------------------------METHOD [<< 5 >>]
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------FIND DELTA OF BBOXES FOR NEXT ITERATION - UPDATE BOUNDING BOXES
#@jit
def ub2(bx, bx2, co, cn):
    
    for i in range(len(bx)):                      # <<< BX UPDATED

        bx[i,0] = cn[i,0] - (co[i,0] - bx[i,0])
        bx[i,1] = cn[i,1] - (co[i,1] - bx[i,1])

    bx  = (bx + bx2)/2                             # <<< BX AVERAGED
    bx2 = bx
        # <<< FOR THE NEXT ITERATION BX2 OF INERTIA IS THE PREVIOUS AVERAGE
#-----------------------------------------------------------------------------------------------------------------------------METHOD [<< 4 >>]
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#----------------------------------------------------------------------------------------------RETRIEVE AND ORDER ODT DATA FOR FLOW ESTIMATION
#@jit
def rd(p0,n,ix):#<<<<<---------------RETRIEVE DATA OF THE FEATURES AND THEIR CORRESPONDING BBOXES

    if ix == 0:    #<<< FOR THE FIRST ITERATION DO NOT COMPARE WITH PREVIOUS SEQUENCES ALONG n 
        st = 0
        d  = n[ix] #<<<<<<<<< NUMBER OF XY COORDS OF FEATURES FOR EACH DETECTION
        
    else:
        st  = np.sum(n[:ix])
        d   = n[ix] + st #<<<<<<<<< NUMBER OF XY COORDS OF FEATURES FOR EACH DETECTION
        
    p = p0[st:d, :] # EXTRACT THE P0 POINTS FOR THE CURRENT n VALUE
    return p    
#-----------------------------------------------------------------------------------------------------------------------------METHOD [<< 3 >>]
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#--------------------------------------------------------------------------------------------------------------------------CALCULATE CENTROIDS
@jit 
def gc(l):                          #<<< IN: LIST OF FEATURES TO TRACK FOR EACH INDIVIDUAL DETECTION IN THE FRAME 
    a = np.sum(l,axis=0)            #<<< CALCULATE CENTROID FOR THE 
    a = a /len(l)                   #### REF. FEATURES
    return a
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||DECODE bx DATA
def ds4(b4,n4):
    
    o4 = np.zeros(( n4 , 4))
    i4 = 0
    for c in range(4):
        for r in range(n4):
            
            o4[r,c] = b4[i4]
            
            i4+=1
    return o4
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||DECODE oc DATA
def ds3(c3,n3):
    
    o3 = np.zeros(( n3 , 2))
    i3 = 0
    for c in range(2):
        for r in range(n3):
            
            o3[r,c] = c3[i3]
            
            i3+=1
    return o3
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||DECODE P0 DATA
def ds2(p2,n2):
    
    o2 = np.zeros(( n2 , 2), dtype=np.float32)
    i2 = 0
    for c in range(2):
        for r in range(n2):
            
            o2[r,c] = p2[i2]
            
            i2+=1

    return o2
#----------------------------------------------------------------------------------------------------------------------------------------VOID METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| DESERIALIZE AND UPDATE TRACKING REFERENCE DATA
def dec(msg):
    
    #print(getattr(srv, 'nn' ))                                                 #!!!!!!!!!!!!!!!!!!!!!!!-------------------
    #setattr(srv ,'nn' , 0 )                                                    #!!!!!!!!!!!!!!!!!!!!!!!-------------------
    #mk0  =  bdg.imgmsg_to_cv2(msg.mk, "mono8"); setattr(srv ,'mk0' , mk0  )    #!!!!!!!!!!!!!!!!!!!!!!!-------------------

    setattr(srv ,'nbx' , msg.nbx ) # <<< -- EXTRACT AND UPDATE DIRECTLY THE NUMBER OF DETECTIONS IN THE OLD GRAY FRAME
    setattr(srv ,'npb' , msg.npb ) # <<< -- EXTRACT AND UPDATE DIRECTLY THE NUMBER OF KLT P0 FEATURES EXTRACTED PER DETECTION (ARRAY OF NUMBER OF FEATURES)
    setattr(srv ,'cl'  , msg.cl  ) # <<< -- EXTRACT AND UPDATE DIRECTLY THE CLASSES DETECTED IN THE OLD GRAY FRAME
     
    of  =  bdg.imgmsg_to_cv2(msg.of, "bgr8")
    og  =  cv2.cvtColor(of, cv2.COLOR_BGR2GRAY);        setattr(srv ,'og' , og )  #<<< UPDATE REFERENCE FRAME FOR TRACKING
    mk  =  np.zeros_like(of);                           setattr(srv ,'mk' , mk  )  #<<< UPDATE MASK FOR DRAWING OF TRACKING PATH    
    p0  =  ds2(msg.p0, msg.np0);   setattr(srv ,'p0' , p0 ) #<<< -- DESERIALIZE AND UPDATE P0 
    oc  =  ds3(msg.oc, msg.nbx);   setattr(srv ,'oc' , oc )#; setattr(srv ,'oc2' , oc ) #<<< -- DESERIALIZE AND UPDATE oc
    nc  =  oc.copy();              setattr(srv ,'nc' , nc ) #<<< -- NEW CENTROIDS ARRAY RE INITIALIZATION 
    bx  =  ds4(msg.bx, msg.nbx);   setattr(srv ,'bx' , bx )#; setattr(srv ,'bx2' , bx ) #<<< -- DESERIALIZE AND UPDATE Bx
#--------------------------------------------------------------------------------------------------------------------------------------------SERVICE
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| RETRIEVE FRAME TO PERFORM DETECTIONS
def klt(idx):
    rospy.wait_for_service('/rnaSrv')
    try:
        msg = rospy.ServiceProxy('/rnaSrv', yoloSample)
        ans = msg(idx)
        return(ans.im)
    except:
        rospy.loginfo("request for frame to perform tracking failed")    
#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
if __name__ == "__main__":
      
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------
    fp = dict( maxCorners = 5, qualityLevel = 0.3, minDistance = 7, blockSize = 7 )                                       #<<< ShiTomasi corner detection params.
    lp = dict( winSize  = (15, 15), maxLevel = 2, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)) #<<< lucas kanade optical flow params
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------
    #MAIN SEQUENCE------------------------------------------------------------------------------------------------------------------------------------------------
    try:
        
        rospy.init_node('kltTracker', anonymous = False)                 # <<< -- INIT KLT TRACKER NODE
        rate   = rospy.Rate(30)                                          # <<< -- SET REFRESH RATE
        rospy.set_param('nodekltTracker', 1)                             # <<< -- ENABLE FLAG FOR THE NODE
        rospy.loginfo('kltTrackerNodeInitialized')                       # <<< -- REGISTER THE INITIALIZATION OF THE NODE

        pub   =  rospy.Publisher('/klt',  Image, queue_size=0.5)         # <<< -- DETECTIONS PUBLISHER
        s1    =  rospy.Subscriber('/kltData', kltData, dec)              # <<< -- KLT DATA SUBSCRIBER              
        bdg   =  CvBridge()                                              # <<< -- INITIALIZE CVBRIDGE OBJECT
        srv   =  server()                                                # <<< -- INITIALIZE PARAMETER SERVER
        
        while True:
            
            #mk0  = getattr(srv, 'mk0' )                   #!!!!!!!!!!!!!!!!!!!!!!!-----------------------------------------
            
            ms   =  klt(1)                                # <<< -- REQUEST SERVICE, GET NEW IMAGE TO PERFORM DETECTIONSON IT (THE 1 DOES NOTHEING, PENDING TO UPDATE)
            nf   =  bdg.imgmsg_to_cv2(ms,"bgr8")          # <<< -- NEW FRAME, TO EXTRACT FEATURES                                 
            ng   =  cv2.cvtColor(nf, cv2.COLOR_BGR2GRAY)  # <<< -- NEW FRAME IN GRAY SCALE  
            #ng  = cv2.bitwise_and(og,og,mask = m222)      #!!!!!!!!!!!!!!!!!!!!!!!----------------------------------------
           
            if getattr(srv, 'p0') is not None:

                og  = getattr(srv, 'og' )
                mk  = getattr(srv, 'mk' )     
                p0  = getattr(srv, 'p0' )        
                oc  = getattr(srv, 'oc' )
                nc  = getattr(srv, 'nc' ) 
                bx  = getattr(srv, 'bx' )
                cl  = getattr(srv, 'cl' )
                npb = getattr(srv, 'npb')
                nbx = getattr(srv, 'nbx') 
                
                #oc2  = getattr(srv, 'oc2' ) #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!--------------------------
                #bx2  = getattr(srv, 'bx2' ) #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!--------------------------
                #nn   = getattr(srv, 'nn' )  #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!--------------------------
                #mk0  = getattr(srv, 'mk0' ) #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!--------------------------   
                po   = getattr(srv, 'po' )         


                for ix in range(len(npb)):
                    p           =  rd(p0,npb,ix)#......................................................................................................[<< 4 >>]
                    po, st, err =  cv2.calcOpticalFlowPyrLK(og, ng, p, None, **lp)

                    if po is not None:
                        if ix == 0:          
                            p1        =  po
                            nc[ix,:]  =  gc(po)#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< CALCULATE CENTROID
                        else:
                            pp       =   po
                            p1       =   np.append(p1,pp, axis=0) #<<<<<<<<<<<<<<<<<<<<<<<<<<<<< STACK LOCATION OF THE NEW FEATURES
                            nc[ix,:] =   gc(pp)#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<CALCULATE CENTROID WITH THE ACTUAL FEATURES LOC
                #"""                
                ub(bx, oc, nc)               #<<< UPDATE BBOX LOCATION------------------------------------------------------------------------[<< 5 >>]
                
                #nc = (nc+oc2)/2                                     #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!--------------------------
                #setattr(srv ,'oc2' , nc   )                         #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!--------------------------
                #setattr(srv ,'nn' ,  nn+1 )                         #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!--------------------------
                #ub2(bx, bx2, oc, nc); setattr(srv ,'bx2' , bx2 )    #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!--------------------------
                
                bx = bx.astype(int)                   #<<< ROUND NEW BOXES TO PLOThem 
                
                try:
                    nf =  pts(oc, nc, nf, mk, cl, bx) #<<< UPDATE FRAME WITH THE FEATURE CENTROIDS AND THE TRACKING PATHS..........................[<< 6 >>]
                except:
                    nf=nf
                #"""
                out = bdg.cv2_to_imgmsg(nf, "bgr8")    #<<< PREPARE FOR BROADCAST FORMAT
                pub.publish(out)
                
                setattr(srv ,'og' , ng ) # <<< UPDATE GRAY SCALE FRAMES FOR TRACKING
                setattr(srv ,'p0' , po ) # <<< UPDATE TRACKING DATA (LIST OF FEATURES)
                setattr(srv ,'oc' , nc ) # <<< UPDATE TRACKING CENTROIDS DATA                
                
                
            if getattr(srv, 'p0')  == [] :    
                out = bdg.cv2_to_imgmsg(nf, "bgr8")    #<<< PREPARE FOR BROADCAST FORMAT
                pub.publish(out)          
                
                setattr(srv ,'og' , ng ) # <<< UPDATE GRAY SCALE FRAMES FOR TRACKING
                setattr(srv ,'oc' , None ) # <<< UPDATE TRACKING CENTROIDS DATA 

            #time.sleep(0.03)
            
    except rospy.ROSInterruptException:
        pass
    
    
