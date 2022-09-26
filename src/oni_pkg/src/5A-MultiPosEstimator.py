#!/usr/bin/env python3

import rospy
import time
import numpy as np
import cv2
import matplotlib.pyplot as plt
import argparse
from numba import jit           
from cv_bridge import CvBridge
import message_filters

from oni_pkg.msg import track
from sensor_msgs.msg import Image
#-----------------------------------------------------------------------------------------------------------------------------------------DICTIONARY
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------------------IDENTIFIERS OF THE CAMERAS
cid =  { 1:"l" , 2:"r", 3:"f", 4:"b"}
#----------------------------------------------------------------------------------------------------------------------------------------------CLASS
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#-------------------------------------------------------------------------------------------------------------DEFINITION OF THE HANDLER CLASS SERVER
class observator1:
    def __init__(self):#, dimg_f, dimg_b):
        self.tracks  = None    
        self.bc      = 0    # <<< SERVER BROADCASTING FLAG
        self.up      = 0    # <<< SERVER UPDATING FLAG   
#---------------------------------------------------------------------------------------------------------------------------------------- 
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------------------------------------------------------DRAW TRACKS        
def draw_bird_tracks(img, data):

    
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    
    for i in range(len(data)): 

        if ns[ data[i,2] , data[i,3] ] ==255  and bu[ data[i,2] , data[i,3] ] ==255:
            col=(0,0,200)
        
        if ns[ data[i,2] , data[i,3] ] == 0 and bu[ data[i,2] , data[i,3] ]==0 :
            col=(255,255,255)
            
        if ns[ data[i,2] , data[i,3] ] ==255  and bu[ data[i,2] , data[i,3] ] == 0:
            col=(0,200,0)            

        if ns[ data[i,2] , data[i,3] ] ==0  and bu[ data[i,2] , data[i,3] ] == 255:
            col=(0,0,200)    
            
            
        #TO PLOT WE TRANSPOSE, PLACING Y,X INSTEAD X,Y BELOW
        cv2.circle(img, (   data[i,3] , data[i,2] ), 2, col, 3)

        mmx, mmy   = 16*(-(data[i,2])+250)/1000 , 16*(-(data[i,3])+250)/1000        #16mm PER PIXEL FOR SQUARE 8000 X 8000 mm BIRD IMAGE AND /1000 FOR METERS
        mmx, mmy   = ("%.2f" % mmx) , ("%.2f" % mmy)
        
        text       =  'ID: '+str(data[i,0])+'\n'+'CLASS: '+str(data[i,1])+'\n'+'LOC: '+str(mmx)+', '+str(mmy) 
        
        x0, y0     =  data[i,3] +10 , data[i,2]
        
        dx,dy      =  15,10

        for idx, line  in enumerate(text.split('\n')):
            y = y0 + idx*dy
            cv2.putText(img, line, (x0,y), cv2.FONT_HERSHEY_SIMPLEX, 0.27, col, 1)  
    
    return(img)    
#-------------------------------------------------------------------------------------------------------------------------------------------------------------
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------------------------------------------------------------------RE-SCALE U,V COORDS
def resc(u, v, lh, rh, fh, bh):
    
    un, vn, hh  = 0,0,0
    ######################################################
    if   u<=1279 and v <= 719:                   #LEEFT IM  
        un, vn = u, v
        hh     = lh           # << SELECT l HOMOGRAPHY MTX
    ######################################################   
    elif u<=1279 and v > 719:                   #RIGHT IM 
        un, vn = u, (v- 719)
        hh     = rh           # << SELECT r HOMOGRAPHY MTX
    ######################################################    
    elif u>1279 and  v <=  719:                  #FRONT IM  
        un, vn = (u-1279), v
        hh     = fh           # << SELECT f HOMOGRAPHY MTX
    ######################################################    
    elif u>1279 and  v >  719:                    #BACK IM    
        un, vn = (u-1279), (v- 719)
        hh     = bh           # << SELECT b HOMOGRAPHY MTX
    ######################################################
    return(un, vn, hh) 
#----------------------------------------------------------------------------------------------------------------------------------------VOID METHOD 
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#---------------------------------------------------------------------------------------------------------------------------------UPDATE OBSERVATOR1
def upob1(dt, obs, rows):    # <<< INPUTS: DATA ARRAY, OBSERVATOR OBJECT, NUMBER OF ROWS OF THE dt ARRAY (NUMBER OF TRACKS)

    out          =  np.zeros((rows,6), np.int16)     # <<< AS MUCH ROWS AS TRACKS: COL1...6 >> TRACK_ID, CLASS_ID, PIX_U, PIX_V, POSX, POSY    
    out[:,0]     =  dt[:,0]                          # <<< KEEP THE FIRST TWO COLUMNS EQUAL SINCE THEY ARE THE TRACK IDS AND THE CLASS IDS
    out[:,1]     =  dt[:,1]                          # <<< KEEP THE FIRST TWO COLUMNS EQUAL SINCE THEY ARE THE TRACK IDS AND THE CLASS IDS

    for t in range(rows):
        trc              =  dt[t,:]                                           # <<< CURRENT TRACK ROW OF INFO (6 COLUMNS)
        c2, c3, c4, c5   =  trc[2], trc[3], trc[4], trc[5]                    # <<< SPLIT DATA OF THE BBOXES (TOP LEFT CORNER AND BOTOM RIGHT CORNER)
        ut, vt           =  int( round( ( c4 + c2)/2) )  , int(round( c5 ))   # <<< FIND BASE POINT (BBOX CENTROID IN X AND BOTOM OF BBOX IN Y)
        ut2, vt2, h4     =  resc(ut, vt, lh, rh, fh, bh)                      # <<< RE-SCALE PIXELS, AND SELECT RESPECTIVE HOMOGRAPHY TEMPLATE MTX      
        sp     =  np.array([[ ut2 ],[ vt2 ],[ 1 ]])                           # <<< SURROUND PIXEL IN HOMOGENEOUS COORDS.
        vp1    =  np.matmul(h4,sp)                                            # <<< VIRTUAL PIXEL IN HOMOGENEOUS COORDS(NOT NORMALIZED).
        vp2    =  vp1/vp1[2]                                                  # <<< VIRTUAL PIXEL IN HOMOGENEOUS COORDS(NORMALIZED !!!).
        ub, vb =  vp2[0], vp2[1]                                              # <<< VIRTUAL PIXEL COMPONENTS READY TO BE EXTRACTED.
        
        out[t,2] , out[t,3] = ub, vb
        
    return(out)
#---------------------------------------------------------------------------------------------------------------------------------CALLBACK-METHOD1.1 
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------------DECODE BOSTER 1 FOR IMAGE DECODE
@jit
def f1(d,o,rrr,ccc):
        
    idx = 0
    for r in range(rrr):
        for c in range(ccc):
            o[r,c] = d[idx]
            idx += 1   
    return o

#-----------------------------------------------------------------------------------------------------------------------------------CALLBACK-METHOD1
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------DECODE TRACKING DATA FROM IMAGE MODULE
def deco(m1, m2):
    
    if m1.tracks is not []:
    
        if getattr(obs, 'bc') == 0:   # <<< IF NOT BROADCASTING

            setattr(obs,'up',1)       # <<< BLOCK SERVER FOR UPDATE

            data  = m1.tracks                      # <<< SERIALIZED TRACKING DATA 
            cc    = 6                               # <<< COLS, STATIC VALUE
            rr    = round(len(data)/cc)             # <<< ROWS, DYNAMIC VALUE         
            o1    = np.zeros((rr,cc), np.int16)
            o2    = f1(data, o1, rr, cc)            # <<< FILL DATA MTX IN ORDERED FORMAT FOR THE OBSERVER 
            out   = upob1(o2, obs, rr)              # <<< UPDATE TRACKS IN OBSERVATOR 1 
            setattr(obs,'up',0)                     # <<< BRELEASE SERVER FOR BROADCASTING

            #ONLY FOR DEBUG------------------------------------------1
            #img      =  np.zeros((500,500), np.uint8)
            #img[:,:] =  255
            
            img      =  bdg.imgmsg_to_cv2(m2,"mono8")
            #img      =  cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            
            img2     =  draw_bird_tracks(img, out)
            msk      =  cv2.addWeighted(ns2, 0.5, bu2,0.5,0)
            img3     =  cv2.addWeighted(img2,1,  msk,0.2, 0)              
            
            m        =  bdg.cv2_to_imgmsg(img3, "bgr8") 
            p.publish(m)
            #ONLY FOR DEBUG------------------------------------------1


        if getattr(obs, 'bc') == 1:   # <<< IF BROADCASTING

            pass

        #ONLY FOR DEBUG------------------------------------------2
        arr = getattr(obs, 'tracks')
        #print (arr)
        #ONLY FOR DEBUG------------------------------------------2
        
        
    else:
        pass

#--------------------------------------------------------------------------------------------------------------------------------------MAIN SEQUENCE  
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#---------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__": 
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--virtualDataPath',    type=str,  default='/home/oni/oni_dir/oni_ws/src/oni_pkg/yaml_config/virtualCam.yaml',  help='info of virtual cam')
    parser.add_argument('--HomographyPath', type=str,  default='/home/oni/oni_dir/oni_ws/src/oni_pkg/data/homography/')
    parser.add_argument('--vehicleMaskPath',  type=str,  default='/home/oni/oni_dir/oni_ws/src/oni_pkg/data/image/sandero_500x500_12kx12k.png',  help='vehicle mask')    
    parser.add_argument('--nioshMaskPath',  type=str,  default='/home/oni/oni_dir/oni_ws/src/oni_pkg/data/image/niosh.png')    
    parser.add_argument('--bubbleMaskPath',  type=str,  default='/home/oni/oni_dir/oni_ws/src/oni_pkg/data/image/bubble.png')    

    args, unknown = parser.parse_known_args()
    print(args)

    vm = cv2.imread(args.vehicleMaskPath,0)   #VEHICLE MASK
    vm = cv2.resize(vm, (500,500), interpolation= cv2.INTER_LINEAR)  # RESIZE VEHICLE MASK
    #vm = np.transpose(cv2.resize(vm, (500,500), interpolation= cv2.INTER_LINEAR))  # RESIZE VEHICLE MASK

    ns  = cv2.imread(args.nioshMaskPath,0)
    ns2 = np.zeros((500,500,3),np.uint8)
    #ns3 = cv2.flip(ns,1)
    
    for i in range(500):
        for j in range(500):
            if ns[i,j]==255:
                ns2[i,j,1] = 200

    bu  = cv2.imread(args.bubbleMaskPath,0)
    bu2 = np.zeros((500,500,3),np.uint8)

    for i in range(500):
        for j in range(500):
            if bu[i,j]==255:
                bu2[i,j,1] = 200

   
    #LOAD HOMOGRAPHY MATRIXES/////////////////////////////////////////////////////////////
    global lh # <<< LEFT CAM HOMOGRAPHY MTX
    global rh # <<< RIGHT CAM HOMOGRAPHY MTX
    global fh # <<< FRONT CAM HOMOGRAPHY MTX
    global bh # <<< BACK CAM HOMOGRAPHY MTX

    with open(args.HomographyPath+'l_homography_mtx.npy', 'rb') as y1:
        lh = np.load(y1)          
    #\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
    with open(args.HomographyPath+'r_homography_mtx.npy', 'rb') as y2:
        rh = np.load(y2)                
    #\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
    with open(args.HomographyPath+'f_homography_mtx.npy', 'rb') as y3:
        fh = np.load(y3)
    #\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
    with open(args.HomographyPath+'b_homography_mtx.npy', 'rb') as y4:
        bh = np.load(y4)            
    #\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ 
       
    try:
        #####################################################################################################
        #----------------------------------------------------------------------------------------------------
        rospy.init_node('poseEstimator1', anonymous = False)                
        rate   = rospy.Rate(30)                                             
        rospy.set_param('nodeposeEstimator1', 1)                             
        rospy.loginfo('poseEStimator1NodeInitialized')          
        
        s1     = message_filters.Subscriber('/yolo', track) 
        s2     = message_filters.Subscriber('/bIm',  Image) 
        
        p     = rospy.Publisher('/pose',  Image, queue_size=0.0005)     # DISTORTED IMAGE PUBLISHER
        s     = message_filters.ApproximateTimeSynchronizer([ s1,s2 ], queue_size=1, slop=0.05)
        s.registerCallback(deco)
        #/////////////////////////////////////////////////////////////////////////////////////
        
        bdg = CvBridge()                     # -- INITIALIZE ROS-CV2 BRIDGE
        obs = observator1()                         # <<< INITIALIZE OBSERVATOR 1 (IMAGE DATA)

        rospy.spin()   

    except rospy.ROSInterruptException:
          
        pass
