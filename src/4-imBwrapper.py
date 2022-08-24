#!/usr/bin/env python3

#PENDING TO IMPLEMENT SOFT STITCH ON IMAGES 

import rospy
import numpy as np
from numpy.linalg import inv
import time
import argparse
import yaml
import cv2
import os
from numba import jit
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
#-----------------------------------------------------------------------------------------------------------------------------------------DICTIONARY
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------------------IDENTIFIERS OF THE CAMERAS
cid =  { 1:"l" , 2:"r", 3:"f", 4:"b"}
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------------------------LOAD VIRTUAL CAM MTX
def ldv(vc):
    
    with open(vc) as c:  
    
        vc = yaml.load(c, Loader=yaml.FullLoader) 
        
    ikd  =  vc.get("virtual_cam_mtx_inv") #INVERSE CAM DICTIONARY  
    iki  =  ikd.get("data")               #INVERSE CAM DATA OR INFO
    vki  =  np.array(iki)                 #INVERSE CAM MTX (list2mtx)    
    h    =  vc.get('h_vir')
    return(h, vki)
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#----------------------------------------------------------------------------------------------------------------------------LOAD INT-EXT CALIB DATA
def ld(rt, fc, tg): 
  
    with open(rt) as a:                              

        e = yaml.load(a, Loader=yaml.FullLoader)      # extrinsic data
    
    el  =  e.get(tg)                                  #get data from target (l,r,f...)  
    eli =  el.get('mtx_rt')                           #get rt info for the target                 
    rt  =  np.array(eli)                              #pass rt data from list to mtx             
 
    with open(fc) as b:                               
    
        i = yaml.load(b, Loader=yaml.FullLoader)      # intrinsic data
        
    il  =  i.get(tg)                                  #get data from target (l,r,f...)
    kdl =  il.get("k_mtx")          

    fax =  kdl[0];   fay = kdl[4];   ua0 = kdl[2];    va0 = kdl[5]
    
    fc  =  np.array([[fax, 0, ua0],[0, fay, va0],[0,0,1]])   #organize and get pinhole mtx                       
    
    return (rt,fc)    
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------------------------------PIXEL WRAPPERS
def pix2cor(px, h, kc_inv): 
    cor_1   =  np.matmul( (h*kc_inv), px )   
    return cor_1

def cor2pix(cor, h, kc):
    pix   =    np.matmul(( (1/h)*kc ), cor )
    return pix

def find_s_cor(v_cor, rt):    
    s_cor   =  np.matmul(rt,v_cor)    
    return s_cor
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------------------------GET THE WRAPPING MAP

def wmap(h, vki, rt_mtx, fc_kc, img):               #WRAPPING MAP

    v_img    =  np.zeros((500,500), np.uint8)
    wm       =  np.zeros((500,500,2), np.uint16)  #WRAPPING MAP

    for u in range(0,500):
        for v in range(0,500):

            v_pixel      =  np.array([[u],[v],[1]])        

            v_cordinate  =  pix2cor(v_pixel, h_vir, vki) 

            s_cordinate  =  find_s_cor(v_cordinate, rt_mtx)   

            H_VAR        =  s_cordinate[2] 

            s_pix        =  cor2pix(s_cordinate, H_VAR, fc_kc)  

            s_pix_int    =  np.array([[s_pix[0]],[s_pix[1]]],np.uint16)  

            if s_pix_int[0] > 0 and s_pix_int[0] < 1279 and s_pix_int[1] > 0 and s_pix_int[1] < 719:

                    wm[u,v,0] = s_pix_int[0]
                    wm[u,v,1] = s_pix_int[1]
            else:
                    wm[u,v,0] = 0   # NOISE
                    wm[u,v,1] = 0   # NOISE
    return(wm)
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#----------------------------------------------------------------------------------------------------------------------------------WRAPPING ON MASKS
@jit
def wrapp(lw, rw, fw, bw, m1, l,r,f,b):
   
    
    w,h = 500, 500
    #-----------------------------------------------------WRAPPER L
    for u in range(0,w):
        for v in range(0,h):

            x = lw[u,v,0]
            y = lw[u,v,1]

            m1[v,u,0] = l[y,x]        
            
    #-----------------------------------------------------WRAPPER R
    for u in range(0,w):
        for v in range(0,h):

            x = rw[u,v,0]
            y = rw[u,v,1]

            m1[v,u,1] = r[y,x]    
    #-----------------------------------------------------WRAPPER F
    for u in range(0,w):
        for v in range(0,h):

            x = fw[u,v,0]
            y = fw[u,v,1]

            m1[v,u,2] = f[y,x]        
    #-----------------------------------------------------WRAPPER B

    for u in range(0,w):
        for v in range(0,h):

            x = bw[u,v,0]
            y = bw[u,v,1]

            m1[v,u,3] = b[y,x]        

    return(m1)

#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------------------------------------CALLBACK
def brd(m):
    
    im   = bdg.imgmsg_to_cv2(m,"bgra8")
    
    l, r, f, b = cv2.split(im)
    
    l[0,0] = 0  # ADD ZERO PIXEL IN THE MIDDLE OF THE IMAGE TO KEEP EMPTY THE EMPTY PART OF THE MASK FOT THIS WRAPPING
    r[0,0] = 0  # ""
    f[0,0] = 0  # ""
    b[0,0] = 0  # ""
    
    m1u = wrapp(lw, rw, fw, bw, m1, l,r,f,b)

    l2 =  m1u[:,:,0]
    r2 =  m1u[:,:,1]
    f2 =  m1u[:,:,2]
    b2 =  m1u[:,:,3]

    o = cv2.addWeighted(f2,0.5,l2,1, 0)
    o = cv2.addWeighted(o,1,r2,0.5, 0)
    o = cv2.addWeighted(o,1,b2 ,0.5, 0)
    
    o = cv2.addWeighted(o,1,vm ,0.5, 0) #ADD_VEHICLE MASK 

    
    o = cv2.transpose(o)
    
    m = bdg.cv2_to_imgmsg(o, "mono8")
    p.publish(m)
    
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#--------------------------------------------------------------------------------------------------------------------------------------MAIN SEQUENCE

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='calibration data paths')
    
    parser.add_argument('-v','--virtualDataPath',  type=str,  default='/home/sierra/oni_dir/oni_ws/src/oni_pkg/yaml_config/virtualCam.yaml',  help='info of virtual cam')
        
    parser.add_argument('-e','--extrinsicDataPath',  type=str,  default='/home/sierra/oni_dir/oni_ws/src/oni_pkg/yaml_config/extrinsicData.yaml',  help='extrinsic calib data')
    
    parser.add_argument('-i','--intrinsicDataPath',  type=str,  default='/home/sierra/oni_dir/oni_ws/src/oni_pkg/yaml_config/intrinsicData.yaml',  help='intrinsic calib data')

    parser.add_argument('-m','--vehicleMaskPath',  type=str,  default='/home/sierra/oni_dir/oni_ws/src/oni_pkg/data/image/sandero_500x500_12kx12k.png',  help='vehicle mask')

    args, unknown = parser.parse_known_args()
    
    vm = cv2.imread(args.vehicleMaskPath,0)   #VEHICLE MASK
    vm = np.transpose(cv2.resize(vm, (500,500), interpolation= cv2.INTER_LINEAR))  # RESIZE VEHICLE MASK

    #----------------------------------------------------------------------------------------IMPORT VIRTUAL CAM INFO
    h_vir, vki   = ldv(args.virtualDataPath)  # H_VIR AND INVERSE VIRTUAL CAM MATRIX

    #----------------------------------------------------------------------------------------GET PATHS TO CALIBRATION DATA
    rts,fcs      =  args.extrinsicDataPath  ,  args.intrinsicDataPath    
    #----------------------------------------------------------------------------------------INITIALIZE INT-EXT DATA ARRAYS
    rt  = np.zeros((3,3,4)) #ARRAY TO STORE THE RT MATRIXES       1.DIM = L --- 2.DIM = R ---3.DIM = F
    fc  = np.zeros((3,3,4)) #ARRAY TO STORE THE PINHOLE MATRIXES  1.DIM = L --- 2.DIM = R ---3.DIM = F    

    #----------------------------------------------------------------------------------------FILL CALIBRATION DATA ARRAYS
    for i in range (len(cid)):                               

            t = cid[i+1] #TARGET                                                                    

            rt[:,:,i], fc[:,:,i] = ld(rts, fcs, t)
    
    try:


        bdg   = CvBridge()
        
        #####################################################################################################
        #----------------------------------------------------------------------------------------------------
        rospy.init_node('imBwrapper', anonymous = False)    
        rate   = rospy.Rate(30)    
        rospy.set_param('nodeimBwrapper', 1)                              #  -- ENABLE FLAG FOR THE "imBroadcaster" NODE
        rospy.loginfo('imBwrapperNodeInitialized')                        #  -- REGISTER THE INITIALIZATION OF THE NODE
        rospy.loginfo('loading wrapping pointers...')                     #  -- REGISTER THE INITIALIZATION OF THE NODE

        
        m0 = np.zeros((1280,720), np.uint8)  #MASK 0 TO FIND THE CORRESPONDING PIXELS FOR THE V IMAGE OR WRAPPING MAP
        m1 = np.zeros((500,500,4), np.uint8) #MASK 1 TO WRAPP PIXELS
        
        lrt,  lkc     =  rt[:,:,0]  ,  fc[:,:,0]        
        lw            =  wmap(h_vir, vki, lrt, lkc, m0) #LEFT WRAPPING MAP AND LEFT MASK

        rrt,  rkc     =  rt[:,:,1]  ,  fc[:,:,1]        
        rw            =  wmap(h_vir, vki, rrt, rkc, m0) #RIGHT WRAPPING MAP AND RIGHT MASK

        frt,  fkc     =  rt[:,:,2]  ,  fc[:,:,2]        
        fw            =  wmap(h_vir, vki, frt, fkc, m0) #FRONT WRAPPING MAP AND FRONT MASK

        brt,  bkc     =  rt[:,:,3]  ,  fc[:,:,3]        
        bw            =  wmap(h_vir, vki, brt, bkc, m0) #BACK WRAPPING MAP AND BACK MASK

        rospy.loginfo('wrapping pointers ready. virtual view generation is now available')    #  -- REGISTER THE INITIALIZATION OF THE NODE
            
        s              = rospy.Subscriber('/uIm', Image, brd)                   # (FUZZY CORRECTION SUBSCRIBER)   
        p              = rospy.Publisher('/bIm',  Image, queue_size=0.00005)
                
        rospy.spin()    


    except rospy.ROSInterruptException:
        pass
