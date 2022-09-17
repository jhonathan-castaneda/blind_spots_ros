#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np
from numba import jit       
import yaml
from numpy.linalg import inv
#-----------------------------------------------------------------------------------------------------------------------------------------DICTIONARY
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------------------------------------------------------CALIBRATION SEQUENCES
l_id = { 1:'l' , 2:'r'}
s_id = { 1: [1, 2, 4] , 2 : [5, 7, 8] } # SEQUENCES TO ORDER REFERENCE POINTS
#----------------------------------------------------------------------------------------------------------------------------------------------CLASS
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#------------------------------------------------------------------------------------------------------------------NODE SERVER CLASS - SUPPORT CLASS
class node_server:
    def __init__(self):           
        self.bc       = 0    # <<< SERVER BROADCASTING FLAG
        self.up       = 0    # <<< SERVER UPDATING FLAG 
        ########################################################################
        #self.html     = None # <<< HOMOGENEOUS TRANSFORMATION MATRIX FOR THE LEFT LIDAR
        #self.htmr     = None # <<< HOMOGENEOUS TRANSFORMATION MATRIX FOR THE RIGHT LIDAR
        ########################################################################
        #self.rad      = 0    # <<< RADIAL DISTANCE TO THE TARGET FOR CALIBRATION
        ########################################################################    
        self.lidar_l_refs    = np.zeros((3,1), np.float32) # <<< ARRAY TO LOAD THE LEFT REF POINTS WHILE CALIBRATING
        self.lidar_r_refs    = np.zeros((3,1), np.float32) # <<< ARRAY TO LOAD THE RIGHT REF POINTS WHILE CALIBRATING
        #self.lidar_l_detects = None#np.zeros((3,2), np.float32) # <<< ARRAY TO LOAD THE LEFT DETECTED REF POINTS WHILE CALIBRATING
        #self.lidar_r_detects = None#np.zeros((3,2), np.float32) # <<< ARRAY TO LOAD THE RIGHT DETECTED REF POINTS WHILE CALIBRATING
        ########################################################################
        self.lidar_l_p1  = 0  # <<< FLAG FOR REFERENCE POINT 1 OF THE LEFT LIDAR
        self.lidar_l_p2  = 0  # <<< FLAG FOR REFERENCE POINT 2 OF THE LEFT LIDAR
        self.lidar_l_p3  = 0  # <<< FLAG FOR REFERENCE POINT 3 OF THE LEFT LIDAR
        ########################################################################
        ########################################################################
        self.lidar_r_p1  = 0  # <<< FLAG FOR REFERENCE POINT 1 OF THE RIGHT LIDAR
        self.lidar_r_p2  = 0  # <<< FLAG FOR REFERENCE POINT 2 OF THE RIGHT LIDAR
        self.lidar_r_p3  = 0  # <<< FLAG FOR REFERENCE POINT 3 OF THE RIGHT LIDAR
        ########################################################################
        self.lidar_l  = 0  # <<< LEFT LIDAR CALIB FLAG   (1 WHEN CALIBRATED)
        self.lidar_r  = 0  # <<< RIGHT LIDAR CALIB FLAG  (1 WHEN CALIBRATED)
        self.calib    = 0  # <<< CALIB COMPLETION FLAG   (1 WHEN CALIBRATION HAS BEEN COMPLETED)
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD 
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------------------------------------FIND HMT         
def find_hmt(srv, ref_path, target):
    
    d        = np.zeros((16,3), np.float32) # ARRAY TO STORE THE DATA, FORMAT: X,Y, POINT ID
    coords_v = np.zeros((3,2), np.float32)  # ARRAY TO STORE THE REFERENCE POINTS
    #/////////////////////////////////////////////////////#IMPORT THE 16 REFERENCE POINTS IN 3D COORDINATES (mm)-----------------------
    with open(ref_path) as r:                              
        ap = yaml.load(r, Loader=yaml.FullLoader) #ALL POINTS

    for i in range (16): #FOR THE 16 CALIB POINTS
        t   = 'p'+str(i+1)         #TARGET POINT ... 1,2,3,4,5,6,7,8 ... 16
        lst = ap.get(t)

        d[i,0],  d[i,1],  d[i,2] = lst.get('x'),  lst.get('y'),  (i+1)
    #/////////////////////////////////////////////////////#EXTRACT THE CALIB SEQUENCE REFERENCE POINTS--------------------------------------
    sq  = s_id[target]     # 4 --- LOAD THE SCANNING SEQUENCE TO EXTRACT THE COORDS FROM EACH LANDMARK

    row = 0

    for i in range (3):    # 6 --- EXTRACT IN THE SEQUENTIAL ORDER THE LIST OF COORDS.
        k   = sq[i]          # THIS WILL BE 1,2,4 .... 5,7,8 ... x IS THE KEY TO COMPARE (ELEMENT OF THE SEQUENCE)
        
        for j in range(16): # SEARCH IN THE 16 ROWS OF POINT DATA
            
            if d[j,2] == k: 
                coords_v[row,0] , coords_v[row,1]  = d[j,0] , d[j,1]   ; row+=1        
            else:
                pass        
    
    #SOLVE EQUATIONS SYSTEM HERE/////////////////////////////////////////////////////////////////////////////
    vp_x = np.zeros((3,1), np.float32)
    vp_y = np.zeros((3,1), np.float32)
    
    
    if target == 1:
        lidar_ref = coords_l
        
        inter_lx = np.zeros((3,3))
        inter_ly = np.zeros((3,3))
             
        inter_lx[0,0], inter_lx[0,1], inter_lx[0,2] = lidar_ref[0,0], lidar_ref[0,1], 1
        inter_lx[1,0], inter_lx[1,1], inter_lx[1,2] = lidar_ref[1,0], lidar_ref[1,1], 1
        inter_lx[2,0], inter_lx[2,1], inter_lx[2,2] = lidar_ref[2,0], lidar_ref[2,1], 1

        inter_ly[0,0], inter_ly[0,1], inter_ly[0,2] = lidar_ref[0,0], lidar_ref[0,1], 1
        inter_ly[1,0], inter_ly[1,1], inter_ly[1,2] = lidar_ref[1,0], lidar_ref[1,1], 1
        inter_ly[2,0], inter_ly[2,1], inter_ly[2,2] = lidar_ref[2,0], lidar_ref[2,1], 1
                    
        res_x = np.matmul( inv(inter_lx), coords_v[:,0]) # EQUATIONS FROM X COMPONENTS
        res_y = np.matmul( inv(inter_ly), coords_v[:,1]) # EQUATIONS FROM Y COMPONENTS
        
        hmt   = np.array([ [res_x[0],  res_x[1],  res_x[2] ],[res_y[0],  res_y[1],  res_y[2] ],[0,0,1]   ])  
        
        
        rospy.loginfo('HMT FOR LEFT LIDAR READY.')      
        print('\n')
        print(hmt)
        print('\n')
        
        with open(lout+'lidar_'+l_id[target]+'hmt.npy', 'wb') as m4:
            np.save(m4, hmt)
        rospy.loginfo('LEFT LIDAR HMT EXPORTED TO: '+lout+'/lidar_'+l_id[target]+'hmt.npy' )      
        
        #USE HERE ONLY FOR INDIVIDUAL EXECUTION
        #####################################
        #####################################
        setattr(srv,'calib',1) 
        #####################################
        #####################################
        
    if target == 2:

        lidar_ref = coords_r

        inter_lx = np.zeroz((3,3))
        inter_ly = np.zeroz((3,3))
             
        inter_lx[0,0], inter_lx[0,1], inter_lx[0,2] = lidar_ref[0,0], lidar_ref[0,1], 1
        inter_lx[1,0], inter_lx[1,1], inter_lx[1,2] = lidar_ref[1,0], lidar_ref[1,1], 1
        inter_lx[2,0], inter_lx[2,1], inter_lx[2,2] = lidar_ref[2,0], lidar_ref[2,1], 1

        inter_ly[0,0], inter_ly[0,1], inter_ly[0,2] = lidar_ref[0,0], lidar_ref[0,1], 1
        inter_ly[1,0], inter_ly[1,1], inter_ly[1,2] = lidar_ref[1,0], lidar_ref[1,1], 1
        inter_ly[2,0], inter_ly[2,1], inter_ly[2,2] = lidar_ref[2,0], lidar_ref[2,1], 1
        
        
        res_x = np.matmul( inv(inter_lx), coords_v[:,0]) # EQUATIONS FROM X COMPONENTS
        res_y = np.matmul( inv(inter_ly), coords_v[:,1]) # EQUATIONS FROM Y COMPONENTS
        
        hmt   = np.array([ [res_x[0],  res_x[1],  res_x[2] ],[res_y[0],  res_y[1],  res_y[2] ],[0,0,1]   ])  

        rospy.loginfo('HMT FOR RIGHT LIDAR READY.')                     
        print('\n')
        print(hmt)
        print('\n')
        
        with open(lout+'lidar_'+l_id[target]+'hmt.npy', 'wb') as m5:
            np.save(m5, hmt)
        rospy.loginfo('RIGHT LIDAR HMT EXPORTED TO: '+lout+'/lidar_'+l_id[target]+'hmt.npy' ) 
 #--------------------------------------------------------------------------------------------------------------------------------------------METHOD2 
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-----------------------------------------------------------------------------------------------------------------------------FIND INDIVIDUAL POINTS        
def find_landmark(key, coords, srv, target_lid):     #target_lid is 1 or 2 for left and right
    
    rad2match = (getattr(srv, 'lidar_'+l_id[target_lid]+'_refs')[key,0])/1000  # <<< LOAD RADIOUS REF AND CONVERT TO METERS
    rads      = np.zeros((len(coords), 1), np.float32) # <<< ARRAY OF RADIAL DISTANCE MEASURED UP TO THE DETECTIONS
    
    rx        = np.zeros((len(coords), 1), np.float32)
    
    idx       = 0                                      # <<< INDEX TO STORE THE POSITION OF THE VALID SAMPLE FOR CALIB
    
    for i in range(len(rads)):
        a = np.power(coords[i,0], 2) 
        b = np.power(coords[i,1], 2) 
        r = np.sqrt( a + b )         # <<< RADIAL DISTANCE TO EACH X,Y POINT IN THE DETECTIONS 
        
        rx[i] = r
        
        if  r < (rad2match+0.1) and r > (rad2match-0.1) :  #IF RADIAL DISTANCE TO THE CLUSTER IS IN THE RANGE OF rad2match +-0.1 METERS (SENSIBLE RANGE)
            rads[i] = r                
            idx     = i
        else:
            rads[i] = 0
    
    #print(rads)
    #print(rad2match)
    #print(rx)
        #GET SURE THAT THE ACQUIRED POINT IS THE ONLY ONE IN THE SENSIBLE RANGE----------------------------------------------------------
    if rads[idx] == np.sum(rads) and rads[idx] !=0:                 # <<< IF THE SAMPLE IS THE ONLY VALID VALUE IN THE SENSIBLE RANGE        
        if target_lid ==1: # IF LEFT LIDAR           
             coords_l[key,0] ,  coords_l[key,1] = coords[idx,0], coords[idx,1]
                  
        if target_lid ==2: #IF RIGHT LIDAR         
             coords_r[key,0] ,  coords_r[key,1] = coords[idx,0], coords[idx,1]      
            
        setattr(srv, ('lidar_'+l_id[target_lid]+'_p'+str(key+1)) , 1 )                                         #SET FLAG FOR Px AS CALIBRATED       
        rospy.loginfo('REF. P'+str(key+1)+'=['+str(coords[idx,0])+','+str(coords[idx,1])+']'+' FOR LIDAR_'+l_id[target_lid]+' ACQUIRED.')                     
    else:
        pass        
#--------------------------------------------------------------------------------------------------------------------------------------------METHOD1 
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#----------------------------------------------------------------------------------------------------------------------------------------SCAN POINTS    
def scan(coords, srv, target):    
           
    if getattr(srv, 'lidar_'+l_id[target]+'_p1') == 0:  #IF POINT 1 FOR LIDAR L OR R IS NOT READY, FIND IT
        find_landmark(0,coords, srv, target)            # <<< FIND POINT 1 OF THE RESPECTIVE CALIB SEQUENCE
            
    if getattr(srv, 'lidar_'+l_id[target]+'_p1') == 1:     # IF POINT 1 FOR LIDAR L OR R IS READY AND ...
        if getattr(srv, 'lidar_'+l_id[target]+'_p2') == 0: # POINT 2 FOR LIDAR L OR R IS NOT READY, FIND IT
            find_landmark(1,coords, srv, target)           # <<< FIND POINT 2 OF THE RESPECTIVE CALIB SEQUENCE
                    
    if getattr(srv, 'lidar_'+l_id[target]+'_p2') == 1:     # IF POINT 2 FOR LIDAR L OR R IS READY , AND ...
        if getattr(srv, 'lidar_'+l_id[target]+'_p3') == 0: # POINT 3 FOR LIDAR L OR R IS NOT READY, FIND IT
            find_landmark(2,coords, srv, target)           # <<< FIND POINT 3 OF THE RESPECTIVE CALIB SEQUENCE
            
    if getattr(srv, 'lidar_'+l_id[target]+'_p3') == 1:     # IF ALL THE PONTS HAVE BEEN EXTRACTED, EXPORT THEM
        
        setattr(srv,'lidar_'+l_id[target],1)               # SET TO ONE THE CALIB FLAG FOR THE RESPECTIVE LIDAR          
#---------------------------------------------------------------------------------------------------------------------------------CALLBACK-METHOD1.1 
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#---------------------------------------------------------------------------------------------------------------------DECODE BOSTER 1 FOR PC2 DECODE
@jit
def f1(d,o,rrr,ccc):   
    idx = 0
    for r in range(rrr):
        for c in range(ccc):
            o[r,c] = d[idx] ; idx += 1
    return o
#-----------------------------------------------------------------------------------------------------------------------------------CALLBACK-METHOD1
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------------------DECODE LDAR PC2 DETECTIONS
def deco(msg):
    if getattr(srv, 'up') == 0:   # <<< IF NOT UPDATING
        setattr(srv,'up',1)       # <<< BLOCK SERVER FOR UPDATE
        
        data  = msg.data                           # <<< SERIALIZED TRACKING DATA 
        cc    = 2                                  # <<< COLS, STATIC VALUE
        rr    = round(len(data)/cc)                # <<< ROWS, DYNAMIC VALUE         
        o1    = np.zeros((rr,cc), np.float32)
        o2    = f1(data, o1, rr, cc)               # <<< FILL DATA MTX IN ORDERED FORMAT FOR THE OBSERVER     
        
        ############################################################################
        if getattr(srv, 'lidar_l') == 0:
            scan(o2, srv, 1)                        # <<< TRY TO CALIB THE LEFT LIDAR
        #if getattr(srv, 'lidar_l') == 1:                    
        #    if getattr(srv, 'lidar_r') == 0:       
        #        scan(o2, srv, 2)                   # <<< TRY TO CALIB THE RIGHT LIDAR
        #if getattr(srv, 'lidar_l') == 1:                    
        #    if getattr(srv, 'lidar_r') == 1:       
        #        pass                               # <<< IF ALL READY.. PASS..
        if getattr(srv, 'lidar_l') == 1:
            
            #USE HERE ONLY FOR INDIVIDUAL EXECUTION
            #####################################
            #####################################
            if getattr(srv, 'calib') == 0:
            #####################################
            #####################################
                find_hmt(srv, rp, 1)
                
                
                
        if getattr(srv, 'calib') == 1:
            pass

        #############################################################################
        setattr(srv,'up',0)                        # <<< BRELEASE SERVER FOR BROADCASTING

    if getattr(srv, 'up') == 1:   # <<< IF UPDATING...
        pass        
#--------------------------------------------------------------------------------------------------------------------------------------------METHOD0
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#--------------------------------------------------------------------------------------------------------------------------LOAD REFS FOR LIDAR CALIB   
def load_lidar_refs(srv, path):
    
    dl = np.zeros((3,1), np.float32) # ARRAY TO STORE THE DATA (DATA LEFT)
    dr = np.zeros((3,1), np.float32) # ARRAY TO STORE THE DATA (DATA RIGHT)
    #########################################################################################################
    with open(path) as r:                              
        ap = yaml.load(r, Loader=yaml.FullLoader) #ALL POINTS
    #########################################################################################################
    row = 0
    apl = ap.get('l')      # OPEN POINTS FOR LEFT LIDAR
    for ix in s_id[1]:     # LEFT LIDAR
        t   = 'p'+str(ix)  # REFERENCE POINT p1, p2, p4 .... IN GENERAL... 1,2,4  FOR LEFT 
        lst = apl.get(t)
        dl[row,0]        = lst.get('r')   ; row += 1        
    #########################################################################################################
    row = 0
    apr = ap.get('r')      # OPEN POINTS FOR RIGHT LIDAR
    for ix in s_id[2]:     # LEFT LIDAR
        t   = 'p'+str(ix)  # REFERENCE POINT p5, p7, p8 .... IN GENERAL... 5,7,8 FOR RIGHT
        lst = apr.get(t)
        dr[row,0]        = lst.get('r')   ; row += 1       
    #########################################################################################################
    setattr(srv, ('lidar_l_refs') , dl )   #LOAD THE REFERENCE POINTS FOR LEFT LIDAR INTO THE SERVER 
    setattr(srv, ('lidar_r_refs') , dr )   #LOAD THE REFERENCE POINTS FOR RIGHT LIDAR INTO THE SERVER 
#--------------------------------------------------------------------------------------------------------------------------------------MAIN SEQUENCE  
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#---------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    try:
        rp   = '/home/oni/oni_dir/oni_ws/src/oni_pkg/yaml_config/refPoints.yaml'   #REFERENCE POINTS PATH (ARUCO COORDS WITH RESPECT TO VEHICLE FRAME)
        lp   = '/home/oni/oni_dir/oni_ws/src/oni_pkg/yaml_config/lidarPoints.yaml' #REFERENCE POINTS PATH (ARUCO COORDS WITH RESPECT TO LIDAR FRAME)
        lout = '/home/oni/oni_dir/oni_ws/src/oni_pkg/data/lidar_hmt'               #BASE PATH FOR EXPORTATION OF HMTs
                
        rospy.init_node('lidarCalib', anonymous=False)
        rospy.loginfo('lidarCalibNodeInitialized')            
        p1         =  rospy.Publisher( "/lidar_calib", Float32MultiArray, queue_size=1)
        laserSub   =  rospy.Subscriber("/lidar_det",   Float32MultiArray, deco)
        ldt        =  Float32MultiArray()                                          
        srv        =  node_server()    
        
        load_lidar_refs(srv, lp)
                
        global coords_l
        global coords_r
        coords_l = np.zeros((3,2))  # TEMPORAL ARRAYS TO LOAD REFERENCE POINTS THROUGH THE CALIB BEFORE COMPLETED
        coords_r = np.zeros((3,2))  # TEMPORAL ARRAYS TO LOAD REFERENCE POINTS THROUGH THE CALIB BEFORE COMPLETED
   
        rospy.spin()
        
    except rospy.ROSInterruptException:    
        pass