#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from numpy.linalg import inv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import argparse
import yaml
import cv2.aruco as aruco
import time as t
import math
import os

#-----------------------------------------------------------------------------------------------------------------------------------------DICTIONARY
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------------------IDENTIFIERS OF THE CAMERAS
c_id = { 1:'l' , 2:'r', 3:'f', 4:"b"}
s_id = { 1: [1, 2, 3, 4] , 2 : [6, 5, 7, 8], 3 : [4, 3, 5, 6], 4 : [8, 7, 2, 1] } # SEQUENCES TO ORDER REFERENCE POINTS
#----------------------------------------------------------------------------------------------------------------------------------------------CLASS
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#-------------------------------------------------------------------------------------------------------------DEFINITION OF THE HANDLER CLASS SERVER
class server:

    def __init__(self):#, dimg_f, dimg_b):
        
        self.l  = None 
        self.r  = None
        self.f  = None        
        self.b  = None         
        
        self.l_p  = 0 # LEFT REFERENCE POINTS
        self.r_p  = 0 # RIGHT REFERENCE POINTS
        self.f_p  = 0 # FRONT REFERENCE POINTS
        self.b_p  = 0 # BACK REFERENCE POINTS
        
        self.l_f = 0  # LEFT LOADING FLAG
        self.r_f = 0  # RIGHT LOADING FLAG
        self.f_f = 0  # FRONT LOADING FLAG
        self.b_f = 0  # BACK LOADING FLAG
        
        self.st  = 0  #FLAG TO NOTIFY IF THE PIXELS FROM LANDMARKS ARE ALREADY EXTRACTED AND EXPORTED TO .YAML CALIB FILE       
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   --EX7--
#------------------------------------------------------------------------------------------------------------------MAP FROM 3D COORDINATES TO PIXELS
def cor2pix(cor, h, kc):
    pix   =    np.matmul(( (1/h)*kc ), cor )
    return pix
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   --EX6--
#----------------------------------------------------------------------------IMPORT REFERENCE POINTS, MAP TO PIXELS AND STACK IN THE CALIB SEQUENCES
def p2pix(rp, srv2, v, h):
    
    d = np.zeros((16,3)) # ARRAY TO STORE THE DATA, FORMAT: X,Y, POINT ID

    #/////////////////////////////////////////////////////#IMPORT THE 16 REFERENCE POINTS IN 3D COORDINATES (mm)-----------------------
    with open(rp) as r:                              

        ap = yaml.load(r, Loader=yaml.FullLoader) #ALL POINTS

    for i in range (16): #FOR THE 16 CALIB POINTS

        t   = 'p'+str(i+1)         #TARGET POINT ... 1,2,3,4,5,6,7,8 ... 16
        lst = ap.get(t)

        d[i,0],  d[i,1],  d[i,2] = lst.get('x'),  lst.get('y'),  (i+1)

    #/////////////////////////////////////////////////////#CONVERT 3D POINTS TO 2D PIXELS---------------------------------------------
    for i in range (16): 

        p  = np.array([ [  -d[i,0] ],[ -d[i,1] ],[h]])  # WE USE THE MINUS TO INVERT THE PIXELS X, Y SINCE THE PINHOLE PROJECTION INVERTS THE IMAGE

        pix= cor2pix(p, h, v)

        d[i,0] = pix[0]
        d[i,1] = pix[1]

    #/////////////////////////////////////////////////////#ORGANIZE AND EXTRACT THE CALIB SEQUENCE REFERENCE POINTS------------------

    for s in range (len(s_id)): # <------------ SCANNING SEQUENCES (3)

        sq  = s_id[s+1]        # 4 --- LOAD THE SCANNING SEQUENCE TO EXTRACT THE COORDS FROM EACH LANDMARK
        x,y = [],[]            # 5 --- INITIALIZE EMPTY STRINGS FOR OUTPUT

        for i in range (4):    # 6 --- EXTRACT IN THE SEQUENTIAL ORDER THE LIST OF COORDS. X AND Y AND STORE THEM AT "out_str_x" & "out_str_y"

            k = sq[i]          # THIS WILL BE 3, 7, 5, 1 .... 2, 6, 8, 4   ....  1, 5, 6, 2 ... x IS THE KEY TO COMPARE (ELEMENT OF THE SEQUENCE)
            for j in range(16): # SEARCH IN THE 16 ROWS OF POINT DATA

                if d[j,2] == k: 

                    if i == 0:     #if the KEY IS THE FIRST ONE OF THE SEQUENCE
                        x = d[j,0] # GET THE X COORD 
                        y = d[j,1] # GET THE Y COORD
                    else:
                        x =  np.append(x ,(d[j,0]) ) #STACK THE X COORD WITH THE ONE OF THE FIRST KEY
                        y =  np.append(y ,(d[j,1]) ) #STACK THE Y COORD WITH THE ONE OF THE FIRST KEY
                else:
                    pass

        o = (np.append(x,y)).tolist()

        setattr(srv2, (c_id[s+1]+'_p') , o )   #LOAD THE REFERENCE POINTS FOR THE CURRENT IMAGE SOURCE IN THE SERVER   
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   --EX5--
#------------------------------------------------------------------------------------------------------------------EXPORT REFERENCE POINTS INTO YAML        
def expo(rp, srv, srv2, vc_kc, h_vir):
    
    p2pix(rp, srv2, vc_kc, h_vir) #IMPORT THE REFERENCE POINTS FROM THE VIRTUAL IMAGE INTO THE srv2 OBJECT
    
    op = '/home/oni/oni_dir/oni_ws/src/oni_pkg/yaml_config/points4ExtCalib.yaml'  #OUTPUT PATH TO EXPORT THE .YAML FILE WITH THE REFERENCE POINTS
    
    out = { 'l': {'in_img': getattr(srv, 'l_p') ,  'vir_img': getattr(srv2, 'l_p') }, 
           
               'r':  {'in_img':  getattr(srv, 'r_p') , 'vir_img': getattr(srv2, 'r_p') },
                
               'f':  {'in_img':  getattr(srv, 'f_p') , 'vir_img': getattr(srv2, 'f_p') },   
           
               'b':  {'in_img':  getattr(srv, 'b_p') , 'vir_img': getattr(srv2, 'b_p') } 
                 
                 }
        
    with open(op, 'w') as o:
        
        yaml.dump(out, o,sort_keys=False) 
    
    rospy.loginfo('EXTRACTION OF REFERENCE LANDMARKS READY. DATA AVAILABLE AT: /oni_pkg/yaml_config/points4ExtCalib.yaml')
    
    setattr(srv, 'st' , 1) #SET THE SERVER FLAG AS 1 INDICATING THAT THE EXTRACTION OF THE REFERENCE POINTS HAS BEEN COMPLETED
    
    rospy.loginfo('LANDMARKS READY -- SHUTTING DOWN ARUCO DETECTOR AND LAUNCHING RT SOLVER...') 
    os.system("roslaunch oni_pkg extCalib2.launch")
    rospy.set_param('nodearucoDetector', 0)
    os.system("rosnode kill arucoDetector")

#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   --EX4--
#-------------------------------------------------------------------------------------------------------------------------------LOAD DETECTIONS INFO    
def load(cen, id):
    
    sq  = s_id[id]       # 4 --- LOAD THE SCANNING SEQUENCE TO EXTRACT THE COORDS FROM EACH LANDMARK
    x,y = [],[]          # 5 --- INITIALIZE EMPTY STRINGS FOR OUTPUT
    for i in range (4):  # 6 --- EXTRACT IN THE SEQUENTIAL ORDER THE LIST OF COORDS. X AND Y AND STORE THEM AT "out_str_x" & "out_str_y"
        
        k = sq[i]        # THIS WILL BE 3, 7, 5, 1 .... 2, 6, 8, 4   ....  1, 5, 6, 2 ... x IS THE KEY TO COMPARE (ELEMENT OF THE SEQUENCE)
        for j in range(4): # SEARCH IN THE 4 BBOXES
                
            if cen[j,2] == k: 

                if i ==0: #if the KEY IS THE FIRST ONE OF THE SEQUENCE
                    x = cen[j,0] # GET THE X COORD 
                    y = cen[j,1] # GET THE Y COORD
                else:
                    x =  np.append(x ,(cen[j,0]) ) #STACK THE X COORD WITH THE ONE OF THE FIRST KEY
                    y =  np.append(y ,(cen[j,1]) )#STACK THE Y COORD WITH THE ONE OF THE FIRST KEY
            else:
                pass
                            
    o = (np.append(x,y)).tolist()
    
    print(o)
        
    setattr(srv, (c_id[id]+'_p') , o )   #LOAD THE REFERENCE POINTS FOR THE CURRENT IMAGE SOURCE IN THE SERVER
    setattr(srv, (c_id[id]+'_f') , 1 )   #SET THE LOADING FLAG FOR THE CURRENT IMAGE SOURCE
    setattr(srv, (c_id[id]) , cen)       #LOAD THE DETECTED CENTROIDS INTO THE SERVER FOR EACH IMAGE SOURCE
    
    rospy.loginfo('LANDMARKS FROM CHANNEL '+c_id[id]+' ACQUIRED AND STORED')
    #THE SERVER OBJECT MUST BE NAMED "srv"
    
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   --EX3.1--
#----------------------------------------------------------------------------------------------PRINT VALUE OF REFERENCE PIXELS FROM ARUCO DETECTIONS   
    
def print_detections(frame, cen, iD):
    
    
    sq  = s_id[iD]       # 4 --- LOAD THE SCANNING SEQUENCE TO EXTRACT THE COORDS FROM EACH LANDMARK
    x,y = [],[]          # 5 --- INITIALIZE EMPTY STRINGS FOR OUTPUT
    for i in range (4):  # 6 --- EXTRACT IN THE SEQUENTIAL ORDER THE LIST OF COORDS. X AND Y AND STORE THEM AT "out_str_x" & "out_str_y"
        
        k = sq[i]        # THIS WILL BE 3, 7, 5, 1 .... 2, 6, 8, 4   ....  1, 5, 6, 2 ... x IS THE KEY TO COMPARE (ELEMENT OF THE SEQUENCE)
        for j in range(4): # SEARCH IN THE 4 BBOXES
                
            if cen[j,2] == k: 

                if i ==0: #if the KEY IS THE FIRST ONE OF THE SEQUENCE
                    x = cen[j,0] # GET THE X COORD 
                    y = cen[j,1] # GET THE Y COORD
                else:
                    x =  np.append(x ,(cen[j,0]) ) #STACK THE X COORD WITH THE ONE OF THE FIRST KEY
                    y =  np.append(y ,(cen[j,1]) )#STACK THE Y COORD WITH THE ONE OF THE FIRST KEY
            else:
                pass    

    
    overlay    =  frame.copy()
    alpha      =  0.6
    cv2.rectangle(frame, (17,20), (432, 170), (0,0,0), -1)  
    im_new     =  cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)
    img        =  im_new
    font       =  cv2.FONT_HERSHEY_SIMPLEX
    fontScale  =  0.7
    fontColor  =  (255, 255, 255)
    lineType   =  1
    text       =  '\n'+'CHANNEL:  '+c_id[iD]+'\n\n'+'REF. PIXEL   U   ,   V    ARUCO ID:' +'\n'+'PIX_1:     '+str(x[0])+' , '+str(y[0])+'     '+str(sq[0])+ '\n' +'PIX_2:     '+str(x[1])+' , '+str(y[1])+'     '+str(sq[1])+ '\n'  +'PIX_3:     '+str(x[2])+' , '+str(y[2])+'     '+str(sq[2])+ '\n'+'PIX_4:     '+str(x[3])+' , '+str(y[3])+'     '+str(sq[3])
    
    
    y0, dy     =  20, 20
    
    for i, line in enumerate(text.split('\n')):
        y = y0 + i*dy
        cv2.putText(img, line, (19,y), font, fontScale, fontColor, lineType)  
        
    return(img)    

#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   --EX3--
#---------------------------------------------------------------------------------------------------------------------------------FIND THE LANDMARKS
def detect(im, markerSize=4, totalMarkers = 250, draw=True ):
 
    imG   = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)  
    #imG = im
    
    k     = getattr(aruco,f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    Dict  = aruco.Dictionary_get(k)
    Param = aruco.DetectorParameters_create()
    bbox, ids, rejected = aruco.detectMarkers(imG, Dict, parameters=Param)
    aruco.drawDetectedMarkers(im, bbox, ids)
    
    return(bbox, ids)
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||  --EX2--
#----------------------------------------------------------------------------------------CHECK IMG, TO DETECT LANDMARKS AND EXTRACT REFERENCE POINTS
def check(im, Id): #im is the frame and Id the image source identifier  
    bgr        = cv2.cvtColor(im ,cv2.COLOR_GRAY2BGR) 
    bbox, ids  = detect(bgr)                          #  --- SEARCH FOR ARUCO TARGETS 
    o          = bdg.cv2_to_imgmsg(bgr, "bgr8")       #  --- SET OUTPUT MSG, ARUCO DETECTIONS (FOR VISUAL FEEDBACK)
    pub.publish(o) 
    try:        
        d = [ids[0,0]  , ids[1,0] , ids[2,0] , ids[3,0]]
        if ((len(ids)) == 4) & (sum(s_id[Id]) == sum(d) ): #   --- IF ALL THE REFERENCE LANDMARKS ARE AVAILABLE AND DETECTED, CONTINUE                                           
                                                           #       WE USE THE SUM TO CHECK EVERY SINGLE SEQUENCE CAUSE THE SUME 
                                                           #       OF THE ELEMENTS OF EACH ONE IS DIFFERENT FROM THE OTHERS
            cen = np.zeros((4,3))
            for i in range(4):          #  --- OBTAIN THE CENTROID FOR EACH BOUNDING BOX AND 
                                        #       FILL THE CENTROIDS ARRAY (COORDS. AND LANDMARK ID)
                pts       =  bbox[i]
                p         =  pts[0]
                x,y       =  p[:,0], p[:,1]  
                lth       =  len(x)
                cen[i,0]  =  round(sum(x)/lth)   # CENTROID COORD X 
                cen[i,1]  =  round(sum(y)/lth)   # CENTROID COORD Y
                cen[i,2]  =  ids[i]              # CENTROID ARUCO ID
                
            load(cen, Id) #LOAD THE REFERENCE POINTS FOR THE CURRENT IMAGE SOURCE IN THE SERVER            
            out           = print_detections(bgr, cen, Id)
            nm            = (c_id[Id])+'_out_from_ext_calib.png'   
            cv2.imwrite(nm, out)              #EXPORT THE IMAGE WITH THE ARUCO DETECTIONS ON IT (FOR DEBUG)             
        else:
            pass   
    except:        
        pass
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   --EX1.1--
#------------------------------------------------------------------------------------------------------------------------im4extCalib SERVICE REQUEST 
#def rim(idx):
#    rospy.wait_for_service('/extCsrv')
#    try:
#        msg = rospy.ServiceProxy('/extCsrv', im4extCalib)
#        ans = msg(idx)  #REQUEST THE RECTIFIED IMAGE FROM THE CHANNEL # idx
#        return(ans.im)  #RETURN THE DEFAULT SRV RESPONSE (sensor_msgs/Image  "im")  #INCOMING MESSAGE IN BGR8 FORMAT
    
#    except:
#        rospy.loginfo('im4extCalin service call failed') 
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   --EX1--
#-------------------------------------------------------------------------------------------------------------------------CALLBACK FOR INCOMING DATA    
def cback(m):    
        
    im   = bdg.imgmsg_to_cv2(m,"bgra8")
    
    l, r, f, b = cv2.split(im)

    
        
    if getattr(srv, 'l_f') == 0:
        check(l, 1)       #DETECT IN LEFT  (Id:1) IMAGE USING THE FUNCTIONS "check" AND "detect" AND ALSO "load"
        
    if getattr(srv, 'l_f') == 1:                     #IF L IS READY GO TO R DETECTION...
        if getattr(srv, 'r_f') == 0:
        
            check(r, 2) #DETECT IN RIGHT (Id:2) IMAGE USING THE FUNCTIONS "check" AND "detect" AND ALSO "load")
    
    if getattr(srv, 'r_f') == 1:# & getattr(srv, 'f_f') == 0: #IF R IS READY GO TO F DETECTION...
        if getattr(srv, 'f_f') == 0:
            
            check(f, 3) #DETECT IN RIGHT (Id:2) IMAGE USING THE FUNCTIONS "check" AND "detect" AND ALSO "load")
            
    if getattr(srv, 'f_f') == 1:# & getattr(srv, 'f_f') == 0: #IF R IS READY GO TO F DETECTION...
        if getattr(srv, 'b_f') == 0:
            
            check(b, 4) #DETECT IN RIGHT (Id:2) IMAGE USING THE FUNCTIONS "check" AND "detect" AND ALSO "load")            
            
            
    #----------------------------------------------------------------------------------------------------------->
    if getattr(srv, 'b_f') == 1:                 #LAST SEQUENCE IN THIS CASE, WE CHECK THE SERVER STATUS TOO HERE
        if getattr(srv, 'st') == 0:
            expo(rp, srv, srv2, vc_kc, h_vir)        #THIS LAST INSTRUCTIONS IS EXECUTED ONCE, DUE TO THE STATUS FLAGS :)         
            
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   --EX0--
#-------------------------------------------------------------------INITIALIZE VIRTUAL CAMERA MATRIX AND ITS INVERSE, THEN EXPORT INFO IN .YAML FILE
def init_vir_mtx(fov,s,m,n, virtualC_path):
   
    f     = math.radians(90-(fov/2))  
    
    h_vir = math.tan(f)*(s/2);  fax=(m/s)*h_vir;  fay=(n/s)*h_vir; ua0=(1/2)*m; va0=(1/2)*n 
    
    vc_kc = np.array([[fax, 0, ua0],[0, fay, va0],[0,0,1]]) 
    
    vc_kc_inv = inv(vc_kc)                                                                                                                                 
    
    
    data2exp = { 'virtual_cam_mtx': {'rows': 3, 'cols': 3,'data': vc_kc.tolist()} , 
                'virtual_cam_mtx_inv': {'rows': 3, 'cols': 3,'data': vc_kc_inv.tolist()} ,
                'h_vir': h_vir, 'fov':fov,'side_length': s, 'm_pixels': m, 'n_pixels': n}    

    with open(virtualC_path, 'w') as p:
        yaml.dump(data2exp, p,sort_keys=False)

    return (vc_kc, vc_kc_inv, h_vir)            
            
    

#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#--------------------------------------------------------------------------------------------------------------------------------------MAIN SEQUENCE

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='DATA FOR VIRTUAL CAMERA CONFIGURATION') 
    
    parser.add_argument('-s', '--side_virtual',              type=int, default=8000,  help='bird visible área, lenght for each side (mm)')
    parser.add_argument('-f', '--fov_virtual_cam',           type=int, default=100,    help='fov of the virtual camera in degrees')
    parser.add_argument('-m', '--m_virtual_pixels',          type=int, default=500,   help='#of virtual pixels in u direction')
    parser.add_argument('-n', '--n_virtual_pixels',          type=int, default=500,   help='#of virtual pixels in v direction')    
    parser.add_argument('-ov', '--out_path_virtual',       type=str,  default='/home/oni/oni_dir/oni_ws/src/oni_pkg/yaml_config/virtualCam.yaml',     help='path, output path4virtual mtx ') 
    parser.add_argument('-rp', '--referencePointsPath',    type=str,  default='/home/oni/oni_dir/oni_ws/src/oni_pkg/yaml_config/refPoints.yaml',      help='path, reference real points file') 

    args, unknown = parser.parse_known_args()
    
    
    fov     =  args.fov_virtual_cam                                 #VIRTUAL CAM FOV
    s       =  args.side_virtual                                    #VIRTUAL CAM -- SIDE LENGHT
    m       =  args.m_virtual_pixels                                #VIRTUAL CAM -- PIXELS IN m DIR
    n       =  args.n_virtual_pixels                                #VIRTUAL CAM -- PIXELS IN n DIR
    
    vo_path =  args.out_path_virtual                                #VIRTUAL CAM -- .YAML INFO PATH FOR EXPORTATION
    rp      =  args.referencePointsPath                             #REFERENCE POINTS FROM REAL MEASUREMENT
    
    
    #CHANGE THIS PER AN IMPORTATION FROM THE AUTOCALIB DETECTOR
    vc_kc, vc_kc_inv, h_vir  = init_vir_mtx(fov,s,m,n,vo_path)      #  1 -- INITIALIZE VIRTUAL CAMERA MATRIX AND ITS INVERSE
    
    try:
    
        bdg  = CvBridge()
        srv  = server()   #INITIALIZE SERVER FOR PIXELS FROM THE FISH CAMS
        srv2 = server()   #INITIALIZE SERVER FOR PIXELS FROM THE REAL WORLD REMAPPED FROM 3D DATA 
        #----------------------------------------------------------------------------------------------------------------------------

        rospy.init_node('arucDetector', anonymous = False)    
        rate   = rospy.Rate(30)    
        rospy.set_param('nodearucoDetector', 1)                              #  -- ENABLE FLAG FOR THE "imAutoCalibDetector" NODE
        rospy.loginfo('arucoDetectorNodeInitialized')                        #  -- REGISTER THE INITIALIZATION OF THE NODE
    
        pub =  rospy.Publisher('/arucoIm', Image, queue_size=0.005) 
        
        s   = rospy.Subscriber('/uIm', Image, cback)                   # (FUZZY CORRECTION SUBSCRIBER)   


        rospy.spin()

    except rospy.ROSInterruptException:
        pass
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||      
