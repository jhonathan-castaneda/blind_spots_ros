#!/usr/bin/env python3

#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# FIX CPU OVERLOAD WITH
# taskset --cpu-list 0,1,2 python3 5-yoloMultiTracker --source 0,1,2 
#https://github.com/mikel-brostrom/Yolov5_StrongSORT_OSNet/issues/48
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
import argparse 
import rospy
import time
import numpy as np
import cv2
import matplotlib.pyplot as plt
import argparse
from cv_bridge import CvBridge
#---------------------------------------------------#CONFIGURE GPU FOR TF
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()
gpus = tf.config.list_physical_devices('GPU')
tf.config.set_logical_device_configuration( gpus[0], [tf.config.LogicalDeviceConfiguration(memory_limit=1024)])
#------------------------------------------------------------------------
from deep_sort import preprocessing
from deep_sort import nn_matching
from deep_sort.detection import Detection
from deep_sort.tracker import Tracker
from deep_sort_pkg.tools import generate_detections as gdet

from oni_pkg.msg import track
from sensor_msgs.msg import Image
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD 
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#----------------------------------------------------------------------------------------------------------------------------------------DRAW BBOXES
"""
#for debug!!!!!!!!!!!!!!!!!
def db(trks,img):
    
    cm  =  plt.get_cmap('tab20b')                    # <<< RETRIEVE COLOR MAP PALETTE
    cl  =  [cm(i)[:3] for i in np.linspace(0,1,20)]  # <<< SELECT COLOR SPACE

    for track in trks.tracks:
        if not track.is_confirmed() or track.time_since_update >1:
            continue       
        bb  = list(track.to_tlbr())
        txt = 'id:' + str(track.track_id) 
	

        (lw,lh), bl = cv2.getTextSize(txt , cv2.FONT_HERSHEY_SIMPLEX,1,1) # <<< LABEL W H AND BASELINE
        tl = tuple(map(int,[int(bb[0]),int(bb[1])-(lh+bl)]))              # <<< TOP LEFT POINT
        tr = tuple(map(int,[int(bb[0])+lw,int(bb[1])]))                   # <<< TOP RIGHT POINT
        rd = tuple(map(int,[int(bb[0]),int(bb[1])-bl]))                   # <<< INFO READY
        
        cv2.rectangle(img, (int(bb[0]), int(bb[1])), (int(bb[2]), int(bb[3])), (255,0,0), 1)
        cv2.rectangle(img, tl, tr, (255,0,0), -1)
        cv2.putText(img, txt, rd, cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1)    
    return img
"""
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD 
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#----------------------------------------------------------------------------------------------------------------------PREPARE DATA FOR BROADCASTING
def p4b(trk, cl):   #OUTPUT FORMAT: COLS  >>> TRACK_ID , CLASS_ID, BBOX

    out = np.zeros((len(cl),6), np.uint16)
    for idx in range(len(cl)):
        bb                         = np.asarray(list((trk.tracks[idx]).to_tlbr()))
        
        out[idx,0],  out[idx,1]    = (trk.tracks[idx]).track_id    , cl[idx]                              # TRACK ID AND CLASS ID
        
        out[idx,2]                = int(bb[0])                                                                   # BBOX DATA
        out[idx,3]                = int(bb[1])                                                                   # BBOX DATA
        out[idx,4]                = int(bb[2])                                                                   # BBOX DATA
        out[idx,5]                = int(bb[3])                                                                   # BBOX DATA
       
    return out
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD 
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#----------------------------------------------------------------------------------------------------------------------PREPARE DATA FOR BROADCASTING
def odt(mi):
    
    f   =  bdg.imgmsg_to_cv2(mi,"mono8")
    f   =  cv2.cvtColor(f, cv2.COLOR_GRAY2BGR)

    #CORE/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    cl, sc, bx      =  mod.detect(f, ct, nmt)                                                          # <<< CLASSES, SCORES, BOXES
    ft              =  enc(f, bx)                                                                      # <<< GET FEATURES FOR TRACKING    
    det             =  [Detection(bbox, score, feature) for bbox, score, feature in zip(bx, sc, ft)]   # <<< STABLISH DETECTIONS
    trk.predict()                                                                                      # <<< KALMAN PREDICTION
    trk.update(det)                                                                                    # <<< KALMAN UPDATE
    sr              =  p4b(trk,cl)                                                                     # <<< PREPARE DATA TO BROADCAST
    #CORE/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    """
    #ONLY FOR DEBUG----------------------------------------2
    out7         =  db(trk,f)                                                                      # <<< DRAW BBOXES TO PLOT
    m7           =  bdg.cv2_to_imgmsg(out7, "bgr8")  
    p2.publish(m7)
    #ONLY FOR DEBUG----------------------------------------2
    """
    #print(sr)
    msg.tracks = [x for x in sr.flatten()]                                                         # <<< SERIALIZE DATA
    p.publish(msg)          

#--------------------------------------------------------------------------------------------------------------------------------------MAIN SEQUENCE  
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------------------------------------------------------ GENERAL V4L SETTINGS
if __name__ == "__main__":
    
    parser  =  argparse.ArgumentParser()    
    parser.add_argument("--yoloNamesPath",         type = str,   default = '/home/oni/oni_dir/oni_ws/src/oni_pkg/data/rna/yolo.names')     
    parser.add_argument("--yoloWeightsPath",       type = str,   default = '/home/oni/oni_dir/oni_ws/src/oni_pkg/data/rna/yolo.weights') 
    parser.add_argument("--yoloConfigPath",        type = str,   default = '/home/oni/oni_dir/oni_ws/src/oni_pkg/data/rna/yolo.cfg') 
    parser.add_argument("--confidenceThreshold",   type = float, default = 0.7) 
    parser.add_argument("--nmsThreshold",          type = float, default = 0.4) 
    parser.add_argument("--trackerModelPath",      type = str,   default = '/home/oni/oni_dir/oni_ws/src/oni_pkg/data/deep_sort/mars-small128.pb')     
    parser.add_argument("--maxCosineDistance",     type = float, default = 0.5) 
    args, unknown = parser.parse_known_args()
    print(args)
    
    #YOLO INITIALIZATION-----------------------------------------------------------------------------------------------------------------------------
    #------------------------------------------------------------------------------------------------------------------------------------------------
    cn  = [c.strip() for c in open(args.yoloNamesPath).readlines()]  #  <<< CLASSES NAMES FOR THE DETECTED OBJECTS
    ct  = args.confidenceThreshold                                   #  <<< YOLO CONFIDENCE THRESHOLD
    nmt = args.nmsThreshold                                          #  <<< YOLO NON MAX SUPPRESSION THRESHOLD
    #------------------------------------------------------------------------------------------------------------------------------------------------
    n = cv2.dnn.readNet(args.yoloWeightsPath, args.yoloConfigPath)
    n.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    n.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)              #  <<< SELECT GPU && FP16 BOOST
    mod = cv2.dnn_DetectionModel(n)                                  #  <<< YOLO MODEL INSTANCE
    mod.setInputParams(size=(416, 416), scale=1/255, swapRB=True)    #  <<< YOLO MODEL INSTANCE SETTINGS
    
    #TRACKER INITIALIZATION--------------------------------------------------------------------------------------------------------------------------
    #------------------------------------------------------------------------------------------------------------------------------------------------
    mcd = args.maxCosineDistance                                                   # <<< MAXIMUM COSINE DISTANCE FOR TRACKER 
    enc = gdet.create_box_encoder(args.trackerModelPath , batch_size=1)            # <<< FEATURE EXTRACTOR MODEL - BASE TRACKER MODEL
    trk = Tracker(nn_matching.NearestNeighborDistanceMetric('cosine', mcd, None))  # <<< TRACKER OBJECT INSTANCE

    #///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    #///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    try:
        
        rospy.init_node('yoloTracker', anonymous = False)                
        rate   = rospy.Rate(30)                                             
        rospy.set_param('nodeyoloTracker', 1)                             
        rospy.loginfo('yoloTrackerNodeInitialized')                                                                        
        p     = rospy.Publisher('/yolo', track, queue_size = 2)    
        
        
        #ONLY FOR DEBUG----------------------------------------3
        #p2     = rospy.Publisher('/tracking', Image, queue_size = 0.0005)    
        #ONLY FOR DEBUG----------------------------------------3
        
        s     = rospy.Subscriber('/4Im', Image, odt)                   # (FUZZY CORRECTION SUBSCRIBER)   
        msg   = track()                                                      
        bdg   = CvBridge()
        
        rospy.spin()       

    except rospy.ROSInterruptException:
        pass
