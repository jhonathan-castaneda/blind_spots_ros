#!/usr/bin/env python3

import rospy
import time
import numpy as np
import cv2
import matplotlib.pyplot as plt
import argparse

import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()

from deep_sort import preprocessing
from deep_sort import nn_matching
from deep_sort.detection import Detection
from deep_sort.tracker import Tracker
from deep_sort_pkg.tools import generate_detections as gdet

from oni_pkg.msg import track

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
#--------------------------------------------------------------------------------------------------------------------------------------MAIN SEQUENCE  
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------------------------------------------------------ GENERAL V4L SETTINGS
if __name__ == "__main__":
    
    parser  =  argparse.ArgumentParser()    
    parser.add_argument("--yoloNamesPath",         type = str,   default = '/home/oni/oni_dir/oni_ws/src/oni_pkg/data/rna/yolo.names')     
    parser.add_argument("--yoloWeightsPath",       type = str,   default = '/home/oni/oni_dir/oni_ws/src/oni_pkg/data/rna/yolo.weights') 
    parser.add_argument("--yoloConfigPath",        type = str,   default = '/home/oni/oni_dir/oni_ws/src/oni_pkg/data/rna/yolo.cfg') 
    parser.add_argument("--confidenceThreshold",   type = float, default = 0.3) 
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

    l = cv2.VideoCapture('in/9.mp4') # <<<VIDEO SOURCE

    #///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    #///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    #///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    rospy.init_node('yoloTracker', anonymous = False)                
    rate   = rospy.Rate(30)                                             
    rospy.set_param('nodeyoloTracker', 1)                             
    rospy.loginfo('yoloTrackerNodeInitialized')                                                                        
    p     = rospy.Publisher('/yolo', track, queue_size = 2)          
    msg   = track()                                                      
    
    
    #ec    = np.array([ [1] ],np.int16) #EMPTY CASE TEMPLATE.... WHEN WE DONT HAVE ANY DETECTION
    
    while not rospy.is_shutdown():

        rf, f = l.read()
        if not rf :
            print('completed')
            break
            
        #CORE/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        cl, sc, bx      =  mod.detect(f, ct, nmt)                                                          # <<< CLASSES, SCORES, BOXES
        ft              =  enc(f, bx)                                                                      # <<< GET FEATURES FOR TRACKING    
        det             =  [Detection(bbox, score, feature) for bbox, score, feature in zip(bx, sc, ft)]   # <<< STABLISH DETECTIONS
        trk.predict()                                                                                      # <<< KALMAN PREDICTION
        trk.update(det)                                                                                    # <<< KALMAN UPDATE
        sr              =  p4b(trk,cl)                                                                     # <<< PREPARE DATA TO BROADCAST
        
        
        print(sr)
        msg.tracks = [x for x in sr.flatten()]                                                         # <<< SERIALIZE DATA
        p.publish(msg)          
"""
        try:
            #ONLY FOR DEBUG
            print(sr)
            msg.tracks = [x for x in sr.flatten()]                                                         # <<< SERIALIZE DATA
            p.publish(msg)                                                                                 # <<< BROADCAST
            
        except:      
             
            pass
        

        if rospy.is_shutdown():
            l.release
"""