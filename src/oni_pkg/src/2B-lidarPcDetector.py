#!/usr/bin/env python3

import rospy
import tf

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

from sensor_msgs.msg import PointCloud2 
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from rospy_tutorials.msg import Floats

from laser_geometry import LaserProjection
from sensor_msgs import point_cloud2 as pc2c
import numpy as np
from sklearn.cluster import DBSCAN


import random
import skimage
import matplotlib.pyplot as plt

###################################################################################################################
###################################################################################################################
###################################################################################################################

def color_dic(nclusters):

    color=["#"+''.join([random.choice('0123456789ABCDEF') for i in range(6)])
           for j in range(nclusters)]
    dic={}
    dic[-1] = '#000000'    #INVALID COLOR (BLACK FOR CLUSTERS WITH ID = -1)
        
    for i in range(0,nclusters-1):   #ASSIGN COLORS TO THE DICTIONARY IN THE VALID INTERVAL [0, MAXid]
        
        dic[i] = color[i]           
             
    return(dic)
###################################################################################################################
###################################################################################################################
###################################################################################################################

#-----------------------------------------------------------------------------------------------------------------------------------CALLBACK-METHOD1
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------------------------------------------------------DETECT IN POINT CLOUD
def cb(ms):
    
    co  =  laserProj.projectLaser(ms)                                       # <<< CONVERT COORDINATED SYSTEM
    Xg  =  pc2c.read_points(co, skip_nans=True, field_names=("x","y","z"))  # <<< READ PC
    cp  =  np.empty((co.width,2))                                           # <<< CLOUD POINTS ARRAY
    a   =  0
        
    for p in Xg:
        cp[a,0], cp[a,1] =  p[0] , p[1]  ;  a=a+1       # <<< FILL OUT POINT CLOUD ARRAY          
        
    clu = db.fit_predict(cp)                            # <<< PERFORM CLUSTERING

    ###################################################################################################################
    ###################################################################################################################
    ###################################################################################################################
    #clusters = DBSCAN(eps=0.2, min_samples=20).fit(cp)
    #(clusters.labels_)
    #nclusters= max(clusters.labels_) + 2   # MAX PLUS THE COLOR FOR ZERO VALUE AND -1 VALUE


    ###################################################################################################################
    ###################################################################################################################
    ################################################################################################################### 
    """
    dic     = color_dic(nclusters)

    palette = []
    for i in range(len(cp[:,0])):
        palette.append(  dic[ ((clusters.labels_) [i] ) ] )


    plt.figure(figsize=(17,17))   
    plt.scatter(cp[:,0], cp[:,1], c = (palette), s=7)
    #print(max(clusters.labels_))
    """ ###################################################################################################################
    ###################################################################################################################
    ################################################################################################################### 

    
    
    
    
    
    cid = np.unique(clu)                                # <<< ASSIGN IDS TO ALL THE CLUSTERS
    dtt = np.zeros((cid.size, 2), dtype=np.float32)     # <<< DETECTIONS OUTPUT ARRAY
    
    ############################################################################################
    #ONLY FOR DEBUG#############################################################################
    #print(cp)
    
    #with open('/home/oni/oni_dir/oni_ws/src/oni_pkg/data/lidar_PC2.npy', 'wb') as m5:
    #    np.save(m5, cp)
    #rospy.loginfo('CLOUD SAMPLEEXPORTED')
    
    #time.sleep(100000)
    ############################################################################################
    ############################################################################################
    
    
    
    
    for q in cid:                                       # <<< FIND MEAN PER CLUSTER AND SEND TO TF2
        if q != -1:
            o1         =  np.mean(cp[clu == q, :], axis=0)
            dtt[q, :]  =  o1
            br.sendTransform((o1[0], o1[1], 0), tf.transformations.quaternion_from_euler(0,0,0), rospy.Time.now(),"detect_%d" % q, "laser")
                                                
    ldt.data        = [x for x in dtt.flatten()]        # <<< SERIALIZE DETECTIONS     
    ldt.layout.dim.append(MultiArrayDimension())
    ldt.layout.dim.append(MultiArrayDimension())
    ldt.layout.dim[0].label   = "height"
    ldt.layout.dim[1].label   = "width"
    ldt.layout.dim[0].size    = cid.size
    ldt.layout.dim[1].size    = 2    
    ldt.layout.dim[0].stride  = (cid.size)*2
    ldt.layout.dim[1].stride  = 2
    ldt.layout.data_offset    = 0   
    p1.publish(ldt)                                     # <<< PUBLISH DETECTIONS
#--------------------------------------------------------------------------------------------------------------------------------------MAIN SEQUENCE  
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#---------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    
    try:
        rospy.init_node('laserDetector', anonymous=False)
        rospy.loginfo('laserDetectorNodeInitialized')            

        p1         =  rospy.Publisher( "/lidar_det", Float32MultiArray, queue_size=1)
        laserSub   =  rospy.Subscriber("/scan",       LaserScan, cb)
        laserProj  =  LaserProjection()                # <<< INITIALIZE LASER PROJECTION MESSAGE
        db         =  DBSCAN()                         # <<< INITIALIZE DBSCAN OBJECT
        br         =  tf.TransformBroadcaster()        # <<< INITIALIZE TF2 OBJECT
        ldt        =  Float32MultiArray()              # <<< INITIALIZE LASER DETECTIONS ARRAY 
   
        rospy.spin()

    except rospy.ROSInterruptException:
          
        pass
