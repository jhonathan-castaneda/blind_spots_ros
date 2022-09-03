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



class Laser2PC():
    
    def __init__(self):
        self.laserProj  = LaserProjection()
        self.pcPub      = rospy.Publisher("/laserPCL", PointCloud2, queue_size=1)
        self.laserSub   = rospy.Subscriber("/scan", LaserScan, self.laserCallback)
        self.detections = np.array([], dtype=np.float32)
        self.detectPub  = rospy.Publisher("/detections", Float32MultiArray, queue_size=1)

    def laserCallback(self,data):
        cloud_out    =  self.laserProj.projectLaser(data)
        Xg           =  pc2c.read_points(cloud_out, skip_nans=True, field_names=("x","y","z"))
        cloud_points =  np.empty((cloud_out.width,2))
        a            =  0
        
        for p in Xg:
            cloud_points[a,0] = p[0]
            cloud_points[a,1] = p[1]
            a=a+1
        
        #CLUSTERS PCL-----------------------------
        dbscan   = DBSCAN()
        clusters = dbscan.fit_predict(cloud_points)
        
        #OBTAIN DETECTIONS------------------------
        cluster_ids = np.unique(clusters)
        detections  = np.zeros((cluster_ids.size, 2), dtype=np.float32)
        br          = tf.TransformBroadcaster()
        
        
        for ci in cluster_ids:
            if ci != -1:
                detect              =  np.mean(cloud_points[clusters == ci, :], axis=0)
                detections[ci, :]   =  detect
                br.sendTransform((detect[0], detect[1], 0),
                                 tf.transformations.quaternion_from_euler(0,0,0),
                                 rospy.Time.now(),
                                 "detect_%d" % ci,
                                 "laser")
                        
                        
        self.detections = detections
        detect_msg      = Float32MultiArray()
        detect_msg.data = [x for x in detections.flatten()]
        rospy.loginfo(detect_msg.data)
        
        detect_msg.layout.dim.append(MultiArrayDimension())
        detect_msg.layout.dim.append(MultiArrayDimension())
        detect_msg.layout.dim[0].label   = "height"
        detect_msg.layout.dim[1].label   = "width"
        detect_msg.layout.dim[0].size    = cluster_ids.size
        detect_msg.layout.dim[1].size    = 2
        detect_msg.layout.dim[0].stride  = cluster_ids.size*2
        detect_msg.layout.dim[1].stride  = 2
        detect_msg.layout.data_offset    = 0
       
        rospy.loginfo("Detect message")
        self.detectPub.publish(detect_msg)
        rospy.loginfo("Detect message")
        self.pcPub.publish(cloud_out)
        rospy.loginfo("Detection Array Start")
        rospy.loginfo(detections)
        rospy.loginfo("DEtection Array Ends")
        

if __name__ == "__main__":

    rospy.init_node('laser2PC', anonymous=False)
    l2pc = Laser2PC()
    rospy.spin()