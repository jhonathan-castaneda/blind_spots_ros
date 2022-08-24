#!/usr/bin/env python3


# PENDING TO MODIFY FOR 640 X 480 RESOLUTION !!! LINE 46 IN fuzzCtrlV2

#VERSION_3 VARIABLES OPTIMIZED

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||METHOD   
#--------------------------------------------------------------------------------------------------------------------------CALLBACK
def sp(m):
    
    im           = bdg.imgmsg_to_cv2(m,"bgra8")
    l, r, f, b   = cv2.split(im)

    ml = bdg.cv2_to_imgmsg(l, "mono8")
    mr = bdg.cv2_to_imgmsg(r, "mono8")
    mf = bdg.cv2_to_imgmsg(f, "mono8")
    mb = bdg.cv2_to_imgmsg(b, "mono8")
    
    pl.publish(ml)
    pr.publish(mr)
    pf.publish(mf)
    pb.publish(mb)
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#---------------------------------------------------------------------------------------------------------------------MAIN_SEQUENCE
if __name__ == '__main__':
    
    try:
        
        bdg = CvBridge()
        
        rospy.init_node('imSplitter',anonymous = False)        # 1 -- INITIALIZE CONTROLLER NODE
        rate       = rospy.Rate(30)                           # 2 -- SET REFRESH RATIO
        
        rospy.set_param('imSplitter', 1)                 # 3 -- ENABLE FLAG FOR THE "fuzzCtrl" NODE
        rospy.loginfo('imSplitterNodeInitialized')             # 4 -- REGISTER THE INITIALIZATION OF THE NODE
        

        s   = rospy.Subscriber('/uIm', Image, sp) 
        
        pl = rospy.Publisher('/uIm_l', Image, queue_size=0.00005) 
        pr = rospy.Publisher('/uIm_r', Image, queue_size=0.00005) 
        pf = rospy.Publisher('/uIm_f', Image, queue_size=0.00005) 
        pb = rospy.Publisher('/uIm_b', Image, queue_size=0.00005) 
         

        rospy.spin()            
        
    except rospy.ROSInterruptException:
        pass    
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||       
    