#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from oni_pkg.msg import cams_status
from oni_pkg.msg import fuzzy_corr
import message_filters
import skfuzzy as fuzz
from skfuzzy import control as ctrl
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||    
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#----------------------------------------------------------------------------------------------------------------------CALCULATIONS  
def hist(frame):
    h, s, v = cv2.split(frame)
    hist_v  = cv2.calcHist([v],[0],None,[256],[0,256])
    return(hist_v)
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#--------------------------------------------------------------------------------------------------------------------EXPOSURE_ERROR
def exp_e(hist_v):

    exp_num = 0
    exp_den = 0
    for i in range(255):
        exp_num += i*( round(hist_v[i,0]) )
        exp_den += round(hist_v[i,0])
        
    if exp_den ==0:
        exp     = (1/255)*(exp_num/0.1)
    else:
        exp     = (1/255)*(exp_num/exp_den)
    exp_err = (-0.5)+exp
    return(exp_err)
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#----------------------------------------------------------------------------------------------INITIALIZE_FUZZY_EXPOSURE_CONTROLLER
def ini_ap_fuzzy():
    #TAGS---------------------------------------------------------------------------
    exp_err    = ctrl.Antecedent(np.arange(-0.5,0.5,0.01), 'exp_err')    
    delta_exp  = ctrl.Consequent(np.arange(-10,10,0.1),  'delta_exp')    
    #ANTECEDENT1||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    exp_err['vn'] = fuzz.trapmf(exp_err.universe,    [-0.5, -0.5,  -0.3,  -0.2])
    exp_err['n'] =  fuzz.trimf(exp_err.universe,     [-0.3,   -0.2, -0.12])    
    exp_err['c'] =  fuzz.trimf(exp_err.universe,     [-0.12,  0,   0.12])
    exp_err['p'] =  fuzz.trimf(exp_err.universe,     [ 0.12,  0.2,   0.3 ])    
    exp_err['vp'] = fuzz.trapmf(exp_err.universe,    [ 0.2,  0.3,  0.5, 0.5])
    #CONSEQUENT---------------------------------------------------------------------1
    delta_exp['vn'] = fuzz.trapmf(delta_exp.universe, [-40, -40, -7, -5])
    delta_exp['n'] =  fuzz.trimf(delta_exp.universe,  [-5, -3, -1])
    delta_exp['c'] =  fuzz.trimf(delta_exp.universe,  [-0.001 , 0, 0.001])
    delta_exp['p'] =  fuzz.trimf(delta_exp.universe,  [1,  3,  5])
    delta_exp['vp'] = fuzz.trapmf(delta_exp.universe, [5, 7, 40, 40])
    #RULES--------------------------------------------------------------------------
    rule1 = ctrl.Rule(exp_err['vn'], delta_exp['vp'])
    rule2 = ctrl.Rule(exp_err['n'],  delta_exp['p'])
    rule3 = ctrl.Rule(exp_err['c'],  delta_exp['c'])
    rule4 = ctrl.Rule(exp_err['p'],  delta_exp['n'])
    rule5 = ctrl.Rule(exp_err['vp'], delta_exp['vn'])
    
    #CONTROLLER---------------------------------------------------------------------
    aperture_ctrl     = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])
    return aperture_ctrl
 
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------IINITIALIZE_FUZZY_GAIN_CONTROLLER    
def ini_g_fuzzy():
    #TAGS---------------------------------------------------------------------------
    exp_err    = ctrl.Antecedent(np.arange(-0.5,0.5,0.01), 'exp_err')    
    cmos_stat  = ctrl.Antecedent(np.arange(0,8190,10),   'cmos_stat')   
    delta_gain = ctrl.Consequent(np.arange(-20,20,1),    'delta_gain')
    #ANTECEDENT1||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    exp_err['vn'] = fuzz.trapmf(exp_err.universe,    [-0.5, -0.5,  -0.3,  -0.2])
    exp_err['n'] =  fuzz.trimf(exp_err.universe,     [-0.3,   -0.2, -0.1])    
    exp_err['c'] =  fuzz.trimf(exp_err.universe,     [-0.1,  0,   0.1])
    exp_err['p'] =  fuzz.trimf(exp_err.universe,     [ 0.1,  0.2,   0.3 ])    
    exp_err['vp'] = fuzz.trapmf(exp_err.universe,    [ 0.2,  0.3,  0.5, 0.5])
    #ANTECEDENT2||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    cmos_stat['l'] = fuzz.trapmf(cmos_stat.universe,    [0, 0,  2047,  4094])
    cmos_stat['m'] =  fuzz.trimf(cmos_stat.universe,     [2047,  4094,   6141])
    cmos_stat['h'] = fuzz.trapmf(cmos_stat.universe,    [4094,  6141,  8190, 8190])   
   
    #CONSEQUENT---------------------------------------------------------------------
    delta_gain['vn'] = fuzz.trapmf(delta_gain.universe,  [-10, -10, -7, -5])
    delta_gain['n']  =  fuzz.trimf(delta_gain.universe,  [-5, -3, -1])
    delta_gain['c']  =  fuzz.trimf(delta_gain.universe,  [-0.007 , 0, 0.007])
    delta_gain['p']  =  fuzz.trimf(delta_gain.universe,  [1,  3,  5])
    delta_gain['vp'] = fuzz.trapmf(delta_gain.universe,  [5, 7, 10, 10])    
    #RULES--------------------------------------------------------------------------
    rule1 = ctrl.Rule(exp_err['vn'] & cmos_stat['l'], delta_gain['c'])#nog
    rule2 = ctrl.Rule(exp_err['n']  & cmos_stat['l'], delta_gain['c'])#nog
    rule3 = ctrl.Rule(exp_err['c']  & cmos_stat['l'], delta_gain['c'])
    rule4 = ctrl.Rule(exp_err['p']  & cmos_stat['l'], delta_gain['n'])
    rule5 = ctrl.Rule(exp_err['vp'] & cmos_stat['l'], delta_gain['vn'])
    
    rule6 = ctrl.Rule(exp_err['vn'] & cmos_stat['m'], delta_gain['c'])#nog
    rule7 = ctrl.Rule(exp_err['n']  & cmos_stat['m'], delta_gain['c'])#nog
    rule8 = ctrl.Rule(exp_err['c']  & cmos_stat['m'], delta_gain['c'])#nog
    rule9 = ctrl.Rule(exp_err['p']  & cmos_stat['m'], delta_gain['n'])
    rule10 = ctrl.Rule(exp_err['vp'] & cmos_stat['m'], delta_gain['vn'])

    rule11 = ctrl.Rule(exp_err['vn'] & cmos_stat['h'], delta_gain['vp'])
    rule12 = ctrl.Rule(exp_err['n']  & cmos_stat['h'], delta_gain['p'])
    rule13 = ctrl.Rule(exp_err['c']  & cmos_stat['h'], delta_gain['c'])
    rule14 = ctrl.Rule(exp_err['p']  & cmos_stat['h'], delta_gain['n'])
    rule15 = ctrl.Rule(exp_err['vp'] & cmos_stat['h'], delta_gain['vn'])
    
    #CONTROLLER---------------------------------------------------------------------
    gain_ctrl     = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14,rule15])
    return gain_ctrl
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#--------------------------------------------------------------------------------------------------------------GET_CORRECTION_VALUE
def get_correction(aperture_ctrl, gain_ctrl, exp_err, exp_cmos):
    
    e_controller                     =  ctrl.ControlSystemSimulation(aperture_ctrl)
    e_controller.input['exp_err']    =  exp_err
    e_controller.compute()
    tunner_e                         =  10

    
    g_controller                     =  ctrl.ControlSystemSimulation(gain_ctrl)
    g_controller.input['exp_err']    =  exp_err
    g_controller.input['cmos_stat']  =  exp_cmos
    
    g_controller.compute()    
    
    tunner_g                         =  1
    
    
    defuzzy_exp                      =  (e_controller.output['delta_exp'])  *  tunner_e
    defuzzy_gain                     =  (g_controller.output['delta_gain']) *  tunner_g
    
    return defuzzy_exp, defuzzy_gain
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||    
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#----------------------------------------------------------------------------------------------------------------------ROS_LISTENER

#WHEN USING MSG_FILTERS THIS IS THE CALLBACK DEPLOYED WHEN THE MSGS ARE SYNCHRONIZED-----------------------------------------------
#---------------------------------------------------------------------------------------------------------------PROCESSING SEQUENCE
def return_corr(msg_l, msg_r, msg_s):
    
    bridge = CvBridge()
    
    img_l = bridge.imgmsg_to_cv2(msg_l,"bgr8") 
    img_r = bridge.imgmsg_to_cv2(msg_r,"bgr8")
   
    g_l, g_r, exp_l, exp_r =  msg_s.gain_l, msg_s.gain_r, msg_s.exp_l, msg_s.exp_r

    hist_l, hist_r         =  hist(img_l) , hist(img_r)

    exp_err_l,  exp_err_r  =  exp_e(hist_l)  , exp_e(hist_r)   
    
    defuzzy_exp_l, defuzzy_gain_l    =  get_correction(aperture_ctrl, gain_ctrl, exp_err_l, exp_l)
    defuzzy_exp_r, defuzzy_gain_r    =  get_correction(aperture_ctrl, gain_ctrl, exp_err_r, exp_r)
    
    fuzzy_msg.corr_e_l  ,   fuzzy_msg.corr_e_r   = round(exp_l + round(defuzzy_exp_l))     ,  round(exp_r + round(defuzzy_exp_r)) 
    fuzzy_msg.corr_g_l  ,   fuzzy_msg.corr_g_r   = round(g_l   + round(defuzzy_gain_l))    ,  round(g_r   + round(defuzzy_gain_r))
    
    pub_c.publish(fuzzy_msg)
    print(fuzzy_msg)
    
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#---------------------------------------------------------------------------------------------------------------------MAIN_SEQUENCE
if __name__ == '__main__':
    
    try:
        
        rospy.init_node('fuzz_img_ctrl',anonymous = False)    #1 INITIALIZE CONTROLLER NODE
        rate       = rospy.Rate(30)      
        fuzzy_msg  = fuzzy_corr()                             #2 INITIALIZE CONTROLLER MSG OBJECT                           
        
        sub_l = message_filters.Subscriber('/d_img_l', Image) #3 SUBSCRIBE TO CONTROLLED TOPICS
        sub_r = message_filters.Subscriber('/d_img_r', Image) 
        sub_s = message_filters.Subscriber('/pub_cams_status', cams_status) 
        
        pub_c = rospy.Publisher('/fuzzy_ctrl', fuzzy_corr, queue_size=1) #4 INITIALIZE PUBLISHER TO RETURN THE CONTROL CORRECTIONS
        
        
        aperture_ctrl  =  ini_ap_fuzzy() #5 INITIALIZE EXPO CONTROLLERS
        gain_ctrl      =  ini_g_fuzzy()  #6 INITIALIZE GAIN CONTROLLERS
        
        sync  = message_filters.ApproximateTimeSynchronizer([sub_l, sub_r, sub_s], queue_size=30, slop=0.015) #7 SYNC RETRIEVED DATA      
        #IMAGE MESSAGES SHOULD ARRIVE EACH 0.033 S FOR 30 FPS, THEN THE MAX WAITING RANGE FOR SYNC IS 0.015 (HALF BROADCASTING PERIOD)
        
        sync.registerCallback(return_corr)  #8  DEFINE DATA HANDLER
        
        rospy.spin()                       #9 INITIALIZE PROCESSING LOOP
        
    except rospy.ROSInterruptException:
        pass    
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||  
