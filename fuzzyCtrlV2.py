#!/usr/bin/env python3


# PENDING TO MODIFY FOR 640 X 480 RESOLUTION !!! LINE 46 IN fuzzCtrlV2

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

#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#---------------------------------------------------------------------------------------------------------------------------V HISTOGRAM CALCULATIONS  
def hist(frame):
    h, s, v = cv2.split(frame)
    hist_v  = cv2.calcHist([v],[0],None,[256],[0,256])
    return(hist_v)
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#-----------------------------------------------------------------------------------------------------------------------------MEASURE EXPOSURE ERROR
def exp_e(hist_v):

    exp_num, exp_den = 0 , 0
    
    for i in range(255):
        exp_num += i*( round(hist_v[i,0]) )
        exp_den += round(hist_v[i,0])
        
    if exp_den ==0:
        exp     = (1/255)*(exp_num/0.1)
    else:
        exp     = (1/255)*(exp_num/exp_den)
    exp_err = (-0.5)+exp
    return(exp_err)
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||  
#-------------------------------------------------------------------------------------------------------------------------GET_FUZZY_CORRECTION_VALUE
def get_correction(a_ctrl, g_ctrl, exp_err, exp_cmos):
    
    e_controller                     =  ctrl.ControlSystemSimulation(a_ctrl)
    e_controller.input['exp_err']    =  exp_err
    e_controller.compute()
    tunner_e                         =  10 # PENDING TO MODIFY FOR 640 X 480 RESOLUTION !!! LINE 46 IN fuzzCtrlV2

    g_controller                     =  ctrl.ControlSystemSimulation(g_ctrl)
    g_controller.input['exp_err']    =  exp_err
    g_controller.input['cmos_stat']  =  exp_cmos
    g_controller.compute()    
    tunner_g                         =  1
     
    defuzzy_exp                      =  (e_controller.output['delta_exp'])  *  tunner_e
    defuzzy_gain                     =  (g_controller.output['delta_gain']) *  tunner_g
    
    return defuzzy_exp, defuzzy_gain
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#---------------------------------------------------------------------------------------------------------------INITIALIZE_FUZZY_EXPOSURE_CONTROLLER
def ini_ap_fuzzy():
    #TAGS---------------------------------------------------------------------------
    exp_err    = ctrl.Antecedent(np.arange(-0.5,0.5,0.01), 'exp_err')    
    delta_exp  = ctrl.Consequent(np.arange(-10,10,0.1),  'delta_exp')    
    #ANTECEDENT1||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    exp_err['vn'] = fuzz.trapmf(exp_err.universe,    [-0.5, -0.5,  -0.3,  -0.2])
    exp_err['n']  =  fuzz.trimf(exp_err.universe,    [-0.3,   -0.2, -0.12])    
    exp_err['c']  =  fuzz.trimf(exp_err.universe,    [-0.12,  0,   0.12])
    exp_err['p']  =  fuzz.trimf(exp_err.universe,    [ 0.12,  0.2,   0.3 ])    
    exp_err['vp'] = fuzz.trapmf(exp_err.universe,    [ 0.2,  0.3,  0.5, 0.5])
    #CONSEQUENT---------------------------------------------------------------------
    delta_exp['vn'] = fuzz.trapmf(delta_exp.universe,  [-40, -40, -7, -5])
    delta_exp['n']  =  fuzz.trimf(delta_exp.universe,  [-5, -3, -1])
    delta_exp['c']  =  fuzz.trimf(delta_exp.universe,  [-0.001 , 0, 0.001])
    delta_exp['p']  =  fuzz.trimf(delta_exp.universe,  [1,  3,  5])
    delta_exp['vp'] = fuzz.trapmf(delta_exp.universe,  [5, 7, 40, 40])
    #RULES--------------------------------------------------------------------------
    rule1 = ctrl.Rule(exp_err['vn'], delta_exp['vp'])
    rule2 = ctrl.Rule(exp_err['n'],  delta_exp['p'])
    rule3 = ctrl.Rule(exp_err['c'],  delta_exp['c'])
    rule4 = ctrl.Rule(exp_err['p'],  delta_exp['n'])
    rule5 = ctrl.Rule(exp_err['vp'], delta_exp['vn'])
    
    #CONTROLLER---------------------------------------------------------------------
    aperture_ctrl     = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])
    return aperture_ctrl
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#-------------------------------------------------------------------------------------------------------------------INITIALIZE_FUZZY_GAIN_CONTROLLER    
def ini_g_fuzzy():
    #TAGS---------------------------------------------------------------------------
    exp_err    = ctrl.Antecedent(np.arange(-0.5,0.5,0.01), 'exp_err')    
    cmos_stat  = ctrl.Antecedent(np.arange(0,8190,10),   'cmos_stat')   
    delta_gain = ctrl.Consequent(np.arange(-20,20,1),    'delta_gain')
    #ANTECEDENT1||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    exp_err['vn'] = fuzz.trapmf(exp_err.universe,     [-0.5, -0.5,  -0.3,  -0.2])
    exp_err['n']  =  fuzz.trimf(exp_err.universe,     [-0.3,   -0.2, -0.1])    
    exp_err['c']  =  fuzz.trimf(exp_err.universe,     [-0.1,  0,   0.1])
    exp_err['p']  =  fuzz.trimf(exp_err.universe,     [ 0.1,  0.2,   0.3 ])    
    exp_err['vp'] = fuzz.trapmf(exp_err.universe,     [ 0.2,  0.3,  0.5, 0.5])
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
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#-------------------------------------------------------------------------------------------------------------------FUZZY-CTRL-NODE

#WHEN USING MSG_FILTERS THIS IS THE CALLBACK DEPLOYED WHEN THE MSGS ARE SYNCHRONIZED-----------------------------------------------
#----------------------------------------------------------------------------------------------------CALLBACK (PROCESSING SEQUENCE)
def return_corr(msg_l, msg_r, msg_f, msg_s):
    
    img_l = bridge.imgmsg_to_cv2(msg_l,"bgr8") 
    img_r = bridge.imgmsg_to_cv2(msg_r,"bgr8")
    img_f = bridge.imgmsg_to_cv2(msg_f,"bgr8")
    #img_b = bridge.imgmsg_to_cv2(msg_b,"bgr8")
    
   
    g_l, g_r, g_f, exp_l, exp_r, exp_f  =  msg_s.gain_l, msg_s.gain_r, msg_s.gain_f, msg_s.exp_l, msg_s.exp_r, msg_s.exp_f

    hist_l, hist_r, hist_f              =  hist(img_l) , hist(img_r) , hist(img_f)

    exp_err_l,  exp_err_r, exp_err_f  =  exp_e(hist_l)  , exp_e(hist_r)  , exp_e(hist_f) 
    
    defuzzy_exp_l, defuzzy_gain_l    =  get_correction(a_ctrl, g_ctrl, exp_err_l, exp_l)  #GET FUZZY CORRECTION FOR LEFT IMG
    defuzzy_exp_r, defuzzy_gain_r    =  get_correction(a_ctrl, g_ctrl, exp_err_r, exp_r)  #GET FUZZY CORRECTION FOR RIGHT IMG
    defuzzy_exp_f, defuzzy_gain_f    =  get_correction(a_ctrl, g_ctrl, exp_err_f, exp_f)  #GET FUZZY CORRECTION FOR FRONT IMG
    #defuzzy_exp_b, defuzzy_gain_b    =  get_correction(a_ctrl, g_ctrl, exp_err_b, exp_b)  #GET FUZZY CORRECTION FOR BACK IMG
    
    fuzzy_msg.corr_e_l,  fuzzy_msg.corr_e_r,  fuzzy_msg.corr_e_f   = round(exp_l + round(defuzzy_exp_l))     ,  round(exp_r + round(defuzzy_exp_r))    ,  round(exp_f + round(defuzzy_exp_f)) 
    fuzzy_msg.corr_g_l,  fuzzy_msg.corr_g_r,  fuzzy_msg.corr_g_f   = round(g_l   + round(defuzzy_gain_l))    ,  round(g_r   + round(defuzzy_gain_r))   ,  round(g_f   + round(defuzzy_gain_f))
        
    pub_c.publish(fuzzy_msg)
    #print(fuzzy_msg)
    
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#---------------------------------------------------------------------------------------------------------------------MAIN_SEQUENCE
if __name__ == '__main__':
    
    try:
        
        bridge = CvBridge()
        
        rospy.init_node('fuzz_img_ctrl',anonymous = False)    # 1 -- INITIALIZE CONTROLLER NODE
        rate       = rospy.Rate(30)                           # 2 -- SET REFRESH RATIO
        
        rospy.set_param('nodefuzzCtrl', 1)                    # 3 -- ENABLE FLAG FOR THE "fuzzCtrl" NODE
        rospy.loginfo('fuzzyCtrlNodeInitialized')              # 4 -- REGISTER THE INITIALIZATION OF THE NODE
        

        fuzzy_msg  = fuzzy_corr()                             # 5 -- INITIALIZE CONTROLLER MSG OBJECT    
        
        a_ctrl  =  ini_ap_fuzzy()                             # 6 -- INITIALIZE EXPO AND GAIN CONTROLLERS
        g_ctrl  =  ini_g_fuzzy() 
        
        sub_l = message_filters.Subscriber('/d_img_l', Image) # 7 -- SUBSCRIBE TO CONTROLLED TOPICS
        sub_r = message_filters.Subscriber('/d_img_r', Image) 
        sub_f = message_filters.Subscriber('/d_img_f', Image)
        #sub_b = message_filters.Subscriber('/d_img_b', Image)
        
        sub_s = message_filters.Subscriber('/pub_cams_status', cams_status) 
        
        pub_c = rospy.Publisher('/fuzzy_ctrl', fuzzy_corr, queue_size=0.0005) #8 -- INITIALIZE PUBLISHER TO RETURN THE CONTROL CORRECTIONS
        
        sync  = message_filters.ApproximateTimeSynchronizer([sub_l, sub_r, sub_f, sub_s], queue_size=1, slop=0.0005) #7 SYNC RETRIEVED DATA      
        #IMAGE MESSAGES SHOULD ARRIVE EACH 0.033 S FOR 30 FPS, THEN THE MAX WAITING RANGE FOR SYNC IS 0.015 (HALF BROADCASTING PERIOD)
        
        sync.registerCallback(return_corr)  #8  DEFINE DATA HANDLER
        
        rospy.spin()                        #9 INITIALIZE PROCESSING LOOP
        
    except rospy.ROSInterruptException:
        pass    
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||       
   
