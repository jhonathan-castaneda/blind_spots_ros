#!/usr/bin/env python3


# PENDING TO MODIFY FOR 640 X 480 RESOLUTION !!! LINE 46 IN fuzzCtrlV2

#VERSION_3 VARIABLES OPTIMIZED

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from oni_pkg.msg import cStat
from oni_pkg.msg import fuzzC
import message_filters
import skfuzzy as fuzz
from skfuzzy import control as ctrl

#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#---------------------------------------------------------------------------------------------------------------------------V HISTOGRAM CALCULATIONS  

def hst(im):
   #l, r, f = cv2.split(im)
    l, r, f, b = cv2.split(im)
    
    hl  = cv2.calcHist([l],[0],None,[256],[0,256])
    hr  = cv2.calcHist([r],[0],None,[256],[0,256])
    #hf  = cv2.calcHist([f],[0],None,[256],[0,256])
    
    #return(hl, hr, hf)
    return(hl, hr)
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#-----------------------------------------------------------------------------------------------------------------------------MEASURE EXPOSURE ERROR

def expe(hst):
    en, ed = 0 , 0   # EXPOSURE NUM AND DEN
     
    for i in range(255):
        en += i*( round(hst[i,0]) )
        ed +=     round(hst[i,0])
        
    if ed ==0:
        exp     = (1/255)*(en/0.1)
    else:
        exp     = (1/255)*(en/ed)
    err = (-0.5)+exp
    return(err)
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||  
#-------------------------------------------------------------------------------------------------------------------------GET_FUZZY_CORRECTION_VALUE
def corr(actrl, gctrl, expErr, expCmos):    # INPUTS: EXPOSURE ERROR AND CMOS STATUS (CURRENT CAMERA EXPOSURE)
    
    ec                     =  ctrl.ControlSystemSimulation(actrl)   # "ec" IS EXPOSURE CONTROLLER
    ec.input['expErr']     =  expErr
    ec.compute()
    
    et                     =  3 #EXPOSURE TUNNER

    gc                     =  ctrl.ControlSystemSimulation(gctrl)   # "gc" IS GAIN CONTROLLER
    gc.input['expErr']     =  expErr
    gc.input['cmosStat']   =  expCmos
    gc.compute()    
    
    gt                     =  1    # GAIN TUNNER
     
    dexp                   =  (ec.output['deltaExp'])  *  et
    dg                     =  (gc.output['deltaGain']) *  gt
    
    return dexp, dg         # RETURN DEFUZZY EXPOSURE AND DEFUZZY GAIN
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#---------------------------------------------------------------------------------------------------------------INITIALIZE_FUZZY_EXPOSURE_CONTROLLER
def iniFuzzyA():
    #TAGS---------------------------------------------------------------------------
    exr        = ctrl.Antecedent(np.arange(-0.5,0.5,0.01),  'expErr'  )      #EXPOSURE ERROR    
    dex        = ctrl.Consequent(np.arange(-10,10,0.1),     'deltaExp')      #DELTA_EXPOSURE  
    #ANTECEDENT1||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    exr['vn'] =  fuzz.trapmf(exr.universe,    [-0.5, -0.5,  -0.3,  -0.2])  
    exr['n']  =  fuzz.trimf(exr.universe,     [-0.3,   -0.2, -0.12])    
    exr['c']  =  fuzz.trimf(exr.universe,     [-0.12,  0,   0.12])
    exr['p']  =  fuzz.trimf(exr.universe,     [ 0.12,  0.2,   0.3 ])    
    exr['vp'] =  fuzz.trapmf(exr.universe,    [ 0.2,  0.3,  0.5, 0.5])
    #CONSEQUENT---------------------------------------------------------------------
    dex['vn'] =  fuzz.trapmf(dex.universe,  [-40, -40, -7, -5])
    dex['n']  =  fuzz.trimf(dex.universe,   [-5, -3, -1])
    dex['c']  =  fuzz.trimf(dex.universe,   [-0.001 , 0, 0.001])
    dex['p']  =  fuzz.trimf(dex.universe,   [1,  3,  5])
    dex['vp'] =  fuzz.trapmf(dex.universe,  [5, 7, 40, 40])
    #RULES--------------------------------------------------------------------------
    r1 = ctrl.Rule(exr['vn'], dex['vp'])
    r2 = ctrl.Rule(exr['n'],  dex['p'])
    r3 = ctrl.Rule(exr['c'],  dex['c'])
    r4 = ctrl.Rule(exr['p'],  dex['n'])
    r5 = ctrl.Rule(exr['vp'], dex['vn'])
    
    #CONTROLLER---------------------------------------------------------------------
    ac     = ctrl.ControlSystem([r1, r2, r3, r4, r5])
    return ac
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#-------------------------------------------------------------------------------------------------------------------INITIALIZE_FUZZY_GAIN_CONTROLLER    
def iniFuzzyG():
    #TAGS---------------------------------------------------------------------------
    exr        = ctrl.Antecedent(np.arange(-0.5,0.5,0.01), 'expErr')       #EXPOSURE ERROR
    cs         = ctrl.Antecedent(np.arange(0,8190,10),   'cmosStat')       #CMOS STATUS
    dg         = ctrl.Consequent(np.arange(-20,20,1),    'deltaGain')      #DELTA GAIN
    #ANTECEDENT1||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    exr['vn'] =  fuzz.trapmf(exr.universe,     [-0.5, -0.5,  -0.3,  -0.2])
    exr['n']  =  fuzz.trimf(exr.universe,      [-0.3,   -0.2, -0.1])    
    exr['c']  =  fuzz.trimf(exr.universe,      [-0.1,  0,   0.1])
    exr['p']  =  fuzz.trimf(exr.universe,      [ 0.1,  0.2,   0.3 ])    
    exr['vp'] =  fuzz.trapmf(exr.universe,     [ 0.2,  0.3,  0.5, 0.5])
    #ANTECEDENT2||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    cs['l'] =  fuzz.trapmf(cs.universe,    [0, 0,  2047,  4094])
    cs['m'] =  fuzz.trimf(cs.universe,     [2047,  4094,   6141])
    cs['h'] =  fuzz.trapmf(cs.universe,    [4094,  6141,  8190, 8190])   
   
    #CONSEQUENT---------------------------------------------------------------------
    dg['vn'] =  fuzz.trapmf(dg.universe,  [-10, -10, -7, -5])
    dg['n']  =  fuzz.trimf(dg.universe,   [-5, -3, -1])
    dg['c']  =  fuzz.trimf(dg.universe,   [-0.007 , 0, 0.007])
    dg['p']  =  fuzz.trimf(dg.universe,   [1,  3,  5])
    dg['vp'] =  fuzz.trapmf(dg.universe,  [5, 7, 10, 10])    
    #RULES--------------------------------------------------------------------------
    r1 = ctrl.Rule(exr['vn']   & cs['l'], dg['c'])#nog
    r2 = ctrl.Rule(exr['n']    & cs['l'], dg['c'])#nog
    r3 = ctrl.Rule(exr['c']    & cs['l'], dg['c'])
    r4 = ctrl.Rule(exr['p']    & cs['l'], dg['n'])
    r5 = ctrl.Rule(exr['vp']   & cs['l'], dg['vn'])
    
    r6  = ctrl.Rule(exr['vn']  & cs['m'],  dg['c'])#nog
    r7  = ctrl.Rule(exr['n']   & cs['m'],  dg['c'])#nog
    r8  = ctrl.Rule(exr['c']   & cs['m'],  dg['c'])#nog
    r9  = ctrl.Rule(exr['p']   & cs['m'],  dg['n'])
    r10 = ctrl.Rule(exr['vp']  & cs['m'],  dg['vn'])

    r11 = ctrl.Rule(exr['vn']  & cs['h'],  dg['vp'])
    r12 = ctrl.Rule(exr['n']   & cs['h'],  dg['p'])
    r13 = ctrl.Rule(exr['c']   & cs['h'],  dg['c'])
    r14 = ctrl.Rule(exr['p']   & cs['h'],  dg['n'])
    r15 = ctrl.Rule(exr['vp']  & cs['h'],  dg['vn'])
    
    #CONTROLLER---------------------------------------------------------------------
    gc     = ctrl.ControlSystem([r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, r13, r14, r15])
    return gc

#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||    
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#-------------------------------------------------------------------------------------------------------------------FUZZY-CTRL-NODE

#WHEN USING MSG_FILTERS THIS IS THE CALLBACK DEPLOYED WHEN THE MSGS ARE SYNCHRONIZED-----------------------------------------------
#----------------------------------------------------------------------------------------------------CALLBACK (PROCESSING SEQUENCE)

def rc(m, ms):
    
    #im = bdg.imgmsg_to_cv2(m,"bgra8")
    
    #gl,   gr,  gf,          el, er, ef  =  ms.gl, ms.gr, ms.gf, ms.el, ms.er, ms.ef # "ms" IS MSG OF CAMERA STATUS

    #hl, hr, hf   = hst(im)
    
    #erl,  err, erf  =  expe(hl)  , expe(hr)  , expe(hf)   # (EXPOSITION_ERROR L, R AND F) 
    
    #dexl, dgl    =  corr(ac, gc, erl, el)  #GET FUZZY CORRECTION FOR LEFT IMG     #(DEFUZZY_EXPO_L, DEFUZZY_GAIN_L)
    #dexr, dgr    =  corr(ac, gc, err, er)  #GET FUZZY CORRECTION FOR RIGHT IMG    #(DEFUZZY_EXPO_R, DEFUZZY_GAIN_R)
    #dexf, dgf    =  corr(ac, gc, erf, ef)  #GET FUZZY CORRECTION FOR FRONT IMG    #(DEFUZZY_EXPO_F, DEFUZZY_GAIN_F)
    #defuzzy_exp_b, defuzzy_gain_b    =  get_correction(a_ctrl, g_ctrl, exp_err_b, exp_b)  #GET FUZZY CORRECTION FOR BACK IMG    #(DEFUZZY_EXPO_B, DEFUZZY_GAIN_B)
    
    #fm.cel,  fm.cer,  fm.cef   = round(el  + round(dexl))   ,  round(er + round(dexr))  ,  round(ef + round(dexf))  #CORRECTION OF EXPOSURE L R AND F
    #fm.cgl,  fm.cgr,  fm.cgf   = round(gl  + round(dgl))    ,  round(gr + round(dgr))   ,  round(gf + round(dgf))   #CORRECTION OF GAIN L, R AND F
        
    im      =  bdg.imgmsg_to_cv2(m,"bgra8")
    
    gl,   gr,   el, er =  ms.gl, ms.gr, ms.el, ms.er# "ms" IS MSG OF CAMERA STATUS
    
    hl, hr  =  hst(im)
    
    erl,  err  =  expe(hl)  , expe(hr)    # (EXPOSITION_ERROR L, R AND F)
    
    dexl, dgl    =  corr(ac, gc, erl, el)  #GET FUZZY CORRECTION FOR LEFT IMG     #(DEFUZZY_EXPO_L, DEFUZZY_GAIN_L)
    dexr, dgr    =  corr(ac, gc, err, er)  #GET FUZZY CORRECTION FOR RIGHT IMG    #(DEFUZZY_EXPO_R, DEFUZZY_GAIN_R)  
    
    fm.cel,  fm.cer   = round(el  + round(dexl))   ,  round(er + round(dexr))    #CORRECTION OF EXPOSURE L R 
    fm.cgl,  fm.cgr   = round(gl  + round(dgl))    ,  round(gr + round(dgr))     #CORRECTION OF GAIN L, R  



        
    pfc.publish(fm)
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#---------------------------------------------------------------------------------------------------------------------MAIN_SEQUENCE
if __name__ == '__main__':
    
    try:
        
        bdg = CvBridge()
        
        rospy.init_node('imFuzzyCtrl',anonymous = False)      # 1 -- INITIALIZE CONTROLLER NODE
        rate       = rospy.Rate(30)                           # 2 -- SET REFRESH RATIO
        
        rospy.set_param('nodeimFuzzyCtrl', 1)                 # 3 -- ENABLE FLAG FOR THE "fuzzCtrl" NODE
        rospy.loginfo('fuzzyCtrlNodeInitialized')             # 4 -- REGISTER THE INITIALIZATION OF THE NODE
        
        fm  = fuzzC()                                         # 5 -- INITIALIZE CONTROLLER MSG OBJECT    
        
        ac  =  iniFuzzyA()   #APERURE CONTROLLER              # 6 -- INITIALIZE EXPO AND GAIN CONTROLLERS
        gc  =  iniFuzzyG()   #GAIN CONTROLLER
        
        s   = message_filters.Subscriber('/dIm', Image)
        sst = message_filters.Subscriber('/cStat', cStat) # SUBSCRIBER TO CAMERA STATUS 
        
        pfc = rospy.Publisher('/fCtrl', fuzzC, queue_size=0.00005)  #(PUBLISHER OF FUZZY CONTROL) #8 -- INITIALIZE PUBLISHER TO RETURN THE CONTROL CORRECTIONS
         
        syn = message_filters.ApproximateTimeSynchronizer([s, sst ], queue_size=1, slop=0.00005)     
        syn.registerCallback(rc)  #8  DEFINE DATA HANDLER  (RETURN CORRECTION)
        rospy.spin()              #9 INITIALIZE PROCESSING LOOP
        
    except rospy.ROSInterruptException:
        pass    
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||       
    
    
    
    
