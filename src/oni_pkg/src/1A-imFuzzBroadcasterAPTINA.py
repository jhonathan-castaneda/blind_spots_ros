#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import argparse
import os
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from oni_pkg.srv import yoloSample

#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------------------IDENTIFIERS OF THE CAMERAS
cid = { 2:"l" , "r":4}
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#---------------------------------------------------------------------------------------------------------------------------V HISTOGRAM CALCULATIONS  
def hst(im):
    hl  = cv2.calcHist([im],[0],None,[256],[0,256])
    return(hl)
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
    #CONSEQUENT---------------------------------------------------------------------1
    dex['vn'] =  fuzz.trapmf(dex.universe,  [-10, -10, -7, -5])
    dex['n']  =  fuzz.trimf(dex.universe,   [-5, -3, -1])
    dex['c']  =  fuzz.trimf(dex.universe,   [-3 , 0, 3])
    dex['p']  =  fuzz.trimf(dex.universe,   [1,  3,  5])
    dex['vp'] =  fuzz.trapmf(dex.universe,  [5, 7, 10, 10])
    #RULES--------------------------------------------------------------------------    
    r1 = ctrl.Rule(exr['vn'], dex['vp'])
    r2 = ctrl.Rule(exr['n'],  dex['p'] )
    r3 = ctrl.Rule(exr['c'],  dex['c'] )
    r4 = ctrl.Rule(exr['p'],  dex['n'] )
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
    dg         = ctrl.Consequent(np.arange(-10,10,1),    'deltaGain')      #DELTA GAIN
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
    dg['c']  =  fuzz.trimf(dg.universe,   [-3 , 0, 3])
    dg['p']  =  fuzz.trimf(dg.universe,   [1,  3,  5])
    dg['vp'] =  fuzz.trapmf(dg.universe,  [5, 7, 10, 10])    
    #RULES--------------------------------------------------------------------------
    r1 = ctrl.Rule(exr['vn']   & cs['l'], dg['vp'])#nog
    r2 = ctrl.Rule(exr['n']    & cs['l'], dg['p'])#nog
    r3 = ctrl.Rule(exr['c']    & cs['l'], dg['c'])
    r4 = ctrl.Rule(exr['p']    & cs['l'], dg['n'])
    r5 = ctrl.Rule(exr['vp']   & cs['l'], dg['vn'])
    
    r6  = ctrl.Rule(exr['vn']  & cs['m'],  dg['vp'])#nog
    r7  = ctrl.Rule(exr['n']   & cs['m'],  dg['p'])#nog
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
#-----------------------------------------------------------------------------------------------------------------------------------------------NODE
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#----------------------------------------------------------------------------------------------------------------------------------------BROADCASTER    
def bct():
    rospy.init_node('imgBroadcasterAPTINA', anonymous = False)                 # 1 -- INIT IM BROADCASTER
    rate   = rospy.Rate(30)                                                    # 2 -- SET REFRESH RATE
    rospy.set_param('nodeimBroadcasterAPTINA', 1)                              # 3 -- ENABLE FLAG FOR THE "imBroadcaster" NODE
    rospy.loginfo('imageBroadcasterAPTINANodeInitialized')                     # 4 -- REGISTER THE INITIALIZATION OF THE NODE    
    p    = rospy.Publisher('/dImAPTINA',  Image, queue_size=0.00005)           # DISTORTED IMAGE PUBLISHER

    #FUZZY MODULW ONLY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ac  =  iniFuzzyA()   #APERURE CONTROLLER              # 6 -- INITIALIZE EXPO AND GAIN CONTROLLERS
    gc  =  iniFuzzyG()   #GAIN CONTROLLER
    #FUZZY MODULW ONLY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    
    while not rospy.is_shutdown():   # MAIN SEQUENCE------------------------------------------
         
        rl, fl = l.read()                      # 6 -- READ LEFT FRAME
        rr, fr = r.read()                      # 7 -- READ RIGHT FRAME
 
        fl = cv2.cvtColor(fl, cv2.COLOR_BGR2GRAY)
        fr = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)    
        c3 = np.zeros_like(fl)
        
        im  = cv2.merge((fl,fr,c3))            # JOIN FRAMES 
        m   =  bdg.cv2_to_imgmsg(im, "bgr8")    
        p.publish(m)
        
        #FUZZY MODULW ONLY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!--------------LEFT CAM
        el        =  int(l.get(cv2.CAP_PROP_EXPOSURE) )  # <<< CURRENT EXPOSURE
        gl        =  int(l.get(cv2.CAP_PROP_GAIN) )      # <<< CURRENT GAIN
            
        hl        =  hst(fl)                   # <<< HISTOGRAM   
        erl       =  expe(hl)                  # <<< EXPOSURE ERROR
        dexl, dgl =  corr(ac, gc, erl, el)     # <<< GET FUZZY CORRECTIONS 
        cel       =  int(round(el  + round(dexl)))  # <<< CORRECTION OF EXPOSURE   
        cgl       =  int(round(gl  + round(dgl)))   # <<< CORRECTION OF GAIN   
        #-------------------------------------------------------------------------------------------------------------                   
        if abs(erl) > 0.1:                     # <<< IF IN CORR RANGE
            
            if cel >= 8187:  
                pass
            elif cel <=0:
                pass   
            elif cel > 0 and erl <8187:
                l.set(cv2.CAP_PROP_EXPOSURE, cel)   
                #pass          
            if cgl >=255:  
                pass  
            elif cgl <=0:
                pass 
            elif cgl > 0 & cgl <255:
                l.set(cv2.CAP_PROP_GAIN, cgl)
                #pass
        else:
            pass
        #FUZZY MODULW ONLY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!---------------LEFT CAM
        #-------------------------------------------------------------------------------------------------------------        
        #-------------------------------------------------------------------------------------------------------------
        #-------------------------------------------------------------------------------------------------------------
        #FUZZY MODULW ONLY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!--------------RIGHT CAM
        er        =  int(r.get(cv2.CAP_PROP_EXPOSURE) )  # <<< CURRENT EXPOSURE
        gr        =  int(r.get(cv2.CAP_PROP_GAIN) )      # <<< CURRENT GAIN
            
        hr        =  hst(fr)                   # <<< HISTOGRAM   
        err       =  expe(hr)                  # <<< EXPOSURE ERROR
        dexr, dgr =  corr(ac, gc, err, er)     # <<< GET FUZZY CORRECTIONS 
        cer       =  int(round(er  + round(dexr)))  # <<< CORRECTION OF EXPOSURE   
        cgr       =  int(round(gr  + round(dgr)))   # <<< CORRECTION OF GAIN   
        #-------------------------------------------------------------------------------------------------------------                   
        if abs(err) > 0.1:                     # <<< IF IN CORR RANGE
            
            if cer >= 8187:  
                pass
            elif cer <=0:
                pass   
            elif cer > 0 and err <8187:
                r.set(cv2.CAP_PROP_EXPOSURE, cer)   
                #pass          
            if cgr >=255:  
                pass  
            elif cgr <=0:
                pass 
            elif cgr > 0 & cgr <255:
                r.set(cv2.CAP_PROP_GAIN, cgr)
                #pass
        else:
            pass
        ################################################################
        #FUZZY MODULW ONLY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!--------------RIGHT CAM 
        if rospy.is_shutdown():
            l.release()
            r.release()
#--------------------------------------------------------------------------------------------------------------------------------------MAIN SEQUENCE  
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------------------------------------------------------ GENERAL V4L SETTINGS
if __name__ == "__main__":
    parser  =  argparse.ArgumentParser()
    parser.add_argument("--pixelFormat",            type=str,  default="M,J,P,G",   help='"M,J,P,G" or "Y,U,Y,V", default="M,J,P,G"')
    parser.add_argument("--fps",                    type=int,  default=30,          help="default=30")   
    parser.add_argument("--field",                  type=str,  default="top",       help="top, bottom, any or none default=top=30fps") 
    parser.add_argument("--imageWidth",             type=int,  default=1280,        help="default=1280")
    parser.add_argument("--imageHeight",            type=int,  default=720,         help="default=720")   
    parser.add_argument("--exposureAutoPriority",  action="store_true", default=0,  help="(bool) default=0")            
    parser.add_argument("--exposureAuto",          action="store_true", default=1,  help="(1=manual,3=auto) default=1")  
    parser.add_argument("--exposureAbsolute",       type=int,  default=3,           help="min=3 max=8186 step=1 default=3")  
    parser.add_argument("--gain",                   type=int,  default=10,          help="min=0 max=255 step=1 default=10")   
    parser.add_argument("--backlightCompensation",  type=int,  default=0,           help="min=0 max=1 step=1 default=0")
    parser.add_argument("--brightness",             type=int,  default=4,           help="min=-64 max=64 step=1 default=4")
    parser.add_argument("--contrast",               type=int,  default=7,           help="min=0 max=95 step=1 default=7")
    parser.add_argument("--saturation",             type=int,  default=1,           help="min=0 max=100 step=1 default=1")
    args, unknown = parser.parse_known_args()
    print(args)
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-----------------------------------------------------------------------------------------------------------LOAD THE INITIAL SETTINGS TO THE CAMERAS
    c1="v4l2-ctl -d 2 --set-ctrl="                                            ; c2="v4l2-ctl -d 4 --set-ctrl="
    
    os.system(c1+'auto_exposure='+str(args.exposureAuto))                     ; os.system(c2+'auto_exposure='+str(args.exposureAuto))
    os.system(c1+'exposure_time_absolute='+str(args.exposureAbsolute))        ; os.system(c2+'exposure_time_absolute='+str(args.exposureAbsolute)) 
    os.system(c1+'gain='+str(args.gain))                                      ; os.system(c2+'gain='+str(args.gain))  
    os.system(c1+'backlight_compensation='+str(args.backlightCompensation))   ; os.system(c2+'backlight_compensation='+str(args.backlightCompensation))
    os.system(c1+'brightness='+str(args.brightness))                          ; os.system(c2+'brightness='+str(args.brightness))
    os.system(c1+'contrast='+str(args.contrast))                              ; os.system(c2+'contrast='+str(args.contrast))
    os.system(c1+'saturation='+str(args.saturation))                          ; os.system(c2+'saturation='+str(args.saturation))
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-----------------------------------------------------------------------------------------------------------------------------CAPTURE INITIALIZATION
    l, r = cv2.VideoCapture(2)                                                , cv2.VideoCapture(4) 
#------------------------------------------------------------------------------------------------------------------SET RESOLUTION, VIDEO FORMAT, FPS
#----------------------------THIS IS CONFIGURED HERE SINCE AFTER INITIALIZING THE VIDEOCAPTURE OBJECTS OF CV2 THE VIDEO FORMAT CONFIG IS OVERWRITTEN
    vf    = (args.pixelFormat).split(",")
    fcc   = cv2.VideoWriter_fourcc(vf[0], vf[1], vf[2], vf[3])

    l.set(cv2.CAP_PROP_FOURCC, fcc)                                           ; r.set(cv2.CAP_PROP_FOURCC, fcc)        
    l.set(cv2.CAP_PROP_FRAME_WIDTH, args.imageWidth)                          ; r.set(cv2.CAP_PROP_FRAME_WIDTH, args.imageWidth)
    l.set(cv2.CAP_PROP_FRAME_HEIGHT, args.imageHeight)                        ; r.set(cv2.CAP_PROP_FRAME_HEIGHT, args.imageHeight)
    l.set(cv2.CAP_PROP_FPS, args.fps)                                         ; r.set(cv2.CAP_PROP_FPS, args.fps)

    rospy.loginfo('DefaultSettingsLoadedToSensors')  # -- REGISTER THE INITIALIZATION OF THE SENSORS

    bdg = CvBridge()                     # -- INITIALIZE ROS-CV2 BRIDGE 
    
    try:
        bct()                            # -- DEPLOY BROADCASTER NODE
    except rospy.ROSInterruptException:
        pass
