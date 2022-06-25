#!/usr/bin/env python3

import rospy
import numpy as np
from numpy.linalg import inv
import math
import time
import yaml
import argparse
#-----------------------------------------------------------------------------------------------------------------------------------------DICTIONARY
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-------------------------------------------------------------------------------------------------------------------------IDENTIFIERS OF THE CAMERAS
cams_ids =  { 1:"left" , 2:"right", 3:"front", 4:"back"}
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#---------------------------------------------------------------------------------------------------INITIALIZE VIRTUAL CAMERA MATRIX AND ITS INVERSE
def init_vir_mtx():
    
    #fax, fay  =  209.775, 209.775
    #ua0, va0  =  250    , 250
    #h_vir     =  3356.4      
    #vc_kc     =  np.array([[fax, 0, ua0],[0, fay, va0],[0,0,1]])        
    #vc_kc_inv =  inv(vc_kc)
    
    fax, fay   =  419.55 , 419.55                                    #ONLY4TEST-----------__!!!!!!!!!!!!
    ua0, va0   =  500    , 500                                       #ONLY4TEST-----------__!!!!!!!!!!!!
    h_vir      =  7551.9                                             #ONLY4TEST-----------__!!!!!!!!!!!!
    vc_kc     =  np.array([[fax, 0, ua0],[0, fay, va0],[0,0,1]])     #ONLY4TEST-----------__!!!!!!!!!!!!    
    vc_kc_inv =  inv(vc_kc)                                          #ONLY4TEST-----------__!!!!!!!!!!!!
        
    
    return (vc_kc, vc_kc_inv, h_vir)
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#---------------------------------------------------------------------------------------IMPORT CALIB DATA FROM YAML AND UPLOAD TO ROS PARAMS. SERVER
def import_im_calib(calib_data_path):
    
    with open(calib_data_path) as c:  
    
        calib_file  = yaml.load(c, Loader=yaml.FullLoader) 
        
    for i in range (len(cams_ids)):
        
        cam_dic  = calib_file.get( cams_ids[i+1] ) #AT THE 1ST ITERATION 0+1 == 1--> EQUAL TO "left" IN THE CAMS DIC OF ABOVE

        k_list   = cam_dic.get("k_mtx")  #EXTRACT THE CAMERA MATRIX
        d_list   = cam_dic.get("d_mtx")  #EXTRACT THE DISTORTION MATRIX   
        rt_list  = cam_dic.get("rt_mtx") #EXTRACT THE RT MATRIX 
        
        rospy.set_param( cams_ids[i+1]+'_mtx_k',  k_list)
        rospy.set_param( cams_ids[i+1]+'_mtx_d',  d_list)    
        rospy.set_param( cams_ids[i+1]+'_mtx_rt', rt_list) 
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
#-----------------------------------------------------------------------------CONVERT LIST TO NO ARRAY TO IMPORT PINHOLE MTX FROM ROS PARAMS. SERVER
def list2mtx_k(data_list):  
    fax   = data_list[0]; fay = data_list[4]; ua0 = data_list[2]; va0 = data_list[5] 
    k_mtx = np.array([[fax, 0, ua0],[0, fay, va0],[0,0,1]])          
    return(k_mtx)
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#------------------------------------------------------------------------------------------LOAD THE REFERENCE POINTS FROM THE .YAML CALIBRATION FILE
def import_ref(ref_path, h_vir, vc_kc_inv, target):
         
    with open(ref_path) as r:                              

        all_points = yaml.load(r, Loader=yaml.FullLoader)     

    data            = all_points.get(target)           # LEFT, RIGHT, FRONT OR BACK
    f_list   = data.get('in_img')              
    v_list   = data.get('vir_img')         

    x1_v = v_list[0]; y1_v = v_list[4]      
    x2_v = v_list[1]; y2_v = v_list[5]
    x3_v = v_list[2]; y3_v = v_list[6]
    x4_v = v_list[3]; y4_v = v_list[7]

    v_pix     =  np.array([ [ x1_v, x2_v, x3_v, x4_v ] , [ y1_v, y2_v, y3_v, y4_v ], [1,1,1,1] ]  )   
    v_cor     =  np.array([ [], [], [] ])   

    for i in range(4):   

        points     = v_pix[0:,i]                                                
        points_vec = np.array([ [points[0]] , [points[1]], [points[2]] ]  )    
        cord   = pix2cor(points_vec, h_vir, vc_kc_inv)                         

        if i==0:               
            v_cor = cord

        elif i>0:             
            v_cor= np.append(v_cor,cord,1)    

    x1_f = f_list[0]; y1_f = f_list[4]        
    x2_f = f_list[1]; y2_f = f_list[5]
    x3_f = f_list[2]; y3_f = f_list[6]
    x4_f = f_list[3]; y4_f = f_list[7]

    f_points  =  np.array([ [ x1_f, x2_f, x3_f, x4_f ] , [ y1_f, y2_f, y3_f, y4_f ], [1,1,1,1] ]  )   

    return(v_pix, v_cor, f_points)        

#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-----------------------------------------------------------------------------------------------------------------------REMAP, PIXELS TO COORDINATES
def pix2cor(px, h, kc_inv): 
    cor_1   =  np.matmul( (h* kc_inv),px )
    return cor_1
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#-----------------------------------------------------------------------------------EXPORT THE RT MATRIX AND ITS INVERSE TO THE MAIN CLIB .YAML FILE
def export_mtx(path2export):    
   
    rt_l = rospy.get_param('left_mtx_rt')
    rt_r = rospy.get_param('right_mtx_rt')
    rt_f = rospy.get_param('front_mtx_rt')
    rt_b = rospy.get_param('back_mtx_rt')
    
    rt_inv_l = rospy.get_param('left_mtx_rt_inv')
    rt_inv_r = rospy.get_param('right_mtx_rt_inv')
    rt_inv_f = rospy.get_param('front_mtx_rt_inv')
    rt_inv_b = rospy.get_param('back_mtx_rt_inv')
             
    data2exp = { 'left':  {'mtx_rt': rt_l,'mtx_rt_inv': rt_inv_l} ,  'right': {'mtx_rt': rt_r,'mtx_rt_inv': rt_inv_r},
                
                'front': {'mtx_rt': rt_f,'mtx_rt_inv': rt_inv_f},    'back':  {'mtx_rt': rt_b,'mtx_rt_inv': rt_inv_b}    }
                
    with open(path2export, 'w') as f:
        
        yaml.dump(data2exp, f,sort_keys=False) 
#---------------------------------------------------------------------------------------------------------------------------------------------METHOD
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#----------------------------------------------------------------------------------------------------------CALCULATE THE ROTATION-TRANSLATION MATRIX
def get_rt_mtx(fc_kc, v_cor, f_points, target):
        
    fax_f = fc_kc[0,0];   fay_f = fc_kc[1,1];   ua0_f = fc_kc[0,2];   va0_f = fc_kc[1,2]    
    
    v11=v_cor[0,0]; v21=v_cor[1,0]; v31=v_cor[2,0]   
    v12=v_cor[0,1]; v22=v_cor[1,1]; v32=v_cor[2,1]   
    v13=v_cor[0,2]; v23=v_cor[1,2]; v33=v_cor[2,2]
    v14=v_cor[0,3]; v24=v_cor[1,3]; v34=v_cor[2,3]   

    a1 = f_points[0,0];     a3 = f_points[0,1];     a5 = f_points[0,2];     a7 = f_points[0,3]
    a2 = f_points[1,0];     a4 = f_points[1,1];     a6 = f_points[1,2];     a8 = f_points[1,3]

    ext =  np.array([ [fax_f*v11,fax_f*v21,fax_f*v31,0,0,0,(ua0_f-a1)*v11,(ua0_f-a1)*v21],  
                      [0,0,0,fay_f*v11,fay_f*v21,fay_f*v31,(va0_f-a2)*v11,(va0_f-a2)*v21],
                      [fax_f*v12,fax_f*v22,fax_f*v32,0,0,0,(ua0_f-a3)*v12,(ua0_f-a3)*v22],
                      [0,0,0,fay_f*v12,fay_f*v22,fay_f*v32,(va0_f-a4)*v12,(va0_f-a4)*v22],
                      [fax_f*v13,fax_f*v23,fax_f*v33,0,0,0,(ua0_f-a5)*v13,(ua0_f-a5)*v23],
                      [0,0,0,fay_f*v13,fay_f*v23,fay_f*v33,(va0_f-a6)*v13,(va0_f-a6)*v23],
                      [fax_f*v14,fax_f*v24,fax_f*v34,0,0,0,(ua0_f-a7)*v14,(ua0_f-a7)*v24],
                      [0,0,0,fay_f*v14,fay_f*v24,fay_f*v34,(va0_f-a8)*v14,(va0_f-a8)*v24]   ]) 

    ext_inv = inv(ext)      
    
    ext_mtx_p=np.array([[(a1-ua0_f)*v31],[(a2-va0_f)*v31],  [(a3-ua0_f)*v31],[(a4-va0_f)*v31],  [(a5-ua0_f)*v31],[(a6-va0_f)*v31], [(a7-ua0_f)*v31],[(a8-va0_f)*v31]])

    rt_pms=np.matmul(ext_inv, ext_mtx_p)  

    r11=rt_pms[0,0];   r12=rt_pms[1,0];   r13=rt_pms[2,0];   r21=rt_pms[3,0];   r22=rt_pms[4,0];   r23=rt_pms[5,0];   r31=rt_pms[6,0];   r32=rt_pms[7,0]  

    rt=np.array([[r11,r12,r13],[r21,r22,r23],[r31,r32,1]])

    rt_inv=inv(rt)   

    
    rospy.set_param( target +'_mtx_rt',      rt.tolist()    )
    rospy.set_param( target +'_mtx_rt_inv',  rt_inv.tolist() )
    
    #export_mtx(path2export, rt, rt_inv, target)
    
#||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||   
#---------------------------------------------------------------------------------------------------------------------MAIN_SEQUENCE

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='calibration data from cameras')
    
    #parser.add_argument('-c','--calib_data_path',  type=str,  default='/home/sierra/oni_dir/oni_ws/src/oni_pkg/yaml_config/cams_data.yaml',       help='img calib data, file path')
    #parser.add_argument('-p','--ref_calib_points', type=str,  default='/home/sierra/oni_dir/oni_ws/src/oni_pkg/yaml_config/ext_cams_calib.yaml',  help='reference points for bird calib, file path')
    #parser.add_argument('-o','--out_path',         type=str,  default='/home/sierra/oni_dir/oni_ws/src/oni_pkg/yaml_config/bird_calib_data.yaml', help='output path for calibration results')  
    
    parser.add_argument('-c','--calib_data_path',  type=str,  default='/home/sierra/oni_dir/oni_ws/src/oni_pkg/yaml_config/cams_data_test.yaml',       help='path')  #ONLY4TEST-----------__!!!!!!!!!!!!
    parser.add_argument('-p','--ref_calib_points', type=str,  default='/home/sierra/oni_dir/oni_ws/src/oni_pkg/yaml_config/ext_cams_calib_test.yaml',  help='path')  #ONLY4TEST-----------__!!!!!!!!!!!!
    parser.add_argument('-o','--out_path',         type=str,  default='/home/sierra/oni_dir/oni_ws/src/oni_pkg/yaml_config/bird_calib_data.yaml',      help='path')  #ONLY4TEST-----------__!!!!!!!!!!!!
    
    
    args, unknown = parser.parse_known_args()

#------------------------------------------------------------------------------------------------------------DEFINITION OF VIRTUAL CAMERA PARAMETERS
#|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
#---------------------------------------------------------------------------------------------------------------------------------------------------

    vc_kc, vc_kc_inv, h_vir  = init_vir_mtx()      #  1 -- INITIALIZE VIRTUAL CAMERA MATRIX AND ITS INVERSE

    import_im_calib(args.calib_data_path)          #  2 -- IMPORT CALIBRATION DATA OF ALL THE CAMERAS FROM THE MAIN CALIB .YAML FILE --- UPLOAT TO ROS PARAM. SERVER
    
    for i in range (len(cams_ids)):                #  3 -- PERFORM EXTRINSIC CALIBRATION FOR EVERY CAMERA (BIRD VIEW CALIB)
        
            target  =  cams_ids[i+1]                                                                    # A --- GET THE IDENTIFIER OF THE IMAGE SORUCE
            
            fc_list =  rospy.get_param(target +'_mtx_k')                                                # B --- LOAD THE PINHOLE MTX DATA FOR THE TARGET IMAGE SOURCE, OBTAIN FROM ROS PARAM. SERVER
                                                                                                         
            fc_kc   =  list2mtx_k(fc_list)                                                              # C --- ORGANIZE PINHOLE MTX FROM THE LIST OF THE ROS PARAM. SERVER
            
            v_pix, v_cor, f_points   =   import_ref(args.ref_calib_points, h_vir, vc_kc_inv, target)    # D --- GET CALIB POINTS FROM THE TARGET IMAGE SOURCE
            
            get_rt_mtx(fc_kc, v_cor, f_points, target)                                                  # E --- CALCULATE RT MTX FOR EACH IMAGE SOURCE AND SAVE THE RT AND RT_INV MTXES
            
            path2export = args.out_path                                                                 # F --- SET EXPORTATION PATH FOR THE CALIB RESULTS
            
            export_mtx(path2export)                                                                     # G --- EXPORT THE CALIBRATION RESULTS TO .YAML FILE
           
    print ('\n'+ 'CALIBRATION RESULTS SUCCESSFULLY STORED AT:' + '\n\n'+ path2export)                   # H --- NOTIFY CALIBRATION IS READY
