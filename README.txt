# blind_spots_ros

#PROJECT IN PROGRESS-- WAIT4 FINAL VER.
#---------------------------------------------------------------------------------------------------
#REQUIREMENTS:

--VIRTUAL ENVIRONMENT DEPENDENCIES (PIP 21.3.1 - python3)
cv_bridge        >>>  1.13.0 [COMPILED BELOW !!]
opencv_python    >>>  4.5.5  [COMPILED BELOW !!]
tf               >>>  1.12.1 [ADDED BELOW !!]
rospy            >>>  1.14.13
numpy            >>>  1.19.5
sensor-msgs      >>>  1.12.8
message_filters  >>>  1.14.13
numba            >>>  0.53.1
scikit-learn     >>>  0.24.2
scikit-fuzzy     >>>  0.4.2
pyYAML           >>>  6.0
laser_geometry   >>>  1.6.7
argparse         >>>  [LOCAL SHARED]
os               >>>  [LOCAL SHARED]
time             >>>  [LOCAL SHARED]
math             >>>  [LOCAL SHARED]

#---------------------------------------------------------------------------------------------------

--MAIN PKG (ONI_PKG) DEPENDENCIES FOR ROS MELODIC:
cv_bridge rospy roscpp std_msgs message_generation sensor_msgs laser_geometry laser_assembler tf

#---------------------------------------------------------------------------------------------------

--NVIDIA DRIVERS AND LIBS (FOR NVIDIA GEFORCE RTX3050TI -- COMPUTE CAPABILITY 8.6):
-CUDA TOOLKIT 11.7 - DRIVER 515.65.01  >>> https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=18.04&target_type=deb_local
-CUDNN 8.4.1.50                        >>> https://developer.nvidia.com/rdp/cudnn-archive (SELECT "Download cuDNN v8.4.1 (May 27th, 2022), for CUDA 11.x")

#---------------------------------------------------------------------------------------------------

--EXTERNAL PACKAGES FOR ROS: (CLONE THEM AND LOCATE THEM IN THE src DIRECTORY WITH THE ONI_PKG OR MAIN PACKAGE. THEN COMPILE EVERYTHING)°°°°°
-geometry2_python3   >>>  https://github.com/jsk-ros-pkg/geometry2_python3  
-rplidar_ros         >>>  https://github.com/Slamtec/rplidar_ros (EDIT rplidar.launch to deploy the devices /dev/ttyUSBX, /dev/ttyUSBX AT 115200 BAUDS)

#---------------------------------------------------------------------------------------------------

--PACKAGES TO BE COMPILED:
[CV_BRIDGE(PYTHON3)] --- FOLLOW THE NEXT TO COMPILE: https://cyaninfinite.com/ros-cv-bridge-with-python-3/ 
-cv_bridge           >>> https://github.com/ros-perception/vision_opencv.git


[OPENCV (WITH DNN SUPPORT)] --- FOLLOW THE NEXT TO COMPILE: https://techzizou.com/setup-opencv-dnn-cuda-module-for-linux/
-opencv 4.5.5               >>> https://github.com/opencv/opencv/archive/4.5.5.zip
-opencv_contrib 4.5.5       >>> https://github.com/opencv/opencv_contrib/archive/4.5.5.zip


[TF2(PYTHON3)] --- FOLLOW THE NEXT TO COMPILE: https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/

