# Calibration-of-Lidar-Camera-for-Object-Detection-onRB5
## Description
<p align="justify">
This project is intended to build and deploy the lidar camera calibration on the Qualcomm Robotics Platform RB5 which accurately interprets the objects in a scene and provides distance estimation.

## Introduction
<p align="justify">
This project proposed an object distance estimation method for autonomous systems. Lidar Camera calibration was performed on Qualcomm’s Robotics Platform RB5. The Platform was equipped with a 360 Laser Distance Sensor LDS-01 LiDAR sensor for 2D acquisition and a Logitech c270 camera for video streaming and Object detection. Both the LiDAR sensor and the camera are calibrated and approximately time-synced based on the pixel-per-meter approach. Further, this information is utilized to estimate the distance of objects detected by the trained model mobilenet_SSD_v2_2, which was converted from TensorFlow to SNPE.  
<p align="justify">
Basically, it finds the object in the input video frame and computes the angle and distance of the object to the camera and lidar.  And from there, the inference node: Qualcomm Neural Processing Engine (SNPE) draws the bounding box around the detected object and displays the accurate distance of that object from the lidar and camera.  
<p align="justify">
The solution was developed as a prototype using the ROS2. 
 
## Prerequisites  

1. A Linux host system with Ubuntu 18.04. 
2. Install Android Platform tools (ADB, Fastboot)  
3. Download and install the SDK Manager for RB5 
4. Flash the RB5 firmware image on to the RB5 
5. Setup the Network on RB5. 
6. Installed Python3.6 on RB5 
7. TurtleBot burger is assembled, operational, connected to RB5. 
8. Setup ROS2-Dashing on RB5. 
9. Connect the camera to RB5. 

 
## Steps to Setup the Project on RB5 

### Installing Dependencies      

- OpenCV Installation on RB5 :   
Run the command given below to install the OpenCV on RB5,    
    ```sh
    sh4.4 # python3 -m pip install --upgrade pip      
    sh4.4 # python3 -m pip install opencv-python 
    ```

- Installation of TurtleBot3 Package:

    For the setup, we will be using the TurtleBot3 Burger, we need to install TurtleBot Packages for controlling the TurtleBot. 

    1. Setup the necessary packages by executing the following commands. 
        ```sh
        sh4.4 #sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
        ```
    2. Create a new directory for TurtleBot 3. 
        ```sh
        sh4.4 # mkdir -p ~/turtlebot3_ws/src  
        sh4.4 # cd ~/turtlebot3_ws/src 
        ```

    3. Clone the necessary repositories and then access TurtleBot Folder 
        ```sh
        sh4.4 # git clone -b dashing-devel https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git 

        sh4.4 # git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git  

        sh4.4 # git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3.git 

        sh4.4 #git clone -b dashing-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git 

        sh4.4 # cd ~/turtlebot3_ws/src/turtlebot3   
        ```

    4. Remove the folders that are not required for current project 
        ```sh
        sh4.4 # rm -r turtlebot3_cartographer turtlebot3_navigation2 
        sh4.4 # cd ~/turtlebot3_ws/ 
        ```
    5. Source & Build the TurtleBot3 Setup file 
        ```sh
        sh4.4 # echo 'source /opt/ros/dashing/setup.bash' >> ~/.bashrc 

        sh4.4 # source ~/.bashrc 

        sh4.4 # colcon build --symlink-install --parallel-workers 1 

        sh4.4 # echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc 

        sh4.4 # source ~/.bashrc 

        sh4.4 # echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc 

        sh4.4 # echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc 

        sh4.4 # source ~/.bashrc 
        ```

- Steps to Setup Camera: 

    1. Connect USB Camera to RB5 board. 

    2. Attach the camera on Turtlebot3 Burger at the 3rd layer from om, 20 degrees facing down from vertical angle. 


- Steps to setup LiDAR: 

    1. Connect LIDAR Scanner to RB5 board using a micro-USB cable. 

    2. After connection makes sure the /dev/ttyUSB0 port is accessible. 

- Steps to setup SNPE: 

    1. Download the SNPE SDK from QUALCOMM's developer Network. from the link given below on the host system 
    https://developer.qualcomm.com/software/qualcomm-neural-processing-sdk/tools 

    2. Follow the instruction in link mentioned below to setup the SNPE https://developer.qualcomm.com/sites/default/files/docs/snpe/overview.html 

    3. Copy the SNPE header files & runtime libraries for aarch64-ubuntu-gcc7.5 on RB5 from host system using ADB.  

        ```sh
        sh4.4 # adb push <SNPE_ROOT>/include/ /data/snpe/include/  

        sh4.4 # adb push <SNPE_ROOT>/lib/aarch64-ubuntu-gcc7.5/* /data/snpe/  

        sh4.4 # adb push <SNPE_ROOT>/lib/dsp/* /data/snpe/  
        ```

    4. Open the terminal of RB5 and append the lines given below at the end of ~/.bashrc file. 
        ```sh
        sh4.4 #export PATH=$PATH:/data/snpe/ 

        sh4.4 #export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/data/snpe/  

        sh4.4 #export ADSP_LIBRARY_PATH="/data/snpe;/system/lib/rfsa/adsp;/system/vendor/lib/rfsa/adsp;/dsp"  
        ```
    

    5. Run the command given below to reinitialize the RB5’s terminal session  

        ```sh
        sh4.4 # source ~/.bashrc 
        ```

- PyBind11 Installation on RB5:

    Run the command given below to setting up the PyBind11 
    ```sh
    sh4.4 # apt update & apt install python3-pybind11 
    ```
 

### Building the SNPE Python Wrapper for Object Detection: 

1. Clone the project from the link below on the RB5 workspace under src directory.
    ```sh
    sh4.4 # git clone link.git 
    ```
 

2. Go inside the src folder of cloned project,  
    ```sh
    sh4.4 # cd <PROJECT_PATH>/ 
    ```
3. Run the command below in order to build the shared library for Python wrapper of the SNPE. 
    ```sh
    sh4.4 # g++ -std=c++11 -fPIC -shared -o qcsnpe.so qcsnpe.cpp -I include/ -I /data/snpe/include/zdl/ -I /usr/include/python3.6m/ -I /usr/local/lib/python3.6/dist-packages/pybind11/include -L /data/snpe/ -lSNPE `pkg-config --cflags --libs opencv` 
    ```
 
### Environment setup instructions before running the application: 

1. In the terminal-1 run the TurtleBot Bringup Command 
    ```sh
    sh4.4 # ros2 launch turtlebot3_bringup robot.launch.py 
    ```
2. Open a second terminal, and we can launch our application from that. If you are using adb shell, don’t miss to source before running the application. 
    ```sh
    sh4.4 #  source ~/.bashrc  
    ```

3. Go to the <PROJECT_PATH> 
    ```sh
    sh4.4 # cd <PROJECT_PATH>/ 
    ```
 
4. Run the main file 
    ```sh
    sh4.4 # python3 main.py 
    ```
