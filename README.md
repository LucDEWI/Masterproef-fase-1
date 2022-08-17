# Masterproef-fase-1

## Masters project 2022: Design and validation of a bin picking robot vison algorithm.

This project contains the first stage of this project. This consists of an algorithm that can detect the position of an object via color filters. 

This project requires a depth camera to function. It is currently set up for the Intel Realsense D415 camera

# Installation
* Install Python 3.9
* Download this repository as a zip
* Open this repository in an IDE
* Install the required packages: Pyrealsense2, Opencv, Numpy, Math3d
* Run the scripts in the order prescribed

# Sctructure of project

The following section describes the scripts and how to use these scripts. Extra information is available in the comments of the scripts.

## Data

This directory contains the neccesary files for the script to run. These are several .npy files.

## Python-urx-master

This directory contains the Python-urx package that is neccesary to control a UR robot.

## Scripts

This directory contains the scripts that are used in this project. The order in which these need to executed will be explained in the use section.

## Source/ src

This directory contains the functions that are used in the scripts.

# Use

To use this project execute the scripts in the following order.

1. Camera_test

    This script is used to test the connection with the Intel Realsense D415 camera. It will give a color and depth image.
  
2. HSV_cal

    This script is used to calibrate the color filter. Start this script and adjust the slidebars until only the object is visible.
  
3. Intri_cal

    Performs the intrinsic calibration of the camera. Use the chessboard provided in the data directory.
  
4. EXTR_cal

    Performs the extrinsic calibration of the camera. Use the chessboard provided in the data directory.

5. Pixel_HSV
  
    This script will detect the object and give the pixelcoordinates of the object.

6. XYZ_coordinates

    This script will give the coordinates of the object in the coordinate sytem of the extrinsic calibration.
  
# Using a robot

At this point the robot_sturen script does not function. It is recommended to not use this.







