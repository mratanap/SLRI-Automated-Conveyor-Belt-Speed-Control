# SLRI-Automated-Conveyor-Belt-Speed-Control




## Automated Conveyor Belt Speed Control 





### Important Files
<p>
     auto_conveyor_control.py 
<br> auto_conveyor_control_aruco.py
<br> realsense_camera.py
<br> Yolov8x_mango.pt [Download Here](https://drive.google.com/file/d/1tRlFyRb7xSoRe0V4f7Gs9ZHWKA-lsY7o/view?usp=sharing)
<br> test_arduino_connection.py
</p>


### Concept overview 
<img src="https://github.com/mratanap/SLRI-Automated-Conveyor-Belt-Speed-Control/assets/125659155/8e53e220-1f76-4343-b335-133941bfb8e6" alt="drawing" width="500"/>







### Programming Language
<p>
Python version 3.10 (pyrealsense2’s support for python 3.11 is unstable at this moment)
</p>







### Packages
<p>
     Please make sure these packages are properly installed, so libraries can be imported later
<br> pip install opencv-python
<br> pip install numpy
<br> pip install ultralytics
<br> pip install pyfirmata
<br> pip install pyserial
</p>







### Libraries
<p>
     import cv2
<br> import math
<br> import time
<br> import numpy as np
<br> from ultralytics import YOLO
<br> import pyfirmata 
<br> import serial
</p>







### Script File
<p>
from realsense_camera import *
</p>







### Functions

    speedConveyor(speed): 

&rarr; To set speed (mm/s) and send pwm to arduino

    vel_function(width,depth):
&rarr; To calculate velocity based on width and depth information of objects that enter the irradiation zone, the velocity formula can be adjusted here.








### Important Variables

#### ** variable needs setting 

    model **
&rarr; Model for object detection

    model_path **
&rarr; Path of model, for window please change \ to /. Example ‘C:/Users/SLRI/…/yolo.pt’

<br>
<br>
<br>
<br>
<br>
<br>
<br>

    elapsed_time 
&rarr; Check if time passed for x seconds, use when capturing every x seconds, ignore this if you want to capture in real time.

<br>
<br>
<br>
<br>
<br>
<br>
<br>

    Camera_dist_mm **
&rarr; Distance from camera to surface
&rarr; Used to calculate objects depth

    mm_px_ratio **
&rarr; Ratio that will be used to calculate size of objects
&rarr; Please manually calibrate here or use aruco marker for semi-auto calibration

<br>
<br>
<br>
<br>
<br>
<br>
<br>

    Irradiating Zone **
&rarr; Straight line indicating irradiating zone
&rarr; Please set coordinates of this line 
 
    irradiating_width 
&rarr; Measure from width of objects that enter irradiating zone (objects that intersect with straight line), measured in pixel

    irradiating_border_top
&rarr; Top most position of irradiating border, in coordinates (x,y)
    
    irradiating_border_bottom
&rarr; Bottom most position of irradiating border, in coordinates (x,y)

<br>
<br>
<br>
<br>
<br>
<br>
<br>

    depth_mm[]
&rarr; Array consisting of depth information of each object that enter irradiating zone
    
    avg_depth_mm
&rarr; Average depth value of all objects that enter irradiating zone

<br>
<br>
<br>
<br>
<br>
<br>
<br>

    output_vel
&rarr; Velocity obtain from formula (vel_function)
    
    default_vel ** 
&rarr; Velocity when  there is no object entering irradiating zone 
    
    conveyor_vel
&rarr; Velocity for testing conveyor belt when output velocity is too low for motor to process, usually x10 of output_vel 








### Additional setting 
<p>
For scenario that the objects move from left to right 

For scenario that the objects move from right to left 
Please fix the code as follow ‘if mango_list_xyxy[i][0] <= irradiating_zone[0][0] and mango_list_xyxy[i][2] >= irradiating_zone[0][0]:’
</p>



