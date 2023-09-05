import cv2
import math
import time
from realsense_camera import *
import numpy as np
from ultralytics import YOLO

import pyfirmata 
import serial


board = pyfirmata.Arduino('COM3')
print("Communication Successfully started")
speed = board.digital[9]
speed.mode = pyfirmata.PWM


def speedConveyor(speed):
    #speed in mm/s
    rpm = speed/0.1353
    pwm = (rpm/800)/5
    speed.write(pwm) 




# set default value for irradiating zone
irradiating_zone =  [(1000,0), (1000,720)]

#set default value for calibration
camera_dist_mm = 547
mm_px_ratio = 0.1353

default_vel = 150
def vel_function(width,depth):
  Dd             = 2                #depth dose, kGy : kJol/kg
  d              = 8                #depth, cm
  dens           = 0.001            #density?, Kg/m3
  um             = 0.03e3           #cm2/kg
  #Dsf           = 1                #surface dose, kGy : kJol/kg
  Ibeam          = 171e-3           #A
  E              = 5.1e6            #eV
  RepRate        = 200
  PulseWidth     = 5e-6             #sec
  P    =  Ibeam*E *RepRate*PulseWidth*1e-3    # electron beam power  
  Dsf = Dd*math.exp(um*dens*depth/10)         # surface dose, kGy : kJol/kg
  v = 0.085*((um*P)/(Dsf*width/10))
  v_mms = v*10

  return v_mms


rs = RealsenseCamera()

model_path = "C:/Users/eye18/OneDrive/Desktop/SLRI_Belt_Aug/yolov8x_best_1050.pt"
model = YOLO(model_path)

speed_array = []
time_array = []

prev_time = time.time()

while (True):
    
    # Calculate time elapsed
    curr_time = time.time()
    elapsed_time = curr_time - prev_time

    if elapsed_time >= 1:
        print("Elapsed time: ", elapsed_time)
        print("Capturing . . .")
        ret, bgr_frame, depth_frame = rs.get_frame_stream()
        if ret:
            
            # Detect object 
            results = model(bgr_frame,augment=False,max_det=10,
                      conf=0.4,iou=0.6,device=0,half=True,verbose=False)[0]

            # set to center of screen when no detection
            irradiating_border_top = int(720/2)
            irradiating_border_bottom = int(720/2)

            if len(results.boxes) != 0 :
                mango_list_xywh = results.boxes.xywh.to('cpu').numpy().astype(int)
                mango_list_xyxy = results.boxes.xyxy.to('cpu').numpy().astype(int)
                depth_mm = []

                
                compare_top = 720
                compare_bottom = 0
                for i in range (len(mango_list_xyxy)):
                    # if object is in irradiating zone
                    if mango_list_xyxy[i][0] <= irradiating_zone[0][0] and mango_list_xyxy[i][2] >= irradiating_zone[0][0]:
                        #if object top is highest
                        if mango_list_xyxy[i][1] < compare_top:
                            irradiating_border_top = mango_list_xyxy[i][1]
                            compare_top = irradiating_border_top
                            
                        #if object bottom is lowest
                        if mango_list_xyxy[i][3] > compare_bottom:
                            irradiating_border_bottom = mango_list_xyxy[i][3]
                            compare_bottom = irradiating_border_bottom

                        depth_mm.append(camera_dist_mm- depth_frame[mango_list_xywh[i][1],mango_list_xywh[i][0]])
                        print("depth_mm : ",depth_mm)
                        print("i : ",i)
                        cv2.putText(bgr_frame, "Depth {} mm".format(mango_list_xywh[i][1],mango_list_xywh[i][0]), 
                                            (mango_list_xywh[i][0],mango_list_xywh[i][1]), 
                                            cv2.FONT_HERSHEY_PLAIN, 2, 
                                            (100, 200, 0), 2)
                        
                        irradiating_width = irradiating_border_bottom-irradiating_border_top
                        print("irradiating_width : ",irradiating_width)
                        
                        if len(depth_mm) <= 1 :
                            avg_depth_mm = depth_mm[0]
                            print("avg_depth_mm : ",avg_depth_mm)
                        else :
                            avg_depth_mm = np.mean(depth_mm)
                            print("avg_depth_mm : ",avg_depth_mm)
                   
                    else :
                        irradiating_width = 0
                        avg_depth_mm = 0

            else: 
                irradiating_width = 0
                avg_depth_mm = 0


            if irradiating_width == 0:
                output_vel = default_vel
                # speedConveyorveyor(output_vel)
                speed_array.append(output_vel)

                conveyor_vel = output_vel
                speedConveyor(conveyor_vel)
                
                # print("output_vel : ",output_vel)
                # print("conveyor_vel : ",output_vel)
            else :
                output_vel = vel_function(irradiating_width*mm_px_ratio,avg_depth_mm/10)
                # speedConveyor(output_vel)
                speed_array.append(output_vel)
                
                conveyor_vel = output_vel*10
                speedConveyor(conveyor_vel)

                # print("output_vel : ",output_vel)
                # print("conveyor_vel : ",conveyor_vel)


            speed_array.append(output_vel)
            time_array.append(time.time())

            
            #irradiating zone
            cv2.line(bgr_frame, irradiating_zone[0], irradiating_zone[1], (0, 255, 0), 2)

            cv2.circle(bgr_frame, (900, 360), 3, (255,0,0),5)
            cv2.putText(bgr_frame, "measuring depth {} mm".format(depth_frame[360][900]), 
                                                        (500,360), 
                                                        cv2.FONT_HERSHEY_PLAIN, 2, 
                                                        (100, 200, 0), 2)

            #irradiating border
            cv2.circle(bgr_frame, (irradiating_zone[0][0], irradiating_border_top), 3, (0,0,255),15)
            cv2.circle(bgr_frame, (irradiating_zone[0][0], irradiating_border_bottom), 3, (0,0,255),15)
            cv2.putText(bgr_frame, "Irradiating Width".format(), 
                                                        (0,50), 
                                                        cv2.FONT_HERSHEY_PLAIN, 2, 
                                                        (100, 200, 0), 2)
            cv2.putText(bgr_frame, "{} mm".format(irradiating_width), 
                                                        (0,80), 
                                                        cv2.FONT_HERSHEY_PLAIN, 2, 
                                                        (100, 200, 0), 2)
         
         
            cv2.putText(bgr_frame, "Avg depth".format(), 
                                                        (0,130), 
                                                        cv2.FONT_HERSHEY_PLAIN, 2, 
                                                        (100, 200, 0), 2)
            cv2.putText(bgr_frame, "{} mm".format(avg_depth_mm), 
                                                        (0,170), 
                                                        cv2.FONT_HERSHEY_PLAIN, 2, 
                                                        (100, 200, 0), 2)
          
          
            cv2.putText(bgr_frame, "calculated speed".format(), 
                                                        (0,220), 
                                                        cv2.FONT_HERSHEY_PLAIN, 2, 
                                                        (100, 200, 0), 2)
            cv2.putText(bgr_frame, "{} mm/s".format(output_vel), 
                                                        (0,250), 
                                                        cv2.FONT_HERSHEY_PLAIN, 2, 
                                                        (100, 200, 0), 2)
            
          
            cv2.putText(bgr_frame, "conveyor speed".format(), 
                                                        (0,350), 
                                                        cv2.FONT_HERSHEY_PLAIN, 2, 
                                                        (100, 200, 0), 2)
            cv2.putText(bgr_frame, "{} mm/s".format(conveyor_vel), 
                                                        (0,380), 
                                                        cv2.FONT_HERSHEY_PLAIN, 2, 
                                                        (100, 200, 0), 2)
            

            cv2.putText(bgr_frame,"Calibration Ratio".format(), 
                                                        (0,480), 
                                                        cv2.FONT_HERSHEY_PLAIN, 2, 
                                                        (100, 200, 0), 2)
            cv2.putText(bgr_frame,"{} mm/px".format(round(mm_px_ratio,3)), 
                                                        (0,510), 
                                                        cv2.FONT_HERSHEY_PLAIN, 2, 
                                                        (100, 200, 0), 2)
            

            cv2.putText(bgr_frame,"Cam-Belt Distance".format(), 
                                                        (0,560), 
                                                        cv2.FONT_HERSHEY_PLAIN, 2, 
                                                        (100, 200, 0), 2)
            cv2.putText(bgr_frame,"Ref @ screen center".format(), 
                                                        (0,590), 
                                                        cv2.FONT_HERSHEY_PLAIN, 2, 
                                                        (100, 200, 0), 2)
            cv2.putText(bgr_frame,"{} mm".format(depth_frame[int(720/2)][int(1280/2)]), 
                                                        (0,620), 
                                                        cv2.FONT_HERSHEY_PLAIN, 2, 
                                                        (100, 200, 0), 2)
            
            
            cv2.imshow("YOLOv8", results.plot())
            # cv2.resize("Resized_Window", 300, 700)

            # press Esc key to exit
            if cv2.waitKey(1) & 0xFF == 27:
                break

        
        # uncomment if you want to capture every x second
        # prev_time = curr_time 

        print()
    
    # press Esc key to exit
    if cv2.waitKey(1) & 0xFF == 27:
        break

print("speed_array : ",speed_array)
print("time_array : ",time_array)
rs.release()
cv2.destroyAllWindows()


