import pyfirmata 
import serial

# ลง package ต่างๆข้างบนก่อน


# ส่วนของการเชื่อมต่อกับ Arduino เปิดเมื่อเชื่อมต่อกับ Arduino แล้ว
# สร้าง object ของ class pyfirmata.Arduino โดยใช้ชื่อ board

board = pyfirmata.Arduino('COM3')
print("Communication Successfully started")
speed = board.digital[9]
speed.mode = pyfirmata.PWM

def speedCon(speeds):
    #speeds mm/s
    rpm = speeds/0.1353
    pwm = (rpm/800)/5
    speed.write(pwm) 

speedCon(0)