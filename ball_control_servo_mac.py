import multiprocessing as mp
import numpy as np
from multiprocessing import Queue
import cvzone
from cvzone.ColorModule import ColorFinder
import cv2
import serial
import math
import time


class PID():


    def __init__(self, Kp, Ki, Kd, SP = 0, init_output = 0, use_limit = False, min = 0, max = 100):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.SP = SP
        self.output = init_output
        self.P = 0
        self.I = 0
        self.D = 0
        self.prev_time = time.time()
        self.min = min
        self.max = max
        self.limit = use_limit
    

    def update(self, PV):
        self.current_time = time.time()
        self.time_delta = self.current_time - self.prev_time
        self.prev_time = self.time_delta

        self.error = PV - self.SP

        self.P = self.Kp * self.error
        self.I = self.I + self.Ki * self.error * self.time_delta
        self.D = self.error / self.time_delta

        self.output = self.P + self.I + self.D
        if self.limit: self.output = limit(self.output, self.min, self.max)

        return self.output


# -------------------------------------------Both programs(Servo Control and Ball Tracker) in one -------------------------------------------
"""
For running both programs simultaneously we can use multithreading or multiprocessing
"""

# PHYSICAL CONSTANTS
L = 220  # Distance between two servos
r = 40  # Length of servo shaft
s = 15  # Length of actuator leg
d = 0   # Offset in Y direction

use_gui_angles = False


# define servo angles and set a value
servo1_angle = 0
servo2_angle = 0
servo3_angle = 0
all_angle = 0

platform_roll_angle = 0
platform_pitch_angle = 0

PLATFORM_ROLL_LOW_LIM = -10
PLATFORM_ROLL_HIGH_LIM = 10
PLATFORM_PITCH_LOW_LIM = -10
PLATFORM_PITCH_HIGH_LIM = 10

# Set a limit to upto which you want to rotate the servos (You can do it according to your needs)
servo1_angle_limit_positive = 10
servo1_angle_limit_negative = -70

servo2_angle_limit_positive = 10
servo2_angle_limit_negative = -70

servo3_angle_limit_positive = 10
servo3_angle_limit_negative = -70

Kp = 1
Ki = 0
Kd = 0

PID_servo1 = PID(Kp, Ki, Kd)
PID_servo2 = PID(Kp, Ki, Kd)
PID_servo3 = PID(Kp, Ki, Kd)

PID_roll = PID(Kp, Ki, Kd)
PID_pitch = PID(Kp, Ki, Kd)





def limit(in_val, min, max):
    out = in_val
    if in_val > max: out = max
    elif in_val < min: out = min
    return out

def ball_track(key1, queue):
    camera_port = 0
    cap = cv2.VideoCapture(camera_port)
    cap.set(3, 1280)
    cap.set(4, 720)

    get, img = cap.read()
    h, w, _ = img.shape

    if key1:
        print('Ball tracking is initiated')

    myColorFinder = ColorFinder(False)  # if you want to find the color and calibrate the program we use this *(Debugging)
    # hsvVals = {'hmin': 0, 'smin': 65, 'vmin': 219, 'hmax': 179, 'smax': 255, 'vmax': 255}
    hsvVals = {'hmin': 58, 'smin': 31, 'vmin': 100, 'hmax': 162, 'smax': 255, 'vmax': 255} # Green

    center_point = [626, 337, 2210] # center point of the plate, calibrated


    while True:
        start_time = time.time() # start recording the cycle time

        get, img = cap.read()

        center = (center_point[0], center_point[1])
        radius = 300

        mask = np.zeros_like(img)
        cv2.circle(mask, center, radius, (255, 255, 255), -1)

        masked_img = cv2.bitwise_and(img, mask)

        imgColor, mask = myColorFinder.update(masked_img, hsvVals)

        imgContour, countours = cvzone.findContours(masked_img, mask)

        if countours:

            data = round((countours[0]['center'][0] - center_point[0]) / 10), \
                   round((h - countours[0]['center'][1] - center_point[1]) / 10), \
                   round(int(countours[0]['area'] - center_point[2])/100)

            queue.put(data)
            #print("The got coordinates for the ball are :", data)
        else:
            data = 'nil' # returns nil if we cant find the ball
            queue.put(data)

        imgStack = cvzone.stackImages([imgContour], 1, 1)
        # imgStack = cvzone.stackImages([img,imgColor, mask, imgContour],2,0.5) #use for calibration and correction
        cv2.imshow("Image", imgStack)

        end_time = time.time() # stop recording the cycle time
        cycle_time = end_time - start_time # calculate the cycle time
        print(f"Ball tracking cycle time: {cycle_time} seconds") # print the cycle time

        cv2.waitKey(1)


def servo_control(key2, queue):
    #port_id = 'COM7'        
    port_id = '/dev/cu.usbmodem1101'
    # initialise serial interface
    arduino = serial.Serial(port=port_id, baudrate=250000, timeout=0.1)
    if key2:
        print('Servo controls are initiated')


    def all_angle_assign(angle_passed1,angle_passed2,angle_passed3):
        global servo1_angle, servo2_angle, servo3_angle
        servo1_angle = math.radians(float(angle_passed1))
        servo2_angle = math.radians(float(angle_passed2))
        servo3_angle = math.radians(float(angle_passed3))
        write_servo()

    

    def writeCoord():
        """
        Here in this function we get both coordinate and servo control, it is an ideal place to implement the controller
        """
        corrd_info = queue.get()

        if corrd_info == 'nil': # Checks if the output is nil
            print('Cannot find the ball :(')
        else:
            print(f'The position of the ball : [{corrd_info[0]}, {corrd_info[1]}]')

            # PID


            roll_angle = PID_roll.update(corrd_info[0])
            pitch_angle = PID_pitch.update(corrd_info[1])

            roll_angle = limit(roll_angle, PLATFORM_ROLL_LOW_LIM, PLATFORM_ROLL_HIGH_LIM)
            pitch_angle = limit(pitch_angle, PLATFORM_PITCH_LOW_LIM, PLATFORM_PITCH_HIGH_LIM)

            roll_angle = -math.radians(roll_angle)
            pitch_angle = -math.radians(pitch_angle)


            # For using the GUI angles
            if use_gui_angles:
                roll_angle = platform_roll_angle
                pitch_angle = platform_pitch_angle

            print(f'Platform angles are : [{roll_angle}, {pitch_angle}]')
            

            # Calculate Z-position for each servo.
            # The z-value is all we need to find a good enough approximations for the angle of the servo
            global L, d
            z_S1 = ((math.sqrt(3)*L/6 + d)*math.sin(pitch_angle)*math.cos(roll_angle) + L/2*math.sin(roll_angle))
            z_S2 = ((math.sqrt(3)*L/6 + d)*math.sin(pitch_angle)*math.cos(roll_angle) - L/2*math.sin(roll_angle))
            z_S3 = ((-math.sqrt(3)*L/3 + d)*math.sin(pitch_angle)*math.cos(roll_angle))

            print(f'Z-values are: [{z_S1}, {z_S2}, {z_S3}]')


            #Approximate each servo angle based on the calculated z-offset
            global r
            ang1 = math.asin(z_S1/r)
            ang2 = math.asin(z_S2/r)
            ang3 = math.asin(z_S3/r)

            ang1 = math.degrees(ang1)
            ang2 = math.degrees(ang2)
            ang3 = math.degrees(ang3)

            ang1 = limit(ang1, servo1_angle_limit_negative, servo1_angle_limit_positive)
            ang2 = limit(ang2, servo2_angle_limit_negative, servo2_angle_limit_positive)
            ang3 = limit(ang3, servo3_angle_limit_negative, servo3_angle_limit_positive)

            all_angle_assign(ang1, ang2, ang3)
        

    def write_arduino(data):
        print('The angles send to the arduino : ', data)

        arduino.write(bytes(data, 'utf-8'))

    def write_servo():
        ang1 = servo1_angle
        ang2 = servo2_angle
        ang3 = servo3_angle

        angles: tuple = (round(math.degrees(ang1), 1),
                         round(math.degrees(ang2), 1),
                         round(math.degrees(ang3), 1))

        write_arduino(str(angles))



    while key2:
        start_time = time.time()
        writeCoord()
        end_time = time.time() # stop recording the cycle time
        cycle_time = end_time - start_time # calculate the cycle time
        print(f"Servo cycle time: {cycle_time} seconds") # print the cycle time

    
    #root.mainloop()  # running loop

if __name__ == '__main__':
    queue = Queue() # The queue is done inorder for the communication between the two processes.
    key1 = 1 # just two dummy arguments passed for the processes
    key2 = 2
    p1 = mp.Process(target= ball_track, args=(key1, queue)) # initiate ball tracking process
    p2 = mp.Process(target=servo_control,args=(key2, queue)) # initiate servo controls
    p1.start()
    p2.start()
    p1.join()
    p2.join()