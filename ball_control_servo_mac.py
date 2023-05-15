# Rydde i kode
# Flytte setpunkt til midten
# Tracking på ballen
# f_c kan være 12.5% av delta_t

import multiprocessing as mp
import numpy as np
from multiprocessing import Queue
import cvzone
from cvzone.ColorModule import ColorFinder
import cv2
import serial
import math
import time

# ==============================PID CLASS==============================
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

# ==============================Servo controll and ball tracker==============================
"""
For running both programs simultaneously we can use multithreading or multiprocessing
"""
# Parameters for setpoint pattern
radius_setpoint = 170       # 100
angle_increment = 0.5       # 1.5

# Controller gains
L1 = 11.8
L2 = -100
K1 = 0.1 #0.01
K2 = 0.3

# PHYSICAL CONSTANTS
L = 220  # Distance between two servo attachement points to platform
r = 40  # Length of servo shaft
s = 15  # Length of actuator leg
d = 0   # Offset in Y direction

# PID parameters
Kp = 0.4
Ki = 0.1
Kd = 0.2

# Time variables for making shape
prev_time = time.time()
prev_time_cv = time.time()

# Observer variables
phat_x_prev = 0
vhat_x_prev = 0
phat_y_prev = 0
vhat_y_prev = 0
p_x_prev = 0
p_y_prev = 0
u_p_prev = 0
u_r_prev = 0
x_ref = 0
y_ref = 0

use_gui_angles = False

# Define servo angles and set a value
servo1_angle = 0
servo2_angle = 0
servo3_angle = 0
all_angle = 0
platform_roll_angle = 0
platform_pitch_angle = 0

# Constraints of angles 
PLATFORM_ROLL_LOW_LIM = -13
PLATFORM_ROLL_HIGH_LIM = 13
PLATFORM_PITCH_LOW_LIM = -13
PLATFORM_PITCH_HIGH_LIM = 13

# Constraints of servo
servo1_angle_limit_positive = 13
servo1_angle_limit_negative = -70

servo2_angle_limit_positive = 10
servo2_angle_limit_negative = -70

servo3_angle_limit_positive = 10
servo3_angle_limit_negative = -70

# PID objects
PID_servo1 = PID(Kp, Ki, Kd)
PID_servo2 = PID(Kp, Ki, Kd)
PID_servo3 = PID(Kp, Ki, Kd)

PID_roll = PID(Kp, Ki, Kd)
PID_pitch = PID(Kp, Ki, Kd)

# Function for limiting angles
def limit(in_val, min, max):
    out = in_val
    if in_val > max: out = max
    elif in_val < min: out = min
    return out

def ball_track(key1, queue):
    camera_port = 0
    cap = cv2.VideoCapture(camera_port)
    w = 1280
    h = 720

    global prev_time_cv
    
    cap.set(3, w)
    cap.set(4, h)

    if key1:
        print('Ball tracking is initiated')

    myColorFinder = ColorFinder(False)  # if you want to find the color and calibrate the program we use this *(Debugging)
    hsvVals = {'hmin': 58, 'smin': 31, 'vmin': 100, 'hmax': 162, 'smax': 255, 'vmax': 255} # Green

    center_point = [600, 336, 2210] # center point of the plate, calibrated
    center_point_setpoint = [580, 390, 2210]


    center = (center_point[0], center_point[1])
    radius = 300
    angle = 18  # Specify the angle of rotation in degrees

    # Define the rotation matrix
    M = cv2.getRotationMatrix2D((w / 2, h / 2), angle, 1)

    while True:
        current_time_cv = time.time()
        delta_t = current_time_cv - prev_time_cv
        prev_time_cv = current_time_cv
        #print(delta_t)
        get, img = cap.read()
        img = cv2.warpAffine(img, M, (w, h))

        angle = current_time_cv*angle_increment
        
        # Shape of a circle
        #x = int(x_ref+radius_setpoint*math.cos(angle))
        #y = int(y_ref+radius_setpoint*math.sin(angle))
        
        # Shape of the number eight
        #x = int(x_ref + radius_setpoint/2 * math.cos(angle)) + int(radius_setpoint/2 * math.cos(2 * angle))
        #y = int(y_ref + radius_setpoint/2 * math.sin(angle)) + int(radius_setpoint/2 * math.sin(2 * angle))
        
        x = 0
        y = 0
        
        mask = np.zeros_like(img)
        cv2.circle(mask, center, radius, (255, 255, 255), -1)
        masked_img = cv2.bitwise_and(img, mask)
        imgColor, mask = myColorFinder.update(masked_img, hsvVals)
        imgContour, countours = cvzone.findContours(masked_img, mask)

        if countours:
            data = round((countours[0]['center'][0] - center_point_setpoint[0] + x) / 10), \
                   round((h - countours[0]['center'][1] - center_point_setpoint[1] + y) / 10), \
                   round(int(countours[0]['area'] - center_point_setpoint[2])/100)
            #print("The got coordinates for the ball are :", data)
        else:
            data = 'nil' # returns nil if we cant find the ball
            
        queue.put(data)
        imgStack = cvzone.stackImages([imgContour], 1, 1)
        # imgStack = cvzone.stackImages([img,imgColor, mask, imgContour],2,0.5) #use for calibration and correction
        cv2.imshow("Image", imgStack)
        cv2.waitKey(1)


def servo_control(key2, queue):       
    port_id =  '/dev/cu.usbmodem11101' # '/dev/cu.usbmodem1101'
    arduino = serial.Serial(port=port_id, baudrate=115200, timeout=0.1)
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
        global prev_time
        current_time = time.time()
        delta_t = current_time-prev_time

        corrd_info = queue.get()


        if corrd_info == 'nil': # Checks if the output is nil
            print('Cannot find the ball :(')
            #None == None
        
        else:
            #print(f'The position of the ball : [{corrd_info[0]}, {corrd_info[1]}]')
            '''
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
            '''

            # Observer LQR

            # Calculate the state estimate using the observer

            global phat_x_prev, vhat_x_prev, phat_y_prev, vhat_y_prev, L1, L2, K1, K2, p_x_prev, p_y_prev, x_ref, y_ref, u_r_prev, u_p_prev

            p_x = corrd_info[0]
            p_y = corrd_info[1]

            # Calculate estimates
            a_x = 7*u_r_prev
            a_y = 7*u_p_prev
            phat_x = phat_x_prev + delta_t*(vhat_x_prev) + L1*(p_x_prev - phat_x_prev)
            #vhat_x = vhat_x_prev + delta_t*(a_x) + L2*(p_x_prev - phat_x_prev)
            phat_y = phat_y_prev + delta_t*(vhat_y_prev) + L1*(p_y_prev - phat_y_prev)
            #vhat_y = vhat_y_prev + delta_t*(a_y) + L2*(p_y_prev - phat_y_prev)
            

            #print(p_x, p_y)

            #'''----------
            # High pass (Oppgave 3B)
            f_c = 1 # ((1/delta_t) / 100) * 12 # 1.5 # hz
            T = 1/(2*math.pi*f_c)
            vhat_x = (1- delta_t/T)*vhat_x_prev + (p_x - p_x_prev)/T
            vhat_y = (1- delta_t/T)*vhat_y_prev + (p_y - p_y_prev)/T
            #-----------'''

            # Vei-fart-tid (Oppgave 3A) 
            # vhat_x = (p_x - p_x_prev)/delta_t
            # vhat_y = (p_y - p_y_prev)/delta_t
            
            #print(phat_x, vhat_x, phat_y, vhat_y)

            # Calculate the control input using the LQR controller
            u_r = -K1*(p_x-x_ref) - K2*vhat_x
            u_p = -K1*(p_y-y_ref) - K2*vhat_y
            #print(f"Raw u: {u_r}, {u_p}")

                    
            roll_angle = u_r  #math.degrees(u_r)
            pitch_angle = u_p #math.degrees(u_p)

            roll_angle = limit(roll_angle, PLATFORM_ROLL_LOW_LIM, PLATFORM_ROLL_HIGH_LIM)
            pitch_angle = limit(pitch_angle, PLATFORM_PITCH_LOW_LIM, PLATFORM_PITCH_HIGH_LIM)

            #print(f'Platform: [{roll_angle}, {pitch_angle}]')

            roll_angle = math.radians(roll_angle)
            pitch_angle = math.radians(pitch_angle)
            
            u_r_prev = roll_angle
            u_p_prev = pitch_angle

            # Update previous values
            phat_x_prev = phat_x
            vhat_x_prev = vhat_x
            phat_y_prev = phat_y
            vhat_y_prev = vhat_y
            p_x_prev = p_x
            p_y_prev = p_y
            prev_time = current_time

            # Calculate Z-position for each servo.
            # The z-value is all we need to find a good enough approximations for the angle of the servo
            global L, d
            z_S1 = ((math.sqrt(3)*L/6 + d)*math.sin(pitch_angle)*math.cos(roll_angle) + L/2*math.sin(roll_angle))
            z_S2 = ((math.sqrt(3)*L/6 + d)*math.sin(pitch_angle)*math.cos(roll_angle) - L/2*math.sin(roll_angle))
            z_S3 = ((-math.sqrt(3)*L/3 + d)*math.sin(pitch_angle)*math.cos(roll_angle))

            #print(f'Z-values: [{z_S1}, {z_S2}, {z_S3}]')


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
        print('Servo angles: ', data)

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
        writeCoord()

    
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

