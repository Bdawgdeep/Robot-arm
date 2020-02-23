## This is the code of calculating rotating angles.
## The functionality is only using Inverse Kinematics, other two are just for testing functionality of service.
## In the end, we converts rotating angles to rotating degrees then send those six values to arduino.
## copyright to Yanrui Wang, Yazhi Fan

import os, sys, time
import numpy as np
import RPi.GPIO as GPIO
import math
from math import pi
import serial

import FK, VK, IK

ArduinoCAM=serial.Serial('/dev/ttyACM0',9600)
ArduinoMOT=serial.Serial('/dev/ttyACM1',9600)
time.sleep(5)
s=[0,1]



# Create initial motor states and variable space
pos_offset = [0, 0, 5]
servo_ID1 = 0.0
servo_degree = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

## system flag-terminates the loop when false
sysRunning_flag = True    # system Running flag
type_flag = True




def GPIO27_callback(channel):
    print ("")
    print("Button 27 pressed...")
    global sysRunning_flag
    sysRunning_flag = False
    print("System shut down")
    
GPIO.setmode(GPIO.BCM)   #set up GPIO pins
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)

## add callback event
GPIO.add_event_detect(27, GPIO.FALLING, callback=GPIO27_callback, bouncetime=300)





try:
    # if sysRunning_flag is True, enter the while loop
    while (sysRunning_flag):
        time.sleep(0.02)
        # Start sweeping the space, sends command to Arduino Motor
        ArduinoMOT.write('S')
        time.sleep(10)
        #Take picture and get XYZ from Arduino Camera
        ArduinoCAM.write('A')
        Ser_read=ArduinoCAM.readline()

        #Make choices based on the results from the camera.

        if Ser_read >0:

            #Start sweeping the space, sends command to Arduino Motor
            ArduinoMOT.write('S')

            #s1 = serial.Serial('/dev/ttyACM0', 9600)
            #s1.flushInput()
            length = len(Ser_read)
            for i in range(0, length):
                a = np.fromstring((Ser_read[i]), dtype=float, sep=' ')
                 # goal_position = [a[0], a[1], a[2]]    ## x, y, z should be converted to meters, like 0.025
                goal_position = [a[0], a[1], a[2]]
                print("Goal position read: ", goal_position)

                # implement forward kinematics from FK.py file
                current_angles = [0, 0, 0, 0, 0]
                test_angle = [pi / 4, pi / 4, pi / 4, pi / 4,0]
                FK_result = FK.fk_srv(test_angle)
                print(FK_result)
                print("above is FK")

                # implement forward kinematics from VK.py file
                jac = VK.vk_srv(current_angles)
                print(jac)
                print("above is VK")


                ## implement Inverse Kinematics from IK.py file
                ##goal_position = [FK_result[0, 3], FK_result[1, 3], FK_result[2, 3]]
                #goal_position = [0.175, 0, 0.08]
                rotating_angle = list(IK.ik_srv(goal_position))
                rotating_angle.append(servo_ID1)
                #rotating_angle[4] = -(rotating_angle[1] + rotating_angle[2] + rotating_angle[3]) + (pi / 2)
                rotating_angle[4] = 0
                print("rotating angle:", rotating_angle)


                # Convert the inverse kinematics results to degrees for arduino
                for i in range(0, 3):
                    servo_degree[i] = int(-(((rotating_angle[i] * 360) / (2 * pi)) / 0.24) - 0.5)

                servo_degree[3] = int((((rotating_angle[3] * 360) / (2 * pi)) / 0.24) + 0.5)
                servo_degree[4] = int((((rotating_angle[4] * 360) / (2 * pi)) / 0.24) + 0.5)
                servo_degree[5] = int((((rotating_angle[5] * 360) / (2 * pi)) / 0.24) + 0.5)
                print("servo degree:", servo_degree)



            ## serial communication to arduino. Sends the six goal target angles for the servo motors

            ArduinoMOT.write('P')
            ArduinoMOT.write(servo_degree[0])
            ArduinoMOT.write(servo_degree[1])
            ArduinoMOT.write(servo_degree[2])
            ArduinoMOT.write(servo_degree[3])
            ArduinoMOT.write(servo_degree[4])
            ArduinoMOT.write(servo_degree[5])
        #     count = 0
        #     comp_list = ["Completed\r\n", "Hello Pi, This is Arduino UNO...:\r\n", "All completed\r\n"]
        #     done_signal = ["done\r\n"]
        #     while count < 6:
        #         if s1.inWaiting()>0:
        #             inputValue = s1.readline()
        #             print(inputValue)
        #             if inputValue in comp_list:
        #                 try:
        #                     n = servo_degree[count]
        #                     print("Pi's pos and number:",count,n)
        #                     s1.write('%d'%n)
        #                     count = count+1
        #                 except:
        #                     print("Input error, please input a number")
        #                     s1.write('0')
        #
        #     inputValue = s1.readline()
        #     while inputValue not in done_signal:
        #         inputValue = s1.readline()

        sysRunning_flag = False

    else:
        ArduinoMOT.write('S')

except KeyboardInterrupt:
    GPIO.cleanup() # clean up GPIO on CTRL+C exit

print("exit")
GPIO.cleanup()
