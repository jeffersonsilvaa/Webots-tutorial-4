# **************************************************************
# Project 04 - Disciplina de robótica Móvel UFC / IFCE / LAPISCO
#       Simulação 01 com robô Pioneer 3AT - Webots R2020a
#     Distance sensors - Lidar Depth Map (Sensor na vertical)
#        Python 3.6 na IDE Pycharm - controller <extern>
#                By: Jefferson Silva Almeida
#                       Data: 26/01/2020
# **************************************************************

from controller import Robot
from controller import Motor
from controller import DistanceSensor
from controller import Lidar
from controller import LidarPoint
# import cv2
# import matplotlib.pyplot as plt
import numpy as np
import math
from controller import Gyro
from controller import Compass

file = open("/home/jefferson/webots/pointsLidar.txt", "w")
# file.write("teste")
# file.close()

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

# maximal value returned by the sensors
MAX_SENSOR_VALUE = 1024
# minimal distance, in meters, for an obstacle to be considered
MIN_DISTANCE = 1.0

# create the robot instance
robot = Robot()

# init lidar lms291
lms291 = robot.getLidar('Sick LMS 291')
print(lms291)
Lidar.enable(lms291, TIME_STEP)
Lidar.enablePointCloud(lms291)
print('Lidar enabled')
lms291_width = Lidar.getHorizontalResolution(lms291)
print(lms291_width)
# half_width = lms291_width / 2
# max_range = Lidar.getMaxRange(lms291)
# num_points = Lidar.getNumberOfPoints(lms291)

#gyro
gyro = robot.getGyro("gyro")
Gyro.enable(gyro, TIME_STEP)

#bússola
mag = robot.getCompass("compass")
Compass.enable(mag, TIME_STEP)

# Lidar.disable(lms291)
print('Lidar disabled')

# get a handler to the motors and set target position to infinity (speed control)
leftMotorFront = robot.getMotor('front left wheel')
rightMotorFront = robot.getMotor('front right wheel')
leftMotorBack = robot.getMotor('back left wheel')
rightMotorBack = robot.getMotor('back right wheel')

leftMotorFront.setPosition(float('inf'))
rightMotorFront.setPosition(float('inf'))
leftMotorBack.setPosition(float('inf'))
rightMotorBack.setPosition(float('inf'))

#lê o giroscopio
angleYOld = 0

while robot.step(TIME_STEP) != -1:
    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed = -0.05 * MAX_SPEED
    rightSpeed = 0.05 * MAX_SPEED

    # set up the motor speeds at x% of the MAX_SPEED.
    leftMotorFront.setVelocity(leftSpeed)
    rightMotorFront.setVelocity(rightSpeed)
    leftMotorBack.setVelocity(leftSpeed)
    rightMotorBack.setVelocity(rightSpeed)

    # read Lidar
    # lms291_values = []
    # lms291_values = Lidar.getRangeImage(lms291)

    cloud = Lidar.getPointCloud(lms291)

    #lê o giroscopio, eixo que o robô gira
    # angle = Gyro.getValues(gyro)
    # angleY = angle[1]
    # angleY = angleY * 180 / math.pi

    #pega ..
    # angleYabsRad = (angleY + angleYOld)
    # angleYOld = angleYabsRad
    # print(angleYabsRad)

    north = Compass.getValues(mag)
    angle = math.atan2(north[0], north[2])
    print(angle)

    a = []
    b = []
    c = []
    for i in range(0, 179):
        x = cloud[i].x
        y = cloud[i].y
        z = cloud[i].z

        #X-axis Rotation
        # https://www.cs.helsinki.fi/group/goa/mallinnus/3dtransf/3drot.html
        y = (y * math.cos(angle)) - (z * math.sin(angle))
        z = (y * math.sin(angle)) + (z * math.cos(angle))

        # array
        a = np.append(a, x)
        b = np.append(b, y)
        c = np.append(c, z)

        file.write("%f %f %f\n" % (x, y, z))
        # file.write(a + " " + b + " " + c + "\n")


file.close()
