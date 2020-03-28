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

file = open("/home/jefferson/webots/datasetLidar/Poste1.txt", "w")
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

#lê bussola
cont = 0
angleStart = 0
angle = 0
tetaDegree = 0
teta = 0

while robot.step(TIME_STEP) != -1:
    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed = -0.05 * MAX_SPEED
    rightSpeed = 0.05 * MAX_SPEED

    # set up the motor speeds at x% of the MAX_SPEED.
    leftMotorFront.setVelocity(leftSpeed)
    rightMotorFront.setVelocity(rightSpeed)
    leftMotorBack.setVelocity(leftSpeed)
    rightMotorBack.setVelocity(rightSpeed)


    cloud = Lidar.getPointCloud(lms291)

    north = Compass.getValues(mag)
    angle = math.atan2(north[0], north[2]) * 180 / math.pi

    if angle < 0:
        angle = angle + 360

    phi = (angle * math.pi) / 180
    print(angle)

    a = []
    b = []
    c = []
    for i in range(0, 719):
        z = cloud[i].x
        x = cloud[i].y
        y = cloud[i].z

        # https://en.wikipedia.org/wiki/Spherical_coordinate_system
        # https://mathinsight.org/applet/cartesian_coordinates_point_3d
        x2 = x*x
        y2 = y*y
        z2 = z*z
        r = math.sqrt(x2 + y2 + z2)
        x = r * math.cos(phi)
        y = r * math.sin(phi)
        # z = r * math.cos(teta)

        # array
        a = np.append(a, x)
        b = np.append(b, y)
        c = np.append(c, z)

        file.write("%f %f %f\n" % (x, y, z))
        tetaDegree = tetaDegree + 0.5
        teta = (tetaDegree * math.pi) / 180

    if cont == 0:
        angleStart = int(angle) + 1
        cont = cont + 1
    elif angleStart == int(angle):
        print("terminou!")
        Lidar.disable(lms291)
        print('Lidar disabled')
        break

file.close()
