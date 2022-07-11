#!/usr/bin/env python3
from objectron import objectron
from utils import transform_point, line3d, refine
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray

data = Float32MultiArray()

def getPoints():

    path1 = 'rtsp://admin:artpark123@192.168.0.220:554/Streaming/Channels/2/'
    camera1Pos = [0.26, -3.79, 3.146]
    lookup1Point = [0.26, 0.097, 0]
    camera1Fx = 489.17587686561023
    camera1Fy = 650.1313894912995
    dist_coeffs1 = [-0.44457107916298927, 0.2071676298596515, -0.002616430505351848, 0.002823234505856929, 0.02897236189496356]
    width = 640
    height = 480

    path2 = 'rtsp://admin:artpark123@192.168.0.221:554/Streaming/Channels/2/'
    camera2Pos = [0.79, 4.68, 3.080]
    lookup2Point = [-1.66, 3.2, 0]
    camera2Fx = 483.51868956325706
    camera2Fy = 642.5784805724196
    dist_coeffs2 = [-0.48434944088453896, 0.5356965932365025, 0.00019531977585342047, -0.0005455101387298529, -0.768169123553319]
    width = 640
    height = 480
    
    points1 = objectron(path1, width, height)
    points2 = objectron(path2, width, height)

    points = []
    numObjects = min(len(points1), len(points2))

    for i in range(numObjects):

        transformedPoint1 = []
        transformedPoint2 = []
        for j in range(9):
            transformedPoint1.append(transform_point(points1[i][j], camera1Pos, lookup1Point, camera1Fx, camera1Fy, width, height, dist_coeffs1))

        for j in range(9):
            transformedPoint2.append(transform_point(points2[i][j], camera2Pos, lookup2Point, camera2Fx, camera2Fy, width, height, dist_coeffs2))

        newPoints = []

        for j in range(9):
            lines = []
            lines.append(line3d(transformedPoint1[j], camera1Pos))
            lines.append(line3d(transformedPoint2[j], camera2Pos))
            newPoints.append(refine(lines))

        xmax = -np.inf
        xmin = np.inf
        ymax = -np.inf
        ymin = np.inf

        for j in range(9):
            if newPoints[j][0] > xmax:
                xmax = newPoints[j][0]
            if newPoints[j][0] < xmin:
                xmin = newPoints[j][0]
            if newPoints[j][1] > ymax:
                ymax = newPoints[j][1]
            if newPoints[j][1] < ymin:
                ymin = newPoints[j][1]
            length = xmax - xmin
            width = ymax - ymin

        points.append([newPoints[0][0], newPoints[0][1], length, width])

    return points[0]

def publish():
    global data
    pub = rospy.Publisher('objectron', Float32MultiArray, queue_size=10)
    rospy.init_node('objectron', anonymous=True)

    while not rospy.is_shutdown():
        points = getPoints()

        x = points[0]
        y = points[1]
        length = points[2]
        width = points[3]

        data.data = [x, y, length, width]
        pub.publish(data)
        # rospy.sleep(5)

if __name__ == "__main__":

    try:
        publish()
    except:
        rospy.ROSInterruptException