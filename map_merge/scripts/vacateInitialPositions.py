#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid

mapData = OccupancyGrid()
copiedMapData = OccupancyGrid()

def mapCallback(data):
    global mapData
    mapData = data

global dataList

def vacate(dataList, x, y, width):
    for i in range(13):
        for j in range(13):
            dataList[int((x-5+i)*width + y-5+j)] = 0

def vacate_node():
    global mapData, dataList
    rospy.init_node('vacate', anonymous = False)
    rospy.Subscriber('/map', OccupancyGrid, mapCallback)
    pub = rospy.Publisher('/map', OccupancyGrid, queue_size = 10)

    while not rospy.is_shutdown():
        if len(mapData.data) > 0:
            res = mapData.info.resolution
            width = mapData.info.width
            height  = mapData.info.height
            ox = mapData.info.origin.position.x
            oy = mapData.info.origin.position.y

            volta1x = 0
            volta1y = 0
            volta2x = 0
            volta2y = 0

            volta1x_pixel = int(abs(volta1x - ox)/res)
            volta1y_pixel = int(abs(volta1y - oy)/res)
            volta2x_pixel = int(abs(volta2x - ox)/res)
            volta2y_pixel = int(abs(volta2y - oy)/res)

            copiedMapData = mapData
            dataList = list(copiedMapData.data)

            vacate(dataList, volta1y_pixel, volta1x_pixel, width)
            vacate(dataList, volta2y_pixel, volta2x_pixel, width)

            copiedMapData.data = tuple(dataList)
            copiedMapData.header.stamp = rospy.Time.now()
            pub.publish(copiedMapData)
        break

if __name__ == "__main__":
    try:
        vacate_node()
    except rospy.ROSInterruptException:
        pass
