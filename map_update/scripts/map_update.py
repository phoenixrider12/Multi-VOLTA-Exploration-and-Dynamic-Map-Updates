#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray

mapData = OccupancyGrid()
copiedMapData = OccupancyGrid()
coordinatesArray = Float32MultiArray()

newX = 0
newY = 0
newLength = 0
newWidth = 0

oldX = 0
oldY = 0
oldLength = 0
oldWidth = 0

def mapCallback(data):
    global mapData
    mapData = data

def coordinatesCallback(data):
    global newX, newY, newLength, newWidth
    coordinatesArray = data
    newX = coordinatesArray.data[0]
    newY = coordinatesArray.data[1]
    newLength = coordinatesArray.data[2]
    newWidth = coordinatesArray.data[3]

global dataList

def insertObstacle(dataList, pxnew, pynew, pNewLength, pNewWidth, mapWidth, mapHeight):
    print('Inserted Obstacle')
    print(pxnew, pynew, pNewLength, pNewWidth)
    for i in range(pNewWidth-5):
        for j in range(pNewLength-5):
            dataList[int((pynew-(pNewWidth//2)+i)*mapWidth + pxnew-(pNewLength//2)+j)] = 100

def removeObstacle(dataList, pxold, pyold, pOldLength, pOldWidth, mapWidth, mapHeight):
    print('Deleted Obstacle')
    print(pxold, pyold, pOldLength, pOldWidth)
    for i in range(pOldWidth-5):
        for j in range(pOldLength-5):
            dataList[int((pyold-(pOldWidth//2)+i)*mapWidth + pxold-(pOldLength//2)+j)] = 0

def node():
    global mapData, dataList, newX, newY, newLength, newWidth, oldX, oldY, oldLength, oldWidth
    rospy.init_node('mapUpdate3D', anonymous=False)
    rospy.Subscriber('/objectron', Float32MultiArray, coordinatesCallback)
    rospy.Subscriber('/map', OccupancyGrid, mapCallback)
    pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
    updated = True

    while not rospy.is_shutdown():
        if len(mapData.data) > 0:
            res = mapData.info.resolution
            mapWidth = mapData.info.width
            mapHeight  = mapData.info.height
            ox = mapData.info.origin.position.x
            oy = mapData.info.origin.position.y

            dist = math.sqrt((newX - oldX)**2 + (newY - oldY)**2)
            # rospy.sleep(5)

            if dist > 0.5:
                if updated:
                    start_time = rospy.Time.now()
                time = rospy.Time.now()
                if time - start_time > rospy.Duration(60):
                    print(dist)
                    pxold = int(abs(oldX - ox)/res)
                    pyold = int(abs(oldY - oy)/res)
                    pxnew = int(abs(newX - ox)/res)
                    pynew = int(abs(newY - oy)/res)

                    pOldLength = int(oldLength/res)
                    pOldWidth = int(oldWidth/res)
                    pNewLength = int(newLength/res)
                    pNewWidth = int(newWidth/res)

                    copiedMapData = mapData
                    dataList = list(copiedMapData.data)

                    # print(pxold, pyold, pxnew, pynew, pOldLength, pOldWidth, pNewLength, pNewWidth)
                    insertObstacle(dataList, pxnew, pynew, pNewLength, pNewWidth, mapWidth, mapHeight)
                    removeObstacle(dataList, pxold, pyold, pOldLength, pOldWidth, mapWidth, mapHeight)
                    copiedMapData.data = tuple(dataList)
                    copiedMapData.header.stamp = rospy.Time.now()
                    pub.publish(copiedMapData)
                    print("Obstacle updated")

                    oldX = newX                
                    oldY = newY
                    oldLength = newLength
                    oldWidth = newWidth
                    updated = True
                else:
                    time = rospy.Time.now()
                    updated = False
            
if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
