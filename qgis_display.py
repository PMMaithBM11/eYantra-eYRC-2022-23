#!/usr/bin/env python3

# Importing the required libraries

from edrone_client.msg import *
from sentinel_drone.msg import Geolocation
import cv2 as cv
import numpy as np
import rospy
import time
from osgeo import gdal
import os
from qgis.utils import iface
from qgis.core import QgsPointXY
from qgis.gui import QgsVertexMarker
from PyQt5.QtGui import QColor
from decimal import Decimal


canvas = iface.mapCanvas()
def plot(p):
    pnt = QgsPointXY(p[0], p[1])
    m = QgsVertexMarker(canvas)
    m.setCenter(pnt)
    m.setColor(QColor('Black'))
    m.setIconType(QgsVertexMarker.ICON_CIRCLE)
    m.setIconSize(12)
    m.setPenWidth(1)
    m.setFillColor(QColor(0, 200, 0))


pathoftxt = r"/home/mohit/Downloads/latlongs.txt"
while not rospy.is_shutdown():
    time.sleep(2)
    file1 = open(pathoftxt, "r")
    st = file1.read()
    file1.close()
    st = st.split(" ")
    # print(st)
    for i in range(0,len(st)-1,2):
        p=(float(st[i]),float(st[i+1]))
        print(p)
        plot(p)
    if(len(st)==7):
        break
# time.sleep(30)
# run line below to remove:
# canvas.scene().removeItem(m)
