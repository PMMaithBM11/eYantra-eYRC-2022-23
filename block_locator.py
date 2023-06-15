#!/usr/bin/env python3

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from sentinel_drone.msg import Geolocation
from pid_tune.msg import PidTune
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import rospy
import time
from osgeo import gdal
import os
import subprocess
import csv


class Edrone():
    """docstring for Edrone"""

    def __init__(self):

        # initializing ros node with name drone_control
        rospy.init_node('drone_control')

        # This corresponds to your current position of drone. This value must be updated each time in your whycon callback
        # [x,y,z]
        self.drone_position = [0.0, 0.0, 0.0]

        # [x_setpoint, y_setpoint, z_setpoint]
        # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
        self.setpoint = [2, 2, 20]

        # Declaring a cmd of message type edrone_msgs and initializing values
        self.cmd = edrone_msgs()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500

        # initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
        # after tuning and computing corresponding PID parameters, change the parameters
        # self.Kp = [19.68,19.68,31.44]
        # self.Ki = [0.0104,0.0104,0.1]
        # self.Kd = [764,764,596.1]
        # self.Kp=[23.58,23.58,31.44]
        # self.Ki=[0.00072,0.00072,0.1]
        # self.Kd=[764,764,596.1]
        self.Kp = [20, 20, 31.5]
        self.Ki = [0.0104, 0.0104, 0.1]
        self.Kd = [760, 760, 600]

        # -----------------------Add other required variables for pid here ----------------------------------------------
        self.throttle_error = 0
        self.throttle_prev_error = 0
        self.throttle_sum_error = 0
        self.min_throttle = 1000
        self.max_throttle = 2000

        self.roll_error = 0
        self.roll_prev_error = 0
        self.roll_sum_error = 0
        self.min_roll = 1000
        self.max_roll = 2000

        self.pitch_error = 0
        self.pitch_prev_error = 0
        self.pitch_sum_error = 0
        self.min_pitch = 1000
        self.max_pitch = 2000

        self.current_frame = None
        self.br = None
        self.set_pt = [320, 240, 22]
        self.drone_pos = None

        self.geo = Geolocation()
        self.geo.objectid = ""
        self.geo.lat = 0
        self.geo.long = 0
        self.latlongs = []
        # Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [pitch, roll, throttle]
        # #		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
        # self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
        # #!/usr/bin/env python3								You can change the upper limit and lower limit accordingly.
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.025  # in seconds

        # Publishing /drone_command, /alt_error, /pitch_error, /roll_error
        self.command_pub = rospy.Publisher(
            '/drone_command', edrone_msgs, queue_size=1)
        # ------------------------Add other ROS Publishers here-----------------------------------------------------

        self.throttle_error_pub = rospy.Publisher(
            '/alt_error', Float64, queue_size=1)
        self.pitch_error_pub = rospy.Publisher(
            '/pitch_error', Float64, queue_size=1)
        self.roll_error_pub = rospy.Publisher(
            '/roll_error', Float64, queue_size=1)
        self.ki_error_pub = rospy.Publisher(
            '/ki_alti_error', Float64, queue_size=1)
        self.Geolocation_pub = rospy.Publisher(
            '/geolocation', Geolocation, queue_size=1)
        # self.diff_error_pub=rospy.Publisher('/diff',Float64,queue_size=1)

        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll ,/edrone/camera_rgb/image_raw
        rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
        rospy.Subscriber('/pid_tuning_altitude',
                         PidTune, self.altitude_set_pid)
        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        # rospy.Subscriber('pid_tuning_pitch',PidTune,self.pitch_set_pid)
        rospy.Subscriber('/edrone/camera_rgb/image_raw',
                         Image, self.get_cur_frame)

        # ------------------------------------------------------------------------------------------------------------

        self.arm()  # ARMING THE DRONE

    # Disarming condition of the drone
    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    def get_cur_frame(self, dt):
        self.br = CvBridge()
        self.current_frame = self.br.imgmsg_to_cv2(
            dt, "bgr8")

    def show(self):
        cv.imshow("do", self.current_frame)
        cv.waitKey(1)

    # Arming condition of the drone : Best practise is to disarm and then arm the drone.
    def arm(self):

        self.disarm()
        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX4 = 1500
        self.command_pub.publish(self.cmd)  # Publishing /drone_command
        rospy.sleep(1)

    # Whycon callback function
    # The function gets executed each time when /whycon node publishes /whycon/poses
    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        # --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z
        # ---------------------------------------------------------------------------------------------------------------

    # Callback function for /pid_tuning_altitude
    # This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
    def altitude_set_pid(self, alt):
        # This is just for an example. You can change the ratio/fraction value accordingly
        self.Kp[2] = alt.Kp * 0.06
        self.Ki[2] = alt.Ki * 0.008
        self.Kd[2] = alt.Kd * 0.3

    # ----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

    def roll_set_pid(self, rol):
        self.Kp[1] = rol.Kp*0.06
        self.Ki[1] = rol.Ki*0.008
        self.Kd[1] = rol.Kd*0.3

    def pitch_set_pid(self, pit):
        self.Kp[0] = pit.Kp*0.06
        self.Ki[0] = pit.Ki*0.008
        self.Kd[0] = pit.Kd*0.3

    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):
        # -----------------------------Write the PID algorithm here--------------------------------------------------------------

        # Steps:
        # 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
        # 2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
        # 3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
        # 4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
        # 5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
        # 6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
        #																														self.cmd.rcPitch = self.max_values[1]
        # 7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
        # 8. Add error_sum

        self.throttle_error = -self.setpoint[2]+self.drone_position[2]
        k = int(1500+self.throttle_error*self.Kp[2]+(self.throttle_error -
                self.throttle_prev_error)*self.Kd[2]+self.throttle_sum_error*self.Ki[2])
        self.cmd.rcThrottle = k
        p = self.throttle_error-self.throttle_prev_error
        if (self.cmd.rcThrottle > self.max_throttle):
            self.cmd.rcThrottle = self.max_throttle
        if (self.cmd.rcThrottle < self.min_throttle):
            self.cmd.rcThrottle = self.min_throttle
        self.throttle_prev_error = self.throttle_error
        self.throttle_sum_error += self.throttle_error
        if (self.throttle_sum_error > 40):
            self.throttle_sum_error = 40
        if (self.throttle_sum_error < -40):
            self.throttle_sum_error = -40

        self.roll_error = self.setpoint[0]-self.drone_position[0]
        k = 1500+int(self.roll_error*self.Kp[1]+(
            self.roll_error-self.roll_prev_error)*self.Kd[1]+self.roll_sum_error*self.Ki[1])
        self.cmd.rcRoll = k
        p = self.roll_error-self.roll_prev_error
        if (self.cmd.rcRoll > self.max_roll):
            self.cmd.rcRoll = self.max_roll
        if (self.cmd.rcRoll < self.min_roll):
            self.cmd.rcRoll = self.min_roll
        self.roll_prev_error = self.roll_error
        self.roll_sum_error += self.roll_error

        self.pitch_error = -self.setpoint[1]+self.drone_position[1]
        k = int(1500+self.pitch_error*self.Kp[0]+(
            self.pitch_error-self.pitch_prev_error)*self.Kd[0]+self.pitch_sum_error*self.Ki[0])
        self.cmd.rcPitch = k
        p = self.pitch_error-self.pitch_prev_error
        if (self.cmd.rcPitch > self.max_pitch):
            self.cmd.rcPitch = self.max_pitch
        if (self.cmd.rcPitch < self.min_pitch):
            self.cmd.rcPitch = self.min_pitch
        self.pitch_prev_error = self.pitch_error
        self.pitch_sum_error += self.pitch_error

    # ------------------------------------------------------------------------------------------------------------------------
        self.command_pub.publish(self.cmd)
        self.throttle_error_pub.publish(self.throttle_error)
        self.roll_error_pub.publish(self.roll_error)
        self.pitch_error_pub.publish(self.pitch_error)
        # self.ki_error_pub.publish(self.throttle_sum_error)


# coordinates of the extremes of area to be traversed
lst1 = [-10, -10, 22]
lst2 = [10, -10, 22]
lst3 = [10, 6, 22]
lst4 = [-10, 6, 22]

# list to store points that control the drone movements
pts = []

# thi function make the list of points that the drone need reach


def make_points():
    p = 0
    stp = 7
    for i in range(lst1[1], lst3[1]+2, 2):
        step = 7
        start = lst1[0]
        if p % 2 == 1:
            step = -stp
            start = -lst1[0]
        for j in range(start, -start, step):
            pts.append([j, i, lst1[2]])
        pts.append([-start, i, lst1[2]])
        p += 1


# img processing++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


# finding squares in a frame-------------------------------------------------------------------------------
def ARR2npb(array):
    return np.uint8(array)


colours = {

    "Yellow": ARR2npb([[5, 80, 170], [30, 255, 255]])

}
epsilon = 0.1
kernel = np.ones((3, 4), np.uint0)


def getFrames(img):
    cvtImage = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    badFrames = []
    frames = []

    for bounds in colours:
        badFrames.append(cv.inRange(
            cvtImage, colours[bounds][0], colours[bounds][1]))
    for badFrame in badFrames:
        frames.append(cv.morphologyEx(cv.morphologyEx(
            badFrame, cv.MORPH_OPEN, kernel), cv.MORPH_CLOSE, kernel))
    return frames


def getCentres(frames):
    centre = (0, 0)
    list = []
    for i in range(len(frames)):
        contours, _ = cv.findContours(
            frames[i], cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
        for j in range(len(contours)):
            if cv.contourArea(contours[j]) < 8000:
                continue
            edges = cv.approxPolyDP(
                contours[j], epsilon*cv.arcLength(contours[j], True), True)
            temp = edges.ravel()
            xSum = 0
            ySum = 0
            for k in range(0, len(temp), 2):
                xSum += temp[k]
                ySum += temp[k+1]
            centre = (int(xSum/(len(temp)/2)), int(ySum/(len(temp)/2)))
            list.append(centre)
    return list

# finding squares part complete----------------------------------------------------------------------------------


pathoftxt = r"/home/mohit/Downloads/latlongs.txt"
file1 = open(pathoftxt, "w")
file1.close()

# georeferencing part starts--------------------------------------------------------------------------------------
# sift
sift = cv.SIFT_create()
# feature matching
bf = cv.BFMatcher(cv.NORM_L2, crossCheck=True)
# enter the path of the tif image
pth1 = r'/home/mohit/Downloads/task2d.tif'

ds = gdal.Open(pth1)
xoff0, a0, b0, yoff0, d0, e0 = ds.GetGeoTransform()


def pixel2coord(x, y, xoff, a, b, yoff, d, e):
    """Returns global coordinates from pixel x, y coords"""
    xp = a * x + b * y + xoff
    yp = d * x + e * y + yoff
    return (xp, yp)


# in order to reduce time we decrease our referencing space
px1 = pixel2coord(0, 0, xoff0, a0, b0, yoff0, d0, e0)

flag = 0


def georeference(img2, upper_left_x, upper_left_y, lower_right_x, lower_right_y, centre_of_sq):
    if (upper_left_x < 0):
        upper_left_x = 0
    if (upper_left_y < 0):
        upper_left_y = 0
    if (lower_right_x > 4000):
        lower_right_x = 4000
    if (lower_right_y > 4000):
        lower_right_y = 4000
    upper_left_x, upper_left_y = pixel2coord(
        upper_left_x, upper_left_y, xoff0, a0, b0, yoff0, d0, e0)
    lower_right_x, lower_right_y = pixel2coord(
        lower_right_x, lower_right_y, xoff0, a0, b0, yoff0, d0, e0)
    # enter the path of ouput cropped tif image
    opt = r'/home/mohit/Downloads/output1.tif'
    window = (upper_left_x, upper_left_y, lower_right_x, lower_right_y)
    gdal.Translate(opt, pth1, projWin=window)
    ds = gdal.Open(opt)
    xoff, a, b, yoff, d, e = ds.GetGeoTransform()
    px2 = pixel2coord(0, 0, xoff, a, b, yoff, d, e)
    px3 = (px1[0]-px2[0], px1[1]-px2[1])
    img1 = cv.imread(opt)
    img1 = cv.cvtColor(img1, cv.COLOR_BGR2GRAY)
    # cv.imshow("cropped_tif", img1)
    # enter the path where img2 will be saved temporarily
    pth2 = r'/home/mohit/Downloads/temp.png'
    cv.imwrite(pth2, img2)
    kp1, desc1 = sift.detectAndCompute(img1, None)
    img2 = cv.cvtColor(img2, cv.COLOR_BGR2GRAY)
    kp2, desc2 = sift.detectAndCompute(img2, None)
    matches = bf.match(desc1, desc2)
    matches = sorted(matches, key=lambda x: x.distance)
    # enter the path where georeferenced image will be saved
    pth3 = r'/home/mohit/Downloads/task2d_done.tif'
    # creation of command to be executed for georeferencing
    command = "gdal_translate "
    for i in range(10):
        p1 = kp1[matches[i].queryIdx].pt
        p2 = kp2[matches[i].trainIdx].pt
        p1 = pixel2coord(p1[0], p1[1], xoff0, a0, b0, yoff0, d0, e0)
        command += "-gcp "
        command += str(p2[0]) + " " + str(p2[1])+" " + \
            str(p1[0])+" " + str(p1[1])+" "
    command += " -of GTiff " + pth2+" "+pth3
    os.system(command)
    im = cv.imread(pth3)
    cv.imshow("georeferenced", im)
    # now we have our georeferenced image in path3
    # so we can now extract the coordinates of centroid

    from_SRS = "EPSG:4326"
    to_SRS = "EPSG:4326"
    src = pth3
    dest = r'/home/mohit/Downloads/updated.tif'
    cmd_list = ["gdalwarp", "-r", "bilinear", "-s_srs",
                from_SRS, "-t_srs", to_SRS, "-overwrite", src, dest]
    subprocess.run(cmd_list)
    ds = gdal.Open(dest)
    xoff, a, b, yoff, d, e = ds.GetGeoTransform()
    print(centre_of_sq)
    centre_coordinates = pixel2coord(
        centre_of_sq[0], centre_of_sq[1], xoff, a, b, yoff, d, e)
    centre_coordinates = (
        centre_coordinates[0]-px3[0], centre_coordinates[1]-px3[1])
    print(centre_coordinates)
    content = str(centre_coordinates[0])+" "+str(centre_coordinates[1])+" "
    fl = 1
    flag = 1
    for i in e_drone.latlongs:
        if abs(i[0]-centre_coordinates[0]) < 0.00005 and abs(i[1]-centre_coordinates[1]) < 0.00005:
            fl = 0
    if fl > 0:
        e_drone.latlongs.append(centre_coordinates)
        e_drone.geo.objectid = "obj"+str(len(e_drone.latlongs))
        e_drone.geo.long = centre_coordinates[0]
        e_drone.geo.lat = centre_coordinates[1]
        e_drone.Geolocation_pub.publish(e_drone.geo)
        print(e_drone.geo)
        file = open(pathoftxt, "a")
        file.write(content)
        file.close()
        content = content.split(' ')
        print(float(content[0]))
    return centre_coordinates


# georeferencing part done------------------------------------------------------------------------------------


# calling function to determine the points to be traversed
make_points()


k = 0
e_drone = Edrone()


# specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
r = rospy.Rate(40)


# print(len(pts))
# print(pts)

# to set initial point
e_drone.setpoint = pts[k]

last = -1
count = 0
lapse = 0
while not rospy.is_shutdown():
    e_drone.pid()
    count += 1
    r.sleep()
    if (abs(e_drone.throttle_error) < 0.5 and abs(e_drone.roll_error) < 0.5 and abs(e_drone.pitch_error) < 0.5):
        k += 1
        if k == len(pts):
            k = k-1
            pts[k][2] = 25
        e_drone.setpoint = pts[k]
    pos = e_drone.drone_position
    fr = e_drone.current_frame
    frms = getFrames(fr)
    lst = getCentres(frms)
    if (lapse != 0):
        lapse -= 1
    if (len(lst) > 0 and k != last and lapse == 0):
        point = (pos[0]*168+2000,
                 pos[1]*168+2000)
        point = georeference(
            fr, point[0]-550, point[1]-550, point[0]+550, point[1]+550, lst[0])
        last = k
        lapse = 4
    e_drone.show()
    if (flag and count > 500):
        e_drone.Geolocation_pub.publish(e_drone.geo)
        count = 0
cv.destroyAllWindows()
print(e_drone.latlongs)
