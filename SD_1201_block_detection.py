#!/usr/bin/env python3

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import rospy
import time


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

    def pidset(self, dt):
        # print(dt)
        self.drone_pos = dt
        ft = 1
        self.throttle_error = -self.drone_pos[2]+self.drone_position[2]
        k = int(1500+self.throttle_error*self.Kp[2]/ft+(self.throttle_error -
                self.throttle_prev_error)*self.Kd[2]/ft+self.throttle_sum_error*self.Ki[2]/ft)
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

        ft = 150
        self.roll_error = -self.set_pt[0]+self.drone_pos[0]
        k = 1500+int(self.roll_error*self.Kp[1]/ft+(
            self.roll_error-self.roll_prev_error)*self.Kd[1]/ft+self.roll_sum_error*self.Ki[1]/ft)
        self.cmd.rcRoll = k
        p = self.roll_error-self.roll_prev_error
        if (self.cmd.rcRoll > self.max_roll):
            self.cmd.rcRoll = self.max_roll
        if (self.cmd.rcRoll < self.min_roll):
            self.cmd.rcRoll = self.min_roll
        self.roll_prev_error = self.roll_error
        self.roll_sum_error += self.roll_error
        # self.cmd.rcRoll = 1500

        ft = 1000
        self.pitch_error = self.set_pt[1]-self.drone_pos[1]
        k = int(1500+self.pitch_error*self.Kp[0]/ft+(
            self.pitch_error-self.pitch_prev_error)*self.Kd[0]/ft+self.pitch_sum_error*self.Ki[0]/ft)
        self.cmd.rcPitch = k
        p = self.pitch_error-self.pitch_prev_error
        if (self.cmd.rcPitch > self.max_pitch):
            self.cmd.rcPitch = self.max_pitch
        if (self.cmd.rcPitch < self.min_pitch):
            self.cmd.rcPitch = self.min_pitch
        self.pitch_prev_error = self.pitch_error
        self.pitch_sum_error += self.pitch_error
        # self.cmd.rcPitch = 1500

    # ------------------------------------------------------------------------------------------------------------------------
        self.command_pub.publish(self.cmd)
        self.throttle_error_pub.publish(self.throttle_error)
        self.roll_error_pub.publish(self.roll_error)
        self.pitch_error_pub.publish(self.pitch_error)


# pts = [[-8, 6, 23], [-8.5, 7, 23], [-6, 7.5, 22], [-8, 7, 22],
#        [-8, -7, 20], [-8, 6, 23], [-8, 5, 20], [-8, 5, 23]]

# pts = [[-8, 8, 22], [8, -8, 22], [8, 8, 22], [-8, 8, 22],
#        [-8, -7, 24], [-8, 6, 23], [-8, 5, 24], [-8, 5, 23]]


# coordinates of the extremes of area to be traversed
lst1 = [-10, -10, 22]
lst2 = [10, -10, 22]
lst3 = [10, 7, 22]
lst4 = [-10, 7, 22]

pts = []
# pts.append([-7, 6, 20])


def make_points():
    p = 0
    stp = 6
    for i in range(lst1[1], lst3[1]+2, 3):
        step = stp
        start = lst1[0]
        if p % 2 == 1:
            step = -stp
            start = -lst1[0]
        for j in range(start, -start, step):
            pts.append([j, i, lst1[2]])
        pts.append([-start, i, lst1[2]])
        p += 1


make_points()
k = 0
ijk = 0
e_drone = Edrone()
# specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
r = rospy.Rate(40)

# img processing


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
            if cv.contourArea(contours[j]) < 1000:
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


cnt = 0
flag = 0
print(len(pts))
print(pts)

while not rospy.is_shutdown():
    if flag == 0:
        e_drone.setpoint = pts[k]
    e_drone.pid()
    ijk += 1
    # print(e_drone.current_frame.shape)
    e_drone.show()
    if (cnt == 1 and flag == 0):
        frms = getFrames(e_drone.current_frame)
        lst = getCentres(frms)
        cnt = 0
        if len(lst) > 0:
            flag = 1
            print(lst)
            print(e_drone.setpoint)
            e_drone.setpoint = e_drone.drone_position
    cnt += 1
    # if flag == 1:
    #     print(e_drone.setpoint)
    r.sleep()
    if (abs(e_drone.throttle_error) < 0.2 and abs(e_drone.roll_error) < 0.2 and abs(e_drone.pitch_error) < 0.2):
        k += 1
    if k == len(pts):
        k = len(pts)-1
    if flag == 1:
        break
# cv.destroyAllWindows()

while (1):
    frms = getFrames(e_drone.current_frame)
    lst = getCentres(frms)
    # print(lst)
    e_drone.show()
    if len(lst) > 0:
        pst = [lst[0][0], lst[0][1], 22]
        e_drone.pidset(pst)
    r.sleep()

cv.destroyAllWindows()
