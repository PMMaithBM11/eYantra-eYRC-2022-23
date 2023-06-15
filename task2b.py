import cv2 as cv, numpy as np
def ARR2npb(array):
    return np.uint8(array)

image = cv.imread("yellow_detect.jpeg")

colours = {

"Yellow" : ARR2npb([[5,80,170], [30,255,255]])

}

epsilon = 0.01
kernel = np.ones((3,4), np.uint0)

def getFrames(img = image):
    cvtImage = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    badFrames = []
    frames = []

    for bounds in colours: badFrames.append(cv.inRange(cvtImage, colours[bounds][0], colours[bounds][1]))
    for badFrame in badFrames:
        frames.append(cv.morphologyEx(cv.morphologyEx(badFrame,cv.MORPH_OPEN,kernel), cv.MORPH_CLOSE,kernel))
    return frames

def getCentre(frames = getFrames()):
    centre = (0,0)
    for i in range(len(frames)):
        xSum = 0
        ySum = 0
        contours,_ = cv.findContours(frames[i], cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
        for j in range(len(contours)):
            edges = cv.approxPolyDP(contours[j], epsilon*cv.arcLength(contours[j], True), True)
            temp = edges.ravel()
            for k in range(0, len(temp), 2):
                xSum += temp[k]
                ySum += temp[k+1]
        centre = (xSum/(len(temp)/2), ySum/(len(temp)/2))
    c=(int(centre[0]),int(centre[1]))
    return c

c=getCentre()
print(c[0],c[1])