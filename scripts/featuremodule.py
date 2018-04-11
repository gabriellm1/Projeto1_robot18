import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

def identifica_feature(frame):

    global centro
    global media
    # Capture frame-by-frame
    print("New frame")

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    grayKP,grayDesc=detector.detectAndCompute(gray,None)

    # A gaussian blur to get rid of the noise in the image
    blur = cv2.GaussianBlur(gray,(5,5),0)
    # Detect the edges present in the image
    bordas = auto_canny(blur)


    matches=flann.knnMatch(grayDesc,madDesc,k=2)

    goodMatch=[]
    for m,n in matches:
        if(m.distance < 0.75*n.distance):
            goodMatch.append(m)
    if(len(goodMatch)>MIN_MATCH_COUNT):
        tp=[]
        qp=[]
        for m in goodMatch:
            tp.append(madKP[m.trainIdx].pt)
            qp.append(grayKP[m.queryIdx].pt)
        tp,qp=np.float32((tp,qp))
        H,status=cv2.findHomography(tp,qp,cv2.RANSAC,3.0)
        h,w=img.shape
        trainBorder=np.float32([[[0,0],[0,h-1],[w-1,h-1],[w-1,0]]])
        queryBorder=cv2.perspectiveTransform(trainBorder,H)
        x0 = queryBorder[0][0][0]
        x1 = (queryBorder[0][1][0])
        x2 = (queryBorder[0][2][0])
        x3 = (queryBorder[0][3][0])
        y0 = (queryBorder[0][0][1])
        y1 = (queryBorder[0][1][1])
        y2 = (queryBorder[0][2][1])
        y3 = (queryBorder[0][3][1])
        media_x = (x0+x1+x2+x3)/4.0
        media_y = (y0+y1+y2+y3)/4.0

        media = (media_x, media_y)  

        centro = (frame.shape[1]//2, frame.shape[0]//2) 
        
        dif_x = media[0]-centro[0]
        dif_y = media[1]-centro[1]

        
        tx = x3 - x0
        ty = y1 - y0
        tamanho = (tx*ty)

        cv2.polylines(bordas,[np.int32(queryBorder)],True,(0,255,0),5)
    else:
        media = (0,0)
        print "Not Enough match found- %d/%d"%(len(goodMatch),MIN_MATCH_COUNT)

    return media, centro