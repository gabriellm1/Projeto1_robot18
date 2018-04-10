#!/usr/bin/env python
# -*- coding:utf-8 -*-

_author_      = "Matheus Dib, Fabio de Miranda"


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


bridge = CvBridge()

cv_image = None

check_delay = True # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados



# If you want to open a video, just change this path
#cap = cv2.VideoCapture('hall_box_battery.mp4')

# Parameters to use when opening the webcam.

template = cv2.imread('case.png',0)
w, h = template.shape[::-1]

centro = []
media = []

atraso = 1.5E9

MIN_MATCH_COUNT=30

detector=cv2.xfeatures2d.SIFT_create()

FLANN_INDEX_KDITREE=0
flannParam=dict(algorithm=FLANN_INDEX_KDITREE,tree=5)
flann=cv2.FlannBasedMatcher(flannParam,{})

img=cv2.imread("case.png",0)
madKP,madDesc=detector.detectAndCompute(img,None)

cam=cv2.VideoCapture(0)



cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

lower = 0
upper = 1

# Returns an image containing the borders of the image
# sigma is how far from the median we are setting the thresholds
def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged


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
    #blur = gray
    # Detect the edges present in the image
    bordas = auto_canny(blur)

    #gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    #grayKP,grayDesc=detector.detectAndCompute(gray,None)
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

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()


def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime
    delay = lag.secs
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        media, centro = identifica_feature(cv_image)
        depois = time.clock()
        cv2.imshow("Camera", cv_image)
    except CvBridgeError as e:
        print('ex', e)

if __name__=="__main__":

    rospy.init_node("projeto")
    # Para usar a Raspberry Pi
    recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
    
    # Para usar a webcam 
    #recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    try:

        while not rospy.is_shutdown():
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            if len(media) != 0 and len(centro) != 0:
                dif_x = media[0]-centro[0]
                dif_y = media[1]-centro[1]
                if math.fabs(dif_x)<30: # Se a media estiver muito proxima do centro anda para frente
                    vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0))
                else:
                    if dif_x > 0: # Vira a direita
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
                    else: # Vira a esquerda
                        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
            velocidade_saida.publish(vel)
            rospy.sleep(0.01)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")