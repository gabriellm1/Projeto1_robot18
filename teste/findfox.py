import cv2
import numpy as np
from matplotlib import pyplot as plt
import time

#Open the webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged

def image_detector(img,frame,frame_g):
    minLines=15
    frame_circle = blur = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),(5,5),0)
    bordas = auto_canny(frame_circle)
    circles = []
    bordas_color = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR)
    circles=cv2.HoughCircles(bordas,cv2.HOUGH_GRADIENT,2,40,param1=50,param2=100,minRadius=15,maxRadius=60)

    if circles is not None:
        circles = np.uint16(np.around(circles))

        for i in circles[0,:]:
            print(i)
            # draw the outer circle
            # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]])
            cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)

    sift = cv2.xfeatures2d.SIFT_create()
    keyPoint1, descrpt1 = sift.detectAndCompute(img,None)
    keyPoint2, descrpt2 = sift.detectAndCompute(frame_g,None)

    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = 0, trees = 5)
    search_params = dict(checks = 50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)

    lines = flann.knnMatch(descrpt1,descrpt2,k=2)

    good = []
    for m,n in lines:
        if m.distance < 0.5*n.distance:
            good.append(m)

    if len(good)>minLines:
        #Transformação para float 32
        src_points = np.float32([ keyPoint1[m.queryIdx].pt for m in good]).reshape(-1,1,2)
        dst_points = np.float32([ keyPoint2[m.trainIdx].pt for m in good]).reshape(-1,1,2)

        M, mask = cv2.findHomography(src_points, dst_points, cv2.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()
        h,w = img.shape[0], img.shape[1]
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

        dst = cv2.perspectiveTransform(pts,M)

        frame = cv2.polylines(frame,[np.int32(dst)],True,255,3,cv2.LINE_AA)
        print("Parece que tem uma raposa na imagem")

    else:
        print("Não tem nenhuma raposa na imagem")
        matchesMask = None

    draw_params = dict(matchColor = (0,255,0),singlePointColor = None,matchesMask = matchesMask,flags=2)

    return frame, bordas_color

while(True):
    ret, frame = cap.read()

    gray = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
    frame = image_detector(cv2.imread("madfox.jpg"),frame,gray)

    cv2.imshow('Raposa Detector',frame[0])
    #cv2.imshow('Detector de circulos',frame[1])

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.desttroyAllWindows()