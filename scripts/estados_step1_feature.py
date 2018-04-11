#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


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
import smach
import smach_ros

###################### features #####################
import featuremodule
# Parameters to use when opening the webcam.

template = cv2.imread('case.png',0)
w, h = template.shape[::-1]

centro = []
media = []

atraso = 1.5E8

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

#########################################################
bridge = CvBridge()

cv_image = None

# Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
media = []
centro = []
area = 0.0


direcao_procurar = 1
tolerancia_x = 50
tolerancia_y = 20
ang_speed = 0.4
area_ideal = 60000 # área da distancia ideal do contorno - note que varia com a resolução da câmera
tolerancia_area = 20000

# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
atraso = 1.5
check_delay = False # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados




def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro
	global area

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso and check_delay==True:
		return
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro = featuremodule.identifica_feature(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)






## Classes - estados


class Procurando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Objeto Encontrado', 'Procurando'])

    def execute(self, userdata):
		global velocidade_saida

		if media is None or len(media)==0:
    		#vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, direcao_procurar*ang_speed))
			#velocidade_saida.publish(vel)
			return 'Procurando'
		if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
			velocidade_saida.publish(vel)
			return 'Procurando'
		if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed))
			velocidade_saida.publish(vel)
			return 'Procurando'
		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'Objeto Encontrado'


class Seguir(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Centralizar', 'Objeto Centralizado'])

    def execute(self, userdata):
		global velocidade_saida


		if media is None:
			return 'Objeto Centralizado'
		if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
			return 'Centralizar'
		if math.fabs(media[0]) > math.fabs(centro[0] - 5*tolerancia_x):
			direcao_procurar = 1
			return 'Centralizar'
		if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
			return 'Centralizar'
		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			direcao_procurar = -1
			return 'alinhado'

# main
def main():
	global velocidade_saida
	global buffer
	rospy.init_node('cor_estados')

	# Para usar a webcam
	recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
	#recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['terminei'])

	# Open the container
	with sm:
	    # Add states to the container
	    #smach.StateMachine.add('LONGE', Longe(),
	    #                       transitions={'ainda_longe':'ANDANDO',
	    #                                    'perto':'terminei'})
	    #smach.StateMachine.add('ANDANDO', Andando(),
	    #                       transitions={'ainda_longe':'LONGE'})
	    smach.StateMachine.add('PROCURANDO', Procurando(),
	                            transitions={'Procurando': 'PROCURANDO',
	                            'Objeto Encontrado':'CENTRO'})
	    smach.StateMachine.add('CENTRO', Seguir(),
	                            transitions={'Centralizar': 'PROCURANDO',
	                            'Objeto Centralizado':'CENTRO'})


	# Execute SMACH plan
	outcome = sm.execute()
	#rospy.spin()


if __name__ == '__main__':
    main()
