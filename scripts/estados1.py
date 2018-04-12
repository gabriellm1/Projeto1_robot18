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

import cormodule
import le_scan_teste


bridge = CvBridge()

cv_image = None

# Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
media = []
centro = []
area = 0.0
distances = []


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
		media, centro, area = cormodule.identifica_cor(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)






## Classes - estados


class Girando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhou', 'girando'])

    def execute(self, userdata):
		global velocidade_saida

		if media is None or len(media)==0:
			return 'girando' #Continua girando

		if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
			velocidade_saida.publish(vel)
			return 'girando' #Continua girando
		if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed))
			velocidade_saida.publish(vel)
			return 'girando' #Continua girando
		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'alinhou' #Alinhou! Agora vai seguir(Centralizado)


class Centralizado(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['brecar','alinhando', 'alinhado'])

    def execute(self, userdata):
		global velocidade_saida

		if media is None:
			return 'alinhou'
		if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
			return 'alinhando'  # alinhaNdo volta pro girando(busca)
		if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
			return 'alinhando'
		else:
			if min(distances) > 0.2: #Falta ver a métrica da distancia e estipular uma distancia minima
				vel = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				return 'alinhado' # Continua seguindo reto
			else:
				vel = Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				return 'brecar'

class Parar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['brecar','girando'])
    def execute(self, userdata):
		global velocidade_saida

		if min(distances) < 0.2:
			vel = Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'brecar'
		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'girando'


# main
def main():
	global velocidade_saida
	global buffer
	rospy.init_node('cor_estados')

	# Para usar a webcam
	#recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

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
	    smach.StateMachine.add('GIRANDO', Girando(),
	                            transitions={'girando': 'GIRANDO',
	                            'alinhou':'CENTRO'})
	    smach.StateMachine.add('CENTRO', Centralizado(),
	                            transitions={'alinhando': 'GIRANDO',
	                            'alinhado':'CENTRO',
								'brecar':'PARAR'})
		smach.StateMachine.add('PARAR', Parar(),
	                            transitions={'girando': 'GIRANDO',
								'brecar':'PARAR'})


	# Execute SMACH plan
	outcome = sm.execute()
	#rospy.spin()


if __name__ == '__main__':
    main()