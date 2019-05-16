#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy


class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.sub=rospy.Subscriber('/duckiebot/camera_node/image/raw',Image,self.procesar_img)
		self.pub=rospy.Publisher('/duckiebot/camera_node/image/uvwevwe',Image,queue_size=0)
		self.pub_mask=rospy.Publisher('/duckiebot/camera_node/image/mask',Image,queue_size=0)
		self.pub_eq=rospy.Publisher('/duckiebot/camera_node/image/equalizada',Image,queue_size=0)


	#def publicar(self):

	#def callback(self,msg):

	def procesar_img(self, img):
		bridge=CvBridge()
		image=bridge.imgmsg_to_cv2(img,"bgr8")

		#img_equalizada=cv2.equalizeHist(image)

		# Cambiar espacio de color
		color_space=cv2.COLOR_BGR2HSV
		img_out = cv2.cvtColor(image,color_space)
		# Filtrar rango util
		cota1=np.array([22,145,98])
		cota2=np.array([104,255,255])
		# Aplicar mascara
		mask=cv2.inRange(image,cota1,cota2)
		img_out = cv2.bitwise_and(img_out,img_out,mask=mask)
		# Aplicar transformaciones morfologicas
		kernel=np.ones((2,2),np.uint8)
		img_out=cv2.erode(img_out,kernel,iterations=1)
		img_out=cv2.dilate(img_out,kernel,iterations=2)
		# Definir blobs
		_, contours, hierarchy=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		# Dibujar rectangulos de cada blob
		for i in contours:
			x,y,w,h=cv2.boundingRect(i)
			if w*h>250:
				cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,255),2)
		# Publicar imagen final
		imgO=bridge.cv2_to_imgmsg(image,"bgr8")
		#imgO=bridge.cv2_to_imgmsg(img_out,"bgr8")
		self.pub.publish(imgO)
		
		# Publicar imagen mascara
		msg_img_mask=bridge.cv2_to_imgmsg(img_out,"bgr8")
		self.pub_mask.publish(msg_img_mask)

		#Publicar Imagen equalizada
		#msg_img_equalizada=bridge.cv2_to_imgmsg(img_equalizada,"bgr8")
		#self.pub_eq.publish(msg_img_equalizada)


def main():
	rospy.init_node('ojitodepiscina') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
