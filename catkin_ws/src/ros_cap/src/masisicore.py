#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import Twist2DStamped

class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.pub=rospy.Publisher('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, queue_size=0)
		self.sub=rospy.Subscriber('/duckiebot/joy',Joy,self.callback)
		self.twist=Twist2DStamped()


	#def publicar(self):

	def callback(self,msg):
#------------------------------------------------------------------------------------------
		A=(msg.axes)[5]
		R=(msg.axes)[2]
		Ac=((A-1)*(A-1))*2.5
		Rv=((R-1)*(R-1))*2.5
		Vp=Ac-Rv
		self.twist.v=Vp
#------------------------------------------------------------------------------------------
		Zm=msg.axes[0]		
		Om=round(Zm,1)*-10
		#Dr=(msg.buttons)[12]
		#izq=(msg.buttons)[13]
		#if Dr==1:
		#	self.twist.omega=10
		#elif izq==1:
		#	self.twist.omega=-10
		#else:
		#	self.twist.omega=0
		self.twist.omega=Om
#------------------------------------------------------------------------------------------
		Em=(msg.buttons)[0]
		if Em==1:
			self.twist.v=0
			self.twist.omega=0
		self.pub.publish(self.twist)

def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
