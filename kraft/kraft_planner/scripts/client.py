#! /usr/bin/env python
import rospy, time
# Sockets
import socket, struct

import StringIO

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from omni_msgs.msg import OmniButtonEvent
from std_msgs.msg import Header
from StringIO import StringIO

UDP_IP = "192.168.0.4"
UDP_PORT = 5052
SApos = 0.0
SEpos = -1.0
ELpos = 0.5
WPpos = 1.0
WYpos = 0.0
WRpos = 0.0
vel = 0.1
button = True
force_lock = True
xpos = 10.0
ypos = 20.0
zpos = 40.0

class LabviewServer:
	def __init__(self):	
		# Configura el listerner socket
		self.labview_port = UDP_PORT
		self.labview_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		# rospy.loginfo('UDP Socket listening on port [%d]' % (self.labview_port))
		
	def recv_timeout(self, timeout = 0.001):
		self.labview_sock.setblocking(0)
		total_data=[]
		data=''
		begin=time.time()
		while 1:
			#if you got some data, then timeout break 
			if total_data and time.time()-begin>timeout:
				break
			#if you got no data at all, wait a little longer
			elif time.time()-begin>timeout*2:
				break
			try:
				data=self.labview_sock.recv(8192)
				if data:
					total_data.append(data)
					begin=time.time()
			except:
				pass
		return ''.join(total_data)

	def send_message(self):
		global xpos
		global ypos
		global zpos

		message = JointState()
		message.header = Header()
		message.header.stamp = rospy.Time.now()
		message.name = ['SA','SE','EL','WP','WY','WR']
		print "SA ",SApos
		print "SE ",SEpos
		print "EL ",ELpos
		print "WP ",WPpos
		print "WY ",WYpos
		print "WR ",WRpos
		print " "
		message.position = [SApos, SEpos, ELpos, WPpos, WYpos, WRpos]
		message.velocity = [vel, vel, vel, vel, vel, vel]
		message.effort = []
		serialmessage = StringIO()
		message.serialize(serialmessage)
		self.labview_sock.sendto(serialmessage.getvalue(),(UDP_IP, UDP_PORT))


# Controla la posicion del Kraft con el Phantom
def callback(data):
	global SApos
	global SEpos
	global ELpos
	global WPpos
	global WYpos
	global WRpos
	global force_lock
	if not button:
		SApos = (data.position[0]+1.02)*3/1.98 - 1.5
		SEpos = (data.position[1]+0.3)*2/1.8 - 1.5
		ELpos = (data.position[2]+0.2)*1.8/2.1 - 0.6
		WYpos = (data.position[3]-3.7)*1.7/4.8 - 0.6
		WPpos = (data.position[4]+0.5)*1.7/2.1 - 0.6
		WRpos = (data.position[5]+2.55)*3/5.1 - 1.5
		# No bloquea el Phantom
		force_lock = False
	else:
		# No actualiza posicion del Kraft (no lee del Phantom)
		# Bloquea el Phantom
		force_lock = True
	print "force_lock ", force_lock
	server.send_message()

# Detecta la pulsacion de alguno de los botones
def buttoncallback(data):
	global button
	print data
	if(data.grey_button == 1) or (data.white_button == 1):
		button = not button
	print "button ", button

# Bloquea la posicion del Phantom cuando se pulse boton
def posecallback(data):
	global xpos
	global ypos
	global zpos
	if not button:
		# No bloquea el Phantom
		xpos = data.pose.position.x
		ypos = data.pose.position.y
		zpos = data.pose.position.z

def callbackIK(data):
	global SApos
	global SEpos
	global ELpos
	global WPpos
	global WYpos
	global WRpos
	SApos = data.position[0]
	SEpos = data.position[1]
	ELpos = data.position[2]
	WPpos = data.position[3]
	WYpos = data.position[4]
	WRpos = data.position[5]
	server.send_message()	
		
if __name__ == '__main__':
	global force_lock
	rospy.init_node('labview_client')
	server = LabviewServer()

	# Se lleva el Kraft a una posicion inicial
	# Hasta que no se pulse el boton no se comienza la teleoperacion
	print "waiting for press button"
	rospy.Subscriber("/phantom/button", OmniButtonEvent, buttoncallback)

	while button:
		server.send_message()
		
	rospy.Subscriber("/phantom/joint_states", JointState, callback)
	rospy.Subscriber("/phantom/pose", PoseStamped, posecallback)
	#rospy.Subscriber("/ikjoints", JointState, callbackIK)

	while not rospy.is_shutdown():
		rospy.spin()