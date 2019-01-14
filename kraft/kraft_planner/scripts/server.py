#! /usr/bin/env python
import rospy, time, numpy
# Sockets
import socket, struct
# Messages
from sensor_msgs.msg import JointState
#from omni_msgs.msg import OmniFeedback

class LabviewServer:
	def __init__(self):	
		# Set up listerner socket
		self.labview_port = 5051
		self.labview_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.labview_sock.bind(('', self.labview_port))
		rospy.loginfo('UDP Socket listening on port [%d]' % (self.labview_port))
		
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
		
		
if __name__ == '__main__':
	rospy.init_node('labview_server')
	server = LabviewServer()
	while not rospy.is_shutdown():
		msg_ros = JointState()
		#force_msg = OmniFeedback()
		data = server.recv_timeout()
		if data:
			msg_ros.deserialize(data)
			print msg_ros
			
			# pub = rospy.Publisher("/phantom/force_feedback", OmniFeedback, queue_size=10)
			# force_msg.force.x = int(msg_ros.effort[0]*1)
			# force_msg.force.y = int(msg_ros.effort[1]*1)
			# force_msg.force.z = int(msg_ros.effort[2]*1)
			# pub.publish(force_msg)

			#print msg_ros.effort[0]

			#for i in range(1):
			#	print msg_ros.name[i], msg_ros.position[i]
