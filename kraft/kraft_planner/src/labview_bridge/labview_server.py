#! /usr/bin/env python
import rospy, time, math
import socket

class TextColors:
  HEADER = '\033[95m'
  OKBLUE = '\033[94m'
  OKGREEN = '\033[92m'
  WARNING = '\033[93m'
  FAIL = '\033[91m'
  ENDC = '\033[0m'

  def disable(self):
    self.HEADER = ''
    self.OKBLUE = ''
    self.OKGREEN = ''
    self.WARNING = ''
    self.FAIL = ''
    self.ENDC = ''

class LabviewServer:
  def __init__(self): 
    self.ns = rospy.get_namespace()
    # Read all the parameters from the parameter server
    self.publish_frequency = self.read_parameter('~publish_frequency', 1000.0)
    # UDP
    self.read_port = int(self.read_parameter('~read_port', 5051))
    self.write_ip = self.read_parameter('~write_ip', '192.168.0.4')
    self.write_port = int(self.read_parameter('~write_port', 5052))
    # Set up read socket
    self.read_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.read_socket.bind(('', self.read_port))
    rospy.loginfo('UDP Socket listening on port [%d]' % (self.read_port))
    # Set up write socket
    self.write_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rospy.loginfo('UDP Socket sending to [udp://%s:%d]' % (self.write_ip, self.write_port))
    # Register rospy shutdown hook
    rospy.on_shutdown(self.shutdown_hook)
    
  def shutdown_hook(self):
    # Do some cleaning depending on the app
    pass

  def recv_timeout(self, timeout=0.001):
    self.read_socket.setblocking(0)
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
        data=self.read_socket.recv(8192)
        if data:
          total_data.append(data)
          begin=time.time()
      except:
        pass
    return ''.join(total_data)
  
  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)

  def loginfo(self, msg):
    rospy.logwarn(self.colors.OKBLUE + str(msg) + self.colors.ENDC)


if __name__ == '__main__':
  rospy.init_node('labview_server')
  server = LabviewServer()
  server.execute()
