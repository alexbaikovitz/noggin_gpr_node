import serial
import rospy
import numpy as np
import sys
from noggin_gpr_node.msg import StampedWaveform


import matplotlib.pyplot as plt

def main(args):
  rospy.init_node('noggin_gpr_node')

  pub = rospy.Publisher('/gpr/stamped_trace', StampedWaveform, queue_size=10)

  serial_ = serial.Serial(port="/dev/ttyUSB0", baudrate=115200,timeout=0)
  r = rospy.Rate(20) # 10hz

  points = 201
  expected_size = points*2 + 6

  while not rospy.is_shutdown():

    serial_.write(b'T\r')
    
    output = serial_.read(expected_size)
    print(len(list(output)))

    if len(output) == expected_size:
      out = list()
      for i in range(0,points):
        out.append(int.from_bytes(output[2*i+3:2*i+5],byteorder='big', signed=True))

      sw = StampedWaveform()
      sw.trace = out
      pub.publish(sw)
    else:
      serial_.reset_output_buffer()

    r.sleep()
  

if __name__=="__main__":
  main(sys.argv[3:])