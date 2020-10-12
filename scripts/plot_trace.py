#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
import numpy as np
from noggin_gpr_node.msg import StampedWaveform

from gpr_feedback.msg import Trace

class TraceMonitor(object):
    def __init__(self):
        rospy.init_node('gpr_plot_trace', anonymous=True)

        self.trace_sub = rospy.Subscriber("/gpr/stamped_trace", StampedWaveform, self.callback_plot)
        
        self.xpos = 0

        self.output_file = "gpr_feedback.txt"

        self.all_data = np.array([]) 
    
    def callback_plot(self, data):
        # temp_mat = np.zeros((num_points,3))

        # temp_mat[:,0] = self.xpos * np.ones(num_points)
        # temp_mat[:,1] = np.arange(0,200)

        trace = np.array(data.trace)
        temp_mat = trace.reshape((trace.size,1))
        self.xpos += 1

        if self.all_data.size == 0:
            self.all_data = temp_mat
        else:
            self.all_data = np.concatenate((self.all_data, temp_mat), axis=1)

        print(self.all_data)

        plt.plot(x_vals,trace)
        
    def write_mat(self):
        np.savetxt("/home/abaik/bagfiles/result.txt",self.all_data,delimiter='\t',fmt='%.18f')

if __name__ == '__main__':
    
    plt.figure()

    num_points = 200
    x_vals = np.arange(num_points)
    plt.title("GPR Traces")
    plt.xlabel("Time Units")
    plt.ylabel("Trace Amplitude")
    
    check_trace = TraceMonitor()
    plt.show()
    rospy.spin()
