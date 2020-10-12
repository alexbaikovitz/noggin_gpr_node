import rospy
import sys
import numpy as np
import matplotlib.pyplot as plt
import time
import cv2

from noggin_gpr_node.msg import StampedWaveform
from gpr_config import GprConfig
from plot_trace_image import RosbagVisualizer
from trace_to_image import TraceToImage

class RealtimeVisualizer:

  def __init__(self, config):
    self.config = config
    self.processor = RosbagVisualizer(config)
    self.num_points = self.config.num_points - abs(self.config.point_offset)
    self.radar_array = np.zeros((181, 300))
    self.viz_length = 300
    self.counter = 0
    self.trace_to_image_converter = TraceToImage()

  # def TraceCallback2(self, msg):



  def TraceCallback(self, msg):

    print(f"Time since acquisition: {rospy.Time.now() - msg.header.stamp}")

    self.counter += 1
    # if self.counter % 4 == 0:
    processed_trace = self.processor.ProcessAScan(np.array(msg.trace))
    processed_trace = self.trace_to_image_converter.Update(processed_trace)
    # print(processed_trace)
    # if self.counter//4 < self.viz_length:
    #   t1 = time.perf_counter()
    #   self.radar_array[:,self.counter//4] = processed_trace
    #   print(f"time to add array element = {time.perf_counter()-t1}")


    # else:
    t1 = time.perf_counter()
    self.radar_array[:,0:-1] = self.radar_array[:,1:]
    self.radar_array[:,-1] = processed_trace
    print(f"time to crate new array array = {time.perf_counter()-t1}")

    if self.counter % 3 == 0:
      cv2.namedWindow("rad", cv2.WINDOW_NORMAL)
      # cv2.resizeWindow("rad", 500, 500)
      cv2.imshow("rad", self.radar_array.astype(np.uint8))
      
      if cv2.waitKey(1) & 0xFF == ord('q'):
        return

          # plt.imshow(self.radar_array, interpolation='none')
          # print(rospy.Time.now() - msg.header.stamp)
          # plt.draw()
          # plt.pause(0.000000000000000000000000000000000001)

  def Execute(self):

    rospy.init_node('gpr_visualizer', anonymous=True)

    rospy.Subscriber(self.config.trace_topic, StampedWaveform, self.TraceCallback)

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":

  config_path = "../config/noggin_gpr_config.yaml"

  gpr_config = GprConfig(config_path)

  viz = RealtimeVisualizer(gpr_config)
  viz.Execute()
  


