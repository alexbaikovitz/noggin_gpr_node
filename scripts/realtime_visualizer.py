import rospy
import sys
import numpy as np
import matplotlib.pyplot as plt
import time
import cv2

from noggin_gpr_node.msg import StampedWaveform
# from nav_msgs import Odometry
from gpr_config import GprConfig
from plot_trace_image import RosbagVisualizer
from trace_to_image import TraceToImage

class RealtimeVisualizer:

  def __init__(self, config):
    self.config = config
    self.processor = RosbagVisualizer(config)
    self.num_points = self.config.num_points - abs(self.config.point_offset)
    self.radar_array = np.zeros((self.num_points, 300))
    self.viz_length = 300
    self.counter = 0
    self.trace_to_image_converter = TraceToImage()

  # def TraceCallback2(self, msg):



  def TraceCallback(self, msg):

    print(f"Time since acquisition: {rospy.Time.now() - msg.header.stamp}")

    self.counter += 1
    processed_trace, _ = self.processor.ProcessAScan(np.array(msg.trace))
    processed_trace = self.trace_to_image_converter.Update(processed_trace)

    self.radar_array[:,0:-1] = self.radar_array[:,1:]
    self.radar_array[:,-1] = processed_trace

    if self.counter % 3 == 0:
      cv2.namedWindow("rad", cv2.WINDOW_NORMAL)
      cv2.resizeWindow("rad", 1000, 1000)
      cv2.imshow("rad", self.radar_array.astype(np.int16))
      if cv2.waitKey(1) & 0xFF == ord('q'):
        return

  def Execute(self):

    rospy.init_node('gpr_visualizer', anonymous=True)

    rospy.Subscriber(self.config.trace_topic, StampedWaveform, self.TraceCallback)
    # rospy.Subscriber("/wheel_encoder/odom", Odometry, self.OdometryCallback)

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":

  config_path = "../config/noggin_gpr_config.yaml"

  gpr_config = GprConfig(config_path)

  viz = RealtimeVisualizer(gpr_config)
  viz.Execute()
  


