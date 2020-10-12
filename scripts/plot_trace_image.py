import rosbag
import sys
import numpy as np
import matplotlib.pyplot as plt
import time


from noggin_gpr_node.msg import StampedWaveform
from gpr_config import GprConfig

import scipy.ndimage

class RosbagVisualizer:

  SEARCH_OFFSET = 30

  def __init__(self, config):
    
    self.config = config
    self.bag_path = self.config.bag_path
    self.gpr_topic = self.config.trace_topic
    self.num_points = config.num_points - abs(self.config.point_offset)
    self.radar_array = np.empty((self.num_points, 0))
    self.num_readings = 0
    self.gain = self.GainFunction()
    self.contrast_scalar = 20
    self.max_amplitude = 12000
    self.max_amplitude2 = 20000
    self.flatten_index = 100


  def GainFunction(self):
    xspace = np.linspace(0, 10, self.num_points)
    gain = np.exp(.5 * xspace)
    gain[100:] = gain[100]

    return gain

  def ProcessAScan(self, unprocessed_trace):

    # Select first minimum position from the airwave as the zero position.
    # set_offset = abs(self.config.point_offset)

    # start_time = time.perf_counter()

    unprocessed_trace = unprocessed_trace[30:]

    filtered_trace = unprocessed_trace
    # filtered_trace = scipy.ndimage.gaussian_filter1d(unprocessed_trace, 2)

    # filtered_trace[np.where(filtered_trace > self.max_amplitude)] = self.max_amplitude
    # filtered_trace[np.where(filtered_trace < -1 * self.max_amplitude)] = -1 * self.max_amplitude

    # min_position = np.argmin(
    #   filtered_trace[set_offset : set_offset + RosbagVisualizer.SEARCH_OFFSET])
    filtered_trace = self.contrast_scalar * filtered_trace

    filtered_trace[np.where(filtered_trace > self.max_amplitude2)] = self.max_amplitude
    filtered_trace[np.where(filtered_trace < -1 * self.max_amplitude2)] = -1 * self.max_amplitude

    # max_position = np.argmax(
    #   unprocessed_trace[set_offset : set_offset + RosbagVisualizer.SEARCH_OFFSET])

    # center_position = np.argmin(np.abs(
    #   unprocessed_trace[min_position : max_position]
    # ))

    # zero_position = 30 # min_position + set_offset

    zero_position = 30

    end_idx = self.config.num_points - zero_position - 1

    out = np.zeros(self.config.num_points - abs(self.config.point_offset))

    out[:end_idx] = filtered_trace
    
    # out = self.contrast_scalar * out
    # out = np.multiply(self.gain, out)
    # out = out.reshape(-1,1)
    # print(f"Time to run processing = {time.perf_counter()-start_time}")
    return out

  def CreateNonMetricImage(self):
    for topic, msg, _ in rosbag.Bag(self.bag_path).read_messages():
      if topic == self.gpr_topic:
        self.num_readings += 1
        
        trace = self.ProcessAScan(np.array(msg.trace))
        self.radar_array = np.hstack((self.radar_array, trace))

    plt.imshow(self.radar_array, interpolation='none')
    plt.colorbar()
    plt.show()

def main(argv):
  config_path = "../config/noggin_gpr_config.yaml"
  
  if len(argv) > 0:
    bag_path = argv[0]
  else:
    print("Default bag file being visualized.")
    bag_path = "/home/abaik/bagfiles/moving2dronecage_2020-10-08-15-32-41.bag"

  gpr_config = GprConfig(config_path, bag_path)

  viz = RosbagVisualizer(gpr_config)
  viz.CreateNonMetricImage()


if __name__ == "__main__":
  main(sys.argv[1:])