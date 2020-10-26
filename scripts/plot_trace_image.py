import rosbag
import sys
import numpy as np
import matplotlib.pyplot as plt
import time

import cv2

from noggin_gpr_node.msg import StampedWaveform
from trace_to_image import TraceToImage
from gpr_config import GprConfig
import signal_processing_utils
import scipy.ndimage

class RosbagVisualizer:

  SEARCH_OFFSET = 30

  def __init__(self, config):
    
    self.config = config
    self.bag_path = self.config.bag_path
    self.gpr_topic = self.config.trace_topic
    self.num_points = config.num_points
    self.desired_image_size = self.num_points - abs(config.point_offset)
    self.radar_array = np.empty((self.desired_image_size, 0))
    self.radar_array_unproc = np.empty((self.desired_image_size, 0))
    self.num_readings = 0
    self.contrast_scalar = 20
    self.max_amplitude = 12000
    self.max_amplitude2 = 20000
    self.flatten_index = 100
    self.zero_position = None

    self.img_converter = TraceToImage()


  def ProcessAScan(self, unprocessed_trace):

    if self.zero_position == None:
      self.zero_position = np.argmin(unprocessed_trace)
    
    time_corrected_trace = unprocessed_trace[self.zero_position:]

    # tr = signal_processing_utils.discrete_wavelet_transform(unprocessed_trace[zero_position:])
    
    filtered_trace = signal_processing_utils.butterworth_bandpass_filter(time_corrected_trace)
    # dewowed_trace = signal_processing_utils.binomial_dewow_filter(filtered_trace)
    filtered_trace, off1 = signal_processing_utils.correct_dc_offset(filtered_trace)
    
    # signal_processing_utils.plot_filter_result([time_corrected_trace, filtered_trace], ["Unprocessed", "Butterworth Bandpass Filter"])
  
    # signal_processing_utils.plot_filter_result(unprocessed_trace[zero_position:], [tr, tr2], ["Discrete Wavelet Transform", "Butterworth Bandpass Filter"])


    # filtered_trace = scipy.ndimage.gaussian_filter1d(unprocessed_trace, 2)

    # filtered_trace[np.where(filtered_trace > self.max_amplitude)] = self.max_amplitude
    # filtered_trace[np.where(filtered_trace < -1 * self.max_amplitude)] = -1 * self.max_amplitude

    # min_position = np.argmin(
    #   filtered_trace[set_offset : set_offset + RosbagVisualizer.SEARCH_OFFSET])
    # filtered_trace = self.contrast_scalar * filtered_trace

    # filtered_trace[np.where(filtered_trace > self.max_amplitude2)] = self.max_amplitude
    # filtered_trace[np.where(filtered_trace < -1 * self.max_amplitude2)] = -1 * self.max_amplitude

    # max_position = np.argmax(
    #   unprocessed_trace[set_offset : set_offset + RosbagVisualizer.SEARCH_OFFSET])

    # center_position = np.argmin(np.abs(
    #   unprocessed_trace[min_position : max_position]
    # ))

    out = np.zeros(self.desired_image_size)
    out2 = np.zeros(self.desired_image_size)
    out[:filtered_trace.size] = filtered_trace
    out2[:time_corrected_trace.size] = time_corrected_trace

    return out, out2

  def CreateNonMetricImage(self):
    for topic, msg, _ in rosbag.Bag(self.bag_path).read_messages():
      if topic == self.gpr_topic:
        self.num_readings += 1
        
        unproc_trace = np.array(msg.trace)
        trace, unproc = self.ProcessAScan(unproc_trace)
        self.radar_array = np.hstack((self.radar_array, np.transpose([trace])))
        self.radar_array_unproc = np.hstack((self.radar_array_unproc, np.transpose([unproc])))

        if self.num_readings > 200: break

    # self.radar_array = scipy.ndimage.filters.gaussian_filter1d(self.radar_array, .1, axis=1, order=0)
    
    # filtered = signal_processing_utils.discrete_wavelet_transform_image(self.radar_array)

    gained_image = signal_processing_utils.find_exponential_envelope(self.radar_array, True, self.radar_array_unproc)

    # filtered = self.contrast_scalar * filtered

    # filtered[np.where(filtered > 1)] = .8
    # filtered[np.where(filtered < -1)] = -.8

    # self.radar_array = self.contrast_scalar * self.radar_array

    # self.radar_array[np.where(self.radar_array > self.max_amplitude2)] = self.max_amplitude
    # self.radar_array[np.where(self.radar_array < -1 * self.max_amplitude2)] = -1 * self.max_amplitude

    # filtered = signal_processing_utils.discrete_wavelet_transform_image(gained_image)

    # blurred = cv2.bilateralFilter(np.float32(np.clip((filtered + 0.5),0,1)),5, .1, 1)

    # img = self.img_converter.traces_to_image2d(filtered, maximum_value=1)
    plt.imshow(gained_image, interpolation='none', cmap='seismic')
    plt.colorbar()
    plt.show()
    # plt.figure()
    # plt.imshow(self.radar_array, interpolation='none')
    # plt.show()

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