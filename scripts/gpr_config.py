import yaml
import os
import rospy

class GprConfig():
  def __init__(self, path_to_config, bag_path=""):

    if os.path.isfile(path_to_config):
      file = open(path_to_config)
      gpr_config = yaml.load(file)

      self.device_id = gpr_config['device_id']
      self.baud_rate = gpr_config['baud_rate']
      self.sampling_frequency = gpr_config['sampling_frequency']
      self.filter = gpr_config['filter']
      self.num_points = gpr_config['points']
      self.point_offset = gpr_config['point_offset']
      self.trace_topic = gpr_config['output_trace_topic']
      self.bag_path = bag_path

    else:
      self.device_id = rospy.get_param("~device_id")
      self.baud_rate = rospy.get_param("~baud_rate")
      self.sampling_frequency = rospy.get_param("~sampling_frequency") 
      self.filter = rospy.get_param("~filter")
      self.num_points = rospy.get_param("~points")
      self.point_offset = rospy.get_param("~point_offset")
      self.trace_topic = rospy.get_param("~output_trace_topic")
      self.bag_path = bag_path

