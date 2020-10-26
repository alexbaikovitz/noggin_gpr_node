import numpy as np

class TraceToImage:

  RANGE = 255.0
  INT16_MAX_VALUE = 32767

  def __init__(self):
    self.abs_max_value = 0
    self.scale = None

  def Update(self, trace, pos=0):
    max_value_hyp = np.max(np.abs(trace))
    if max_value_hyp > self.abs_max_value:
      self.abs_max_value = max_value_hyp
      self.scale = TraceToImage.INT16_MAX_VALUE / (max_value_hyp)

    print(self.scale, trace.shape)
    normalized_trace = self.scale * trace # + TraceToImage.RANGE // 2
    return normalized_trace

  def traces_to_image2d(self, img, maximum_value=None):
    if maximum_value == None:
      maximum_value = np.max(np.abs(img))
    
    scale = TraceToImage.INT16_MAX_VALUE / maximum_value
    out = scale * img
    return out.astype(np.uint16)

      
    

    


    