import numpy as np

class TraceToImage:

  RANGE = 255.0

  def __init__(self):
    self.abs_max_value = 0
    self.scale = None

  def Update(self, trace, pos=0):
    max_value_hyp = np.max(np.abs(trace))
    if max_value_hyp > self.abs_max_value:
      self.abs_max_value = max_value_hyp
      self.scale = TraceToImage.RANGE / (2.0 * max_value_hyp)

    normalized_trace = self.scale * trace + TraceToImage.RANGE // 2
    print(self.scale)
    return normalized_trace

    


    