import pywt
import numpy as np
import matplotlib.pyplot as plt
from skimage.restoration import (denoise_wavelet, estimate_sigma)
from skimage.metrics import peak_signal_noise_ratio
# import scipy.fft
from scipy import fftpack
from scipy.signal import butter, firwin, lfilter, filtfilt, hilbert

def binomial_dewow_filter(signal):
  model = np.polyfit(np.arange(signal.size), signal, 3)
  predicted = np.polyval(model, np.arange(signal.size))
  return signal + predicted

def butter_bandpass(order=8):
  SAMPLE_RATE = 5000 # MHz
  SYSTEM_PEAK_FREQUENCY = 500 # MHz
  # LOW_CUT = 500 - (1.75*SYSTEM_PEAK_FREQUENCY/2)
  # HIGH_CUT = 500 + (20*SYSTEM_PEAK_FREQUENCY/2)
  LOW_CUT = 1
  HIGH_CUT = 2000
  
  nyq = 0.5 * SAMPLE_RATE
  low = LOW_CUT / nyq
  high = HIGH_CUT / nyq
  b, a = butter(order, .3) #.30
  # b, a = butter(order, [low, high], btype='band')
  return b, a

def butterworth_bandpass_filter(signal, order=8):
    b, a = butter_bandpass(order)
    y = filtfilt(b, a, signal)
    return y

def discrete_wavelet_transform(trace_1d, threshold=.45):
  wavelet = pywt.Wavelet('db2')
  coeffs = pywt.wavedec(trace_1d, wavelet, level=3)

  for i in range(len(coeffs)):
    if i == 0: 
      continue
    K = np.round(threshold * len(coeffs[i])).astype(int)
    coeffs[i][K:] = np.zeros(len(coeffs[i]) - K)

  return pywt.waverec(coeffs, wavelet)


def discrete_wavelet_transform_image(noisy_image):

  maximum_value = np.max(noisy_image[:,0])
  transformed_image = np.float32(np.clip(noisy_image / (2 * maximum_value) + 0.5, 0, 1))

  sigma_est = estimate_sigma(transformed_image, multichannel=True, average_sigmas=True)

  unscaled = denoise_wavelet(transformed_image, multichannel=True,
                             method='BayesShrink', mode='soft', 
                             rescale_sigma=True)

  print(f'PSNR ratio: {peak_signal_noise_ratio(transformed_image, unscaled)}')

  return unscaled - 0.5

def correct_dc_offset(signal):
  dc_offset = np.sum(signal)
  individual_offset = dc_offset / signal.size
  return signal - individual_offset, dc_offset

def find_exponential_envelope(filtered_image, plot=False, unfiltered_image=np.array([])):
  average_amplitude_trace = np.average(filtered_image, axis=1)
  average_amplitude_trace /= np.max(np.abs(average_amplitude_trace))
  # scale = np.max(np.abs(average_amplitude_trace))
  analytic_signal = hilbert(average_amplitude_trace)
  amplitude_envelope = np.abs(analytic_signal)

  decay, coeff = np.polyfit(np.arange(amplitude_envelope.size)[:75], np.log(amplitude_envelope[:75]), 1)
  exp_fn = np.exp(coeff) * np.exp(decay * np.arange(amplitude_envelope.size))
  # gain_fn = np.exp(coeff) * np.exp(np.abs(decay) * np.arange(amplitude_envelope.size))
  gain_fn = 1 / exp_fn

  CUT_OFF_IDX = 80

  gain_fn[CUT_OFF_IDX:] = np.full(gain_fn.size - CUT_OFF_IDX, gain_fn[CUT_OFF_IDX])
  plt.figure()
  plt.plot(gain_fn)
  gain_fn = np.transpose([gain_fn])

  gained_image = np.multiply(gain_fn, filtered_image)
  gained_image /= np.max(np.abs(gained_image)) # Normalize image.

  if (unfiltered_image.size and plot):
    start_idx_unproc = unfiltered_image[:,100].size - filtered_image[:,100].size
    # scale = np.max(unfiltered_image[:,100]) / np.max(filtered_image[:,100])
    print(np.max(np.abs(filtered_image[:,100])), np.max(np.abs(unfiltered_image[:,100])))

    proc_ls = [unfiltered_image[start_idx_unproc:,100]/np.max(np.abs(unfiltered_image[:,100])), filtered_image[:,100]/np.max(np.abs(filtered_image[:,100])), gained_image[:,100]]
    name_ls = ["Unfiltered", "Filtered", "Gain on Filtered"]
    plot_filter_result(proc_ls, name_ls)

  gained_image[np.where(gained_image > .4)] = .4
  gained_image[np.where(gained_image < -.4 )] = -1 * .4

  if plot:
    t = np.arange(average_amplitude_trace.size)
    plt.figure()
    plt.plot(t, average_amplitude_trace)
    plt.plot(t, amplitude_envelope, label='envelope')
    plt.plot(t, exp_fn)
    plt.show()


  return gained_image


def triangular(ar, header, freqmin, freqmax, zerophase=True, verbose=False):
    """
    Vertical triangular FIR bandpass. This filter is designed to closely emulate that of RADAN.
    Filter design is implemented by :py:func:`scipy.signal.firwin` with :code:`numtaps=25` and implemented with :py:func:`scipy.signal.lfilter`.
    .. note:: This function is not compatible with scipy versions prior to 1.3.0.
    :param np.ndarray ar: The radar array
    :param dict header: The file header dictionary
    :param int freqmin: The lower corner of the bandpass
    :param int freqmax: The upper corner of the bandpass
    :param bool zerophase: Whether to run the filter forwards and backwards in order to counteract the phase shift
    :param bool verbose: Verbose, defaults to False
    :rtype: :py:class:`numpy.ndarray`
    """
    #samp_freq = 1 / ((header['rhf_depth'] * 2) / header['cr'] / header['rh_nsamp'])
    samp_freq = header['samp_freq']
    freqmin = freqmin * 10 ** 6
    freqmax = freqmax * 10 ** 6
    
    numtaps = 25

    filt = firwin(numtaps=numtaps, cutoff=[freqmin, freqmax], window='triangle', pass_zero='bandpass', fs=samp_freq)

    far = lfilter(filt, 1.0, ar, axis=0).copy()
    if zerophase:
        far = lfilter(filt, 1.0, far[::-1], axis=0)[::-1]

    return far


def plot_filter_result(proc_ls, name_ls):

  plt.figure()

  plt.xlabel("Time [ns]")
  plt.ylabel("Scaled Amplitude")

  for proc in proc_ls:
    plt.plot(proc)

  plt.legend(name_ls)
  plt.show()






  
  
  




