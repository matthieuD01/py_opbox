# Author: Matthieu Dominici
# Date Started: 12/27/2024
# Notes: Simply triggers a pulse with the OpBox to test.

import utils.opbox as opbox
import matplotlib.pyplot as plt
import numpy as np
# from pithy3 import showme
showme = plt.show

params = {
    'delay': 0,  # us
    'range': 15,  # us
    'gain': 35  # dB
}

opbox = opbox.Opbox_v21(verbose=False)
opbox.SetExpParameters(params)

# 2 ways to get the averaged data: as the object returned by trigger_and_read, or with opbox.data.
try:
    data = opbox.Trigger_And_Read()
    header, samples, all_samples = opbox.header, opbox.data, opbox.all_samples
    print('First trigger successful.')
except Exception as e:
    if not str(e).startswith('Data length is incorrect.'):
        raise Exception(e)
    else:
        print('First trigger unsuccessful, retriggering... (when initializing device, half of first triggers are unsuccessful).')
        data = opbox.Trigger_And_Read()
        header, samples, all_samples = opbox.header, opbox.data, opbox.all_samples


header, avg_data, all_samples = opbox.header, opbox.data, opbox.all_samples
depth = int(opbox._meas_range * 1e-6 * opbox._sampling_freq)
timesteps = np.linspace(
    params['delay'], params['delay'] + depth/(opbox._sampling_freq * 1e-6), depth)
print(
    f'{len(all_samples)} acquisitions per measurement, each acquisition has length of {len(all_samples[0])} bytes.')
fig, ax = plt.subplots()
ax.plot(timesteps, opbox.data, ls='-', marker='.', ms=1)
ax.set_ylim(-1, 1)
ax.set_xlabel('Time ($\mu$s)')
ax.set_ylabel('Amplitude $\in$ [-1, 1]')
fig.suptitle(
    f'Range = {opbox._meas_range}us, delay = {opbox._delay}us, gain = {opbox._gain}dB')
showme()
