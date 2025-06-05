import json
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import sqlite3
from crux import acoustics1d, metadata
from subprocess import getoutput as go
from pathlib import Path

from utils import opbox

params = {
    'delay': 0,  # us
    'range': 20,  # us
    'gain':  50  # dB
}
EXP_ID = 'MD_AP_Sandia_Local_Test_2'

data_folder = Path('./acoustic_data/')
acoustics_path = data_folder / (EXP_ID + '.db')
print('Making directory: ', go(f'mkdir {data_folder}'))

opbox = opbox.Opbox_v21(verbose=False)
opbox.SetExpParameters(params)

delay = 5  # Time in seconds between each measurement
sl = 0.01

now = time.time() - 4.5

i = 1

while True:
    if now + delay < time.time():
        old_time = now
        now = time.time()
        try:
            data = opbox.Trigger_And_Read()
            header, samples, all_samples = opbox.header, opbox.data, opbox.all_samples
        except Exception as e:
            if not str(e).startswith('Data length is incorrect.'):
                raise Exception(e)
            else:
                now = old_time
                print('Fail. Jumping to next...')
                continue

        meta = json.dumps({'Gain': opbox._gain, 'Range': opbox._meas_range, 'Delay': opbox._delay,
                          'Input voltage': opbox._voltage, 'Registers': opbox.read_all_parameters(verbose=False)})
        # samples = (samples - 127) / 128

        waveform = samples.tobytes()

        payload = acoustics1d.ORM(
            time=time.time(),
            amps=sqlite3.Binary(waveform),  # type: ignore[arg-type]
            meta=meta,
        )

        formatted_time = time.strftime(
            '%m/%d/%Y - %H:%M:%S', time.localtime(now))
        message = f'{formatted_time}\nMeasurement {i} successful \n({now - old_time:.1f}s since last update)'
        print(message)
        i += 1
        with acoustics1d.Database(acoustics_path) as db:
            row_count = db.write(payload)

    # time.sleep(sl)
    plt.pause(sl)
