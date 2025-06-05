import sqlite3
import matplotlib.pyplot as plt
import numpy as np
showme = plt.show


path = ".\\acoustic_data\\AP_TR_2025_03_20_Test.db"
sqlite_db = path
db = sqlite3.connect(sqlite_db)
print(db)
cur = db.cursor()
rowid = 25
cur.execute(f'SELECT amps FROM acoustics WHERE rowid={rowid}')
waveform = cur.fetchone()[0]
samples = np.frombuffer(waveform, dtype=np.float16)
fig, ax = plt.subplots()
ax.plot(samples)
showme()
