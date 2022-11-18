import numpy as np
import matplotlib.pyplot as plt

import science_calc as sc

# Generate trajectories
t = np.linspace(0, 10.0, 1000, endpoint=False)
x = np.random.normal(0.0, np.sqrt(0.01), size=(1000, 1000, 3)).astype(np.float32)
x = np.cumsum(x, axis=0)

msd = sc.msd(x)

plt.plot(t, msd)
plt.xlabel("Time")
plt.ylabel("MSD")
plt.show()