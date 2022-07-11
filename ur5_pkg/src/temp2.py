import numpy as np
from scipy import signal
from numpy.random import default_rng

rng = default_rng()

x = rng.standard_normal(1000)

y = np.concatenate([rng.standard_normal(100), x])

correlation = signal.correlate(x, y, mode="full")

lags = signal.correlation_lags(x.size, y.size, mode="full")

lag = lags[np.argmax(correlation)]