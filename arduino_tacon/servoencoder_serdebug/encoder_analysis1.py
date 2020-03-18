# ---
# jupyter:
#   jupytext:
#     comment_magics: true
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.2'
#       jupytext_version: 1.2.3
#   kernelspec:
#     display_name: Python 3
#     language: python
#     name: python3
# ---

# %%
# #%matplotlib notebook
# %matplotlib inline

# %%
import numpy as np
import pandas as pd
from matplotlib import pyplot as pl

# %%
# ticks were sampled every 20ms
# diff0 = discrete difference
# so diff0 = [ticks/20ms]

# %%
log = pd.read_csv('./out.log', names=['ms', 'ticks']).sort_values(by='ms').reset_index(drop=True)
log['ticks'] = -log['ticks']

# %%
# encoder is backwards
log['diff0'] = log.ticks.diff()
log['diff5'] = log.ticks.diff(periods=5)
log['diff0_s'] = (log['diff0']*1000.0)/20.0
log['diff5_s'] = (log['diff5']*1000.0)/(20.0 * 5)

# %%
log

# %%
#logs = log.sort_values(by='ms')

# %%
log

# %%
log.plot(x='ms', y='ticks', style=['.'])

# %%
log.plot(x='ms', y='diff0', style=['.'])

# %%
log.plot(x='ms', y='diff0_s', style=['.-'])

# %%
log.plot(x='ms', y='diff5_s', style=['.-'])

# %%
log.ticks.plot(style=['-'])

# %%
log0 = log.loc[2600:2800, :]

# %%
log0.plot(x='ms', y='ticks', style=['.-'])

# %%
log0.plot(x='ms', y='diff0', style=['.-'])

# %%
log0.plot(x='ms', y='diff0_s', style=['.-'])

# %%
log0.plot(x='ms', y='diff5_s', style=['.-'])

# %%
log1 = log.loc[5500:5800, :]

# %%
log1.plot(x='ms', y=['ticks'], style=['.-'])

# %%
log1.plot(x='ms', y='diff0', style=['.-'])

# %%
log1.plot(x='ms', y='diff0_s', style=['.-'])

# %%
log1.plot(x='ms', y='diff5_s', style=['.-'])

# %%
log2 = log.loc[1200:1400, :]
log2.plot(x='ms', y='ticks', style=['.-'])

# %%
log2.plot(x='ms', y='diff0', style=['.-'])

# %%
log2.plot(x='ms', y='diff0_s', style=['.-'])

# %%
log2.plot(x='ms', y='diff5_s', style=['.-'])

# %%
# TODO: look at pd rolling
# def mad(x): return np.fabs(x - x.mean()).mean()
# s.rolling(window=60).apply(mad, raw=True).plot(style='k')

# %%
