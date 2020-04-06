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
def tracking_loop(ms, ticks, kp, ki):
    vel_ests, vel_ints, ticks_errs = [], [], []
    
    vel_est = 0.0
    ticks_est = 0.0
    vel_int = 0.0
    last_ms_i = float(ms[0] - 20.0)
    for ms_i, ticks_i in zip(ms, ticks):
        dt = (ms_i - last_ms_i)/1000.0 # in s
        last_ms_i = ms_i
        ticks_est += vel_est * dt
        ticks_err = float(ticks_i) - ticks_est
        vel_int += ticks_err * ki * dt
        vel_est = ticks_err * kp + vel_int
        vel_ests.append(vel_est)
        vel_ints.append(vel_int)
        ticks_errs.append(ticks_err)
    return np.asarray(vel_ests), np.asarray(vel_ints), np.asarray(ticks_errs)


# %%
#vel_ests, vel_ints, ticks_errs = tracking_loop(log.ms.values, log.ticks.values, kp=12.0, ki=100.0)
vel_ests, vel_ints, ticks_errs = tracking_loop(log.ms.values, log.ticks.values, kp=12.0, ki=200.0)
log['pidve_p12_i200_s'] = vel_ests
log['pidvi_p12_i200_s'] = vel_ints

vel_ests, vel_ints, ticks_errs = tracking_loop(log.ms.values, log.ticks.values, kp=12.0, ki=400.0)
log['pidve_p12_i400_s'] = vel_ests
log['pidvi_p12_i400_s'] = vel_ints

vel_ests, vel_ints, ticks_errs = tracking_loop(log.ms.values, log.ticks.values, kp=8.0, ki=80.0)
log['pidve_p8_i80_s'] = vel_ests
log['pidvi_p8_i80_s'] = vel_ints

vel_ests, vel_ints, ticks_errs = tracking_loop(log.ms.values, log.ticks.values, kp=4.0, ki=80.0)
log['pidve_p4_i80_s'] = vel_ests
log['pidvi_p4_i80_s'] = vel_ints

vel_ests, vel_ints, ticks_errs = tracking_loop(log.ms.values, log.ticks.values, kp=40.0, ki=200.0)
#sl = slice(5500, 5800)
sl = slice(1200, 1400)
#pl.plot(log.ms.values[sl], vel_ests[sl], '-', log.ms.values[sl], log.diff0_s.values[sl], '.')
pl.plot(log.ms.values[sl], vel_ints[sl], '-', log.ms.values[sl], vel_ests[sl], '-', log.ms.values[sl], log.diff0_s.values[sl], '.')


# %%
def fit_vel(x): 
    #dt_s = ((x.size-1) * 20.0)/1000.0
    #dx = x[-1] - x[0]
    #return (dx/dt_s)
    dt_s = (np.arange(len(x))*20.0)/1000.0
    A = np.vstack([dt_s, np.ones(len(dt_s))]).T
    m, c = np.linalg.lstsq(A, x, rcond=None)[0]
    return m
vel_ests = log.ticks.rolling(window=11, center=False).apply(fit_vel, raw=True)
pl.plot(log.ms.values[sl], vel_ests[sl], '-', log.ms.values[sl], log.diff0_s.values[sl], '.')


# %%
def tracking_loop2(ms, ticks, kp):
    # adaptive i-term
    # 600 -> i=8
    # 100 -> i=4
    #dx = (600. - 100.)
    # dy = (8. - 4.)
    # m = dy/dx
    # b = 8 - m*600
    # m=0.008, b=3.2
    vel_ests, vel_ints, ticks_errs = [], [], []
    
    ki = 8.0
    ki_m = 0.010
    ki_b = 4.0
    
    vel_est = 0.0
    ticks_est = 0.0
    vel_int = 0.0
    last_ms_i = float(ms[0] - 20.0)
    
    for ms_i, ticks_i in zip(ms, ticks):
        dt = (ms_i - last_ms_i)/1000.0 # in s
        last_ms_i = ms_i
        ticks_est += vel_est * dt
        ticks_err = float(ticks_i) - ticks_est
        vel_int += ticks_err * ki * dt
        vel_est = ticks_err * kp + vel_int
        
        ki = np.clip(ki_m*vel_est + ki_b, 4.0, 8.0)
        
        vel_ests.append(vel_est)
        vel_ints.append(vel_int)
        ticks_errs.append(ticks_err)
    return np.asarray(vel_ests), np.asarray(vel_ints), np.asarray(ticks_errs)


# %%
vel_ests, vel_ints, ticks_errs = tracking_loop2(log.ms.values, log.ticks.values, kp=4.0)
#sl = slice(5500, 5800)
sl = slice(1200, 1400)
#pl.plot(log.ms.values[sl], vel_ests[sl], '-', log.ms.values[sl], log.diff0_s.values[sl], '.')
pl.plot(log.ms.values[sl], vel_ints[sl], '-', log.ms.values[sl], vel_ests[sl], '-', log.ms.values[sl], log.diff0_s.values[sl], '.')

#log['apidve_p4_i80_s'] = vel_ests
#log['apidvi_p4_i80_s'] = vel_ints

# %%

# %%

# %%
log.ticks.rolling??

# %%
pl.plot(log.ms.values[sl], ticks_errs[sl])

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
#log0.plot(x='ms', y='diff0', style=['.-'])

# %%
log.keys()

# %%
#log.plot?

# %%
log0.plot(x='ms', y=['diff0_s', 
                     #'diff5_s', 
                     #'pidve_p12_i200_s', 
                     #'pidvi_p12_i200_s',
                     #'pidve_p12_i400_s',
                     #'pidvi_p12_i400_s',
                     'pidve_p8_i80_s',
                     #'pidvi_p8_i80_s',
                     'pidve_p4_i80_s',
                     #'pidvi_p4_i80_s',
                    ], 
          style=['.', '-', '-', '-'],
         figsize=(8, 6))

# %%
#log0.plot(x='ms', y='diff5_s', style=['.-'])

# %%
log1 = log.loc[5500:5800, :]

# %%
log1.plot(x='ms', y=['ticks'], style=['.-'])


# %%
log1.plot(x='ms', y='diff0', style=['.-'])

# %%
log1.plot(x='ms', y=['diff0_s', 
                     #'diff5_s', 
                     #'pidve_p12_i200_s', 
                     #'pidvi_p12_i200_s',
                     #'pidve_p12_i400_s',
                     #'pidvi_p12_i400_s',
                     'pidve_p8_i80_s',
                     #'pidvi_p8_i80_s',
                     'pidve_p4_i80_s',
                     #'pidvi_p4_i80_s',
                    ], 
          style=['.', '-', '-', '-'],
         figsize=(8, 6))

# %%
log2 = log.loc[1200:1400, :]
log2.plot(x='ms', y='ticks', style=['.-'])

# %%
log2.plot(x='ms', y=['diff0_s', 
                     #'diff5_s', 
                     #'pidve_p12_i200_s', 
                     #'pidvi_p12_i200_s',
                     #'pidve_p12_i400_s',
                     #'pidvi_p12_i400_s',
                     'pidve_p8_i80_s',
                     #'pidvi_p8_i80_s',
                     #'pidve_p4_i80_s',
                     #'pidvi_p4_i80_s',
                    ], 
          style=['.', '-', '-', '-'],
         figsize=(8, 6))

# %%
#log2.plot(x='ms', y=['diff0_s', 'diff5_s'])

# %%

# %%
# TODO: look at pd rolling
# def mad(x): return np.fabs(x - x.mean()).mean()
# s.rolling(window=60).apply(mad, raw=True).plot(style='k')

# %%
log2

# %%
