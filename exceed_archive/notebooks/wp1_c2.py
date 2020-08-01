# ---
# jupyter:
#   jupytext:
#     comment_magics: true
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.2'
#       jupytext_version: 1.0.5
#   kernelspec:
#     display_name: Python 2
#     language: python
#     name: python2
# ---

# %%
# %autosave 0

# %%
import subprocess
import time
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import Dataset
from torch.utils.data import DataLoader
from torchvision import transforms
from torchvision.models import resnet18
import numpy as np
from spath import Path
from PIL import Image
import hashlib
import random

# %%
import exceed_bot.data as ebdata
import exceed_bot.train_utils as ebutils

# %%
nbins = None
img_ds_factor = 2
name = 'wp1_c2'
model_dir = Path('/home/dmaturan/ws/exceed_bot/src/exceed_bot/models')
dset_dir = Path('/home/dmaturan/data/exceed_bot')

# %%
train_list, valid_list = ebdata.load_cached_wp_filelist(dset_dir)

# %%
train_ds = ebdata.WpDataset(train_list, nbins=nbins, img_ds_factor=img_ds_factor)
train_loader = DataLoader(train_ds, batch_size=4, shuffle=True, pin_memory=True)
valid_ds = ebdata.WpDataset(valid_list, nbins=nbins, img_ds_factor=img_ds_factor)
valid_loader = DataLoader(valid_ds, batch_size=1, shuffle=True)

# %%
for b in train_loader:
    bx = b[0]
    by = b[1]
    break
print(bx.shape)
print(by.shape)

# %%
net = resnet18(pretrained=True)
net.fc = nn.Linear(512, 1)

# %%
state_dict = ebutils.try_load_model(model_dir, name)
if state_dict is None:
    print('unable to load prior checkpt')
else:
    #net.load_state_dict(params)
    ebutils.partial_load(net, state_dict)
# %%
ebutils.save_model(net, model_dir, name)
ebutils.export_torchscript(net, bx[0:1], model_dir, name)

# %%
net = net.train()
net = net.cuda()

# %%
opt = optim.Adam(net.parameters(), lr=1e-4)
itr = 0
epoch = 0

# %%
for _ in range(100):
    for bx, by in train_loader:
        losses = []
        bx = bx.cuda()
        by = by.cuda()
        opt.zero_grad()
        yhat = net(bx)
        if nbins is None:
            loss = F.mse_loss(yhat, by)
        else:
            loss = F.cross_entropy(yhat, by)
        losses.append(loss.item())
        loss.backward()
        opt.step()
        if itr % 100 == 0:
            avg_loss = np.mean(np.asarray(losses))
            print('epoch: %d, itr: %d, loss: %f' % (epoch, itr, avg_loss))
            losses = []
        if itr % 800 == 0:
            if nbins is None:
                ebutils.validate_continuous(net, valid_loader, max_itr=200)
            else:
                ebutils.validate_discrete(net, valid_loader, max_itr=200)
            net = net.train()
        itr += 1
    ebutils.save_model(net, model_dir, name)
    ebutils.export_torchscript(net, bx[0:1], model_dir, name)
    epoch += 1

# %%
bi = 2
img = ebdata.unpreprocess(bx[bi])
print(yhat[bi].item(), by[bi].item())
img

# %%
