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
nbins = 16
img_ds_factor = 2


# %%
def get_hash(fname):
    h = hashlib.md5()
    h.update(fname)
    return int(h.hexdigest(), 16)


# %%
dset_dir = Path('/home/dmaturan/data/exceed_bot')

# %%
split_fname = dset_dir/('split.json')

if split_fname.exists():
    print('loaded from cache')
    split = split_fname.read_json()
    train_dset = split['train']
    valid_dset = split['valid']
else:
    print('loading from scratch')
    train_dset = []
    valid_dset = []

    for fname in dset_dir.walkfiles('*.jpg'):
        yfname = Path(fname.stripext() + '.json')
        if yfname.exists():
            if get_hash(fname) % 100 < 80:
                train_dset.append((fname, yfname))
            else:
                valid_dset.append((fname, yfname))
    split_fname.write_json({'train': train_dset, 'valid': valid_dset}, indent=2)


# %%
class CloneDataset(object):
    def __init__(self, fnames):
        self.fnames = fnames
        #self.bins = np.linspace(-1., 1., 32+1)[1:]
        self.bins = np.linspace(-1., 1., nbins+1)[1:]
        
    def __len__(self):
        return len(self.fnames)
    
    def get_img(self, ix):
        xfname, yfname = self.fnames[ix]
        img = Image.open(xfname)
        w, h = img.size
        # resize to 320, 240
        img = img.resize((w//img_ds_factor, h//img_ds_factor))
        return img
    
    def get_steer(self, ix):
        xfname, yfname = self.fnames[ix]
        steer = Path(yfname).read_json()['steer']
        return steer
    
    def __getitem__(self, ix):
        xfname, yfname = self.fnames[ix]
        img = np.asarray(self.get_img(ix))
        img = img.astype('f4')/255.
        img[0] = (img[0] - 0.485)/0.229
        img[1] = (img[1] - 0.456)/0.224
        img[2] = (img[2] - 0.406)/0.225
        img = img.transpose((2, 0, 1))
        steer = self.get_steer(ix)
        steer_disc = np.digitize(steer, self.bins, right=True)
        return (img, steer_disc)


# %%
def validate(net):
    net = net.eval()
    correct = 0
    tic = time.time()
    val_itr = 0
    for bx, by in valid_loader:
        bx = bx.cuda()
        by = by.cuda()
        yhat = net(bx)
        if yhat.argmax().item() == by.item():
            correct += 1
        val_itr += 1
        if val_itr == 100:
            break
    toc = time.time()
    acc = float(correct)/val_itr 
    print('acc: %f' % acc)
    print('avg time: %f' % ((toc - tic)/val_itr))
    net = net.train()


# %%
train_ds = CloneDataset(train_dset)
train_loader = DataLoader(train_ds, batch_size=4, shuffle=True)
valid_ds = CloneDataset(valid_dset)
valid_loader = DataLoader(valid_ds, batch_size=1, shuffle=True)

# %%
for b in train_loader:
    bx = b[0]
    by = b[1]
    break

# %%
bx.shape

# %%
net = resnet18(pretrained=True)
net.fc = nn.Linear(512, nbins)

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
        loss = F.cross_entropy(yhat, by)
        losses.append(loss.item())
        loss.backward()
        opt.step()
        if itr % 100 == 0:
            avg_loss = np.mean(np.asarray(losses))
            print('epoch: %d, itr: %d, loss: %f' % (epoch, itr, avg_loss))
            losses = []
        if itr % 800 == 0:
            validate(net)
            net = net.train()
        itr += 1
    torch.save(net.state_dict(), 'clone1.pt')
    epoch += 1

# %%
#torch.save?

# %%
img = bx[0].cpu().numpy()
img = img.transpose((1, 2, 0))
img[0] = (img[0]*0.229) + 0.485
img[1] = (img[1]*0.224) + 0.456
img[2] = (img[2]*0.225) + 0.406
img = np.clip(img * 255., 0, 255).astype('u1')
img = Image.fromarray(img)

# %%
print(yhat[0].argmax().item(), by[0].item())
img

# %%

# %%

# %%

# %%
