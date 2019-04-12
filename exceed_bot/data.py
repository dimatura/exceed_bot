import subprocess
from spath import Path

import numpy as np

import torch
import torchvision
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset

import rospy
import cv_bridge

from PIL import Image
import cv2


def preprocess(img):
    img = np.asarray(img)
    img = img.astype('f4')/255.
    img[0] = (img[0] - 0.485)/0.229
    img[1] = (img[1] - 0.456)/0.224
    img[2] = (img[2] - 0.406)/0.225
    img = img.transpose((2, 0, 1))
    return img


def unpreprocess(timg):
    img = timg.cpu().data.numpy()
    img = img.transpose((1, 2, 0))
    img[0] = (img[0]*0.229) + 0.485
    img[1] = (img[1]*0.224) + 0.456
    img[2] = (img[2]*0.225) + 0.406
    img = np.clip(img * 255., 0, 255).astype('u1')
    img = Image.fromarray(img)
    return img


def discretize(steer, bins):
    disc_steer = np.digitize(steer, bins, right=True)
    return disc_steer


def undiscretize(disc_steer, bins):
    # TODO could interpolate
    return bins[disc_steer]


def load_cached_clone_filelist(dset_dir):
    split_fname = Path(dset_dir)/('split_clone.json')

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
                if get_hash(fname) % 100 < 92:
                    train_dset.append((fname, yfname))
                else:
                    valid_dset.append((fname, yfname))
        split_fname.write_json({'train': train_dset, 'valid': valid_dset}, indent=2)
    return train_dset, valid_dset



class CloneDataset(object):
    def __init__(self, fnames, nbins, img_ds_factor):
        self.nbins = nbins
        self.img_ds_factor = img_ds_factor
        self.fnames = fnames
        if self.nbins is not None:
            self.bins = np.linspace(-1., 1., self.nbins+1)[1:]

    def __len__(self):
        return len(self.fnames)

    def get_img(self, ix):
        xfname, yfname = self.fnames[ix]
        img = Image.open(xfname)
        w, h = img.size
        # resize to 320, 240 or 160, 120
        img = img.resize((w//self.img_ds_factor, h//self.img_ds_factor))
        return img

    def get_steer(self, ix):
        xfname, yfname = self.fnames[ix]
        steer = Path(yfname).read_json()['steer']
        return steer

    def __getitem__(self, ix):
        xfname, yfname = self.fnames[ix]
        img = self.get_img(ix)
        img = preprocess(img)
        steer = self.get_steer(ix)
        if self.nbins is not None:
            steer = discretize(steer, self.bins)
        return (img, steer)
