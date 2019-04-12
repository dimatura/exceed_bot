import subprocess
import json
from spath import Path
from collections import OrderedDict
from xml.etree import ElementTree
import hashlib
import funcy

import numpy as np

import torch
import torchvision
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset

import rospy
import cv_bridge

from PIL import Image
import cv2

import exceed_bot.train_utils as ebutils

import Augmentor as aug


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
                if ebutils.get_hash(fname) % 100 < 92:
                    train_dset.append((fname, yfname))
                else:
                    valid_dset.append((fname, yfname))
        split_fname.write_json({'train': train_dset, 'valid': valid_dset}, indent=2)
    return train_dset, valid_dset


def load_cached_wp_filelist(dset_dir):
    split_fname = Path(dset_dir)/('split_wp.json')

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
            yfname = Path(fname.stripext() + '.xml')
            if yfname.exists():
                if ebutils.get_hash(fname) % 100 < 98:
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
        steer = np.float32(self.get_steer(ix))
        if self.nbins is not None:
            steer = discretize(steer, self.bins)
        return (img, steer)


class WpDataset(object):
    def __init__(self, fnames, nbins, img_ds_factor, augment=True):
        self.nbins = nbins
        self.img_ds_factor = img_ds_factor
        self.augment = augment
        all_fnames = fnames
        self.fnames = []
        for xfname, yfname in all_fnames:
            steer = self.parse_xml(yfname)
            if steer is None:
                continue
            self.fnames.append((xfname, yfname))

        if self.nbins is not None:
            self.bins = np.linspace(-1., 1., self.nbins+1)[1:]

        p = aug.Pipeline()
        #p.flip_left_right(0.5)
        p.random_brightness(0.5, 0.6, 1.6)
        p.random_erasing(0.1, 0.2)
        #p.histogram_equalisation(0.9)
        p.random_color(0.5, 0.6, 1.6)
        p.random_contrast(0.5, 0.6, 1.6)
        self.augtf = p.torch_transform()


    def __len__(self):
        return len(self.fnames)

    def get_img(self, ix):
        xfname, yfname = self.fnames[ix]
        img = Image.open(xfname)
        w, h = img.size
        # resize to 320, 240 or 160, 120
        img = img.resize((w//self.img_ds_factor, h//self.img_ds_factor))
        if self.augment:
            img = self.augtf(img)
        return img

    @funcy.memoize
    def parse_xml(self, fname):
        tree = ElementTree.parse(fname)
        object_tree_list = tree.findall('object')

        steer = None
        for object_annotation in object_tree_list:
            class_name = object_annotation.find('name').text
            if class_name == 'waypointlo':
                pt = object_annotation.find('point')
                ptx = float(pt.find('x1').text)
                pty = float(pt.find('y1').text)
                steer = ptx
        return steer

    def get_steer(self, ix):
        xfname, yfname = self.fnames[ix]
        #steer = Path(yfname).read_json()['steer']
        steer = self.parse_xml(yfname)
        return steer

    def __getitem__(self, ix):
        xfname, yfname = self.fnames[ix]
        img = self.get_img(ix)
        img = preprocess(img)
        steer = np.float32(self.get_steer(ix))
        steer /= float(self.img_ds_factor)
        # norm to 01 CHW
        steer /= float(img.shape[2])
        # to -1,1
        steer = np.float32((2.*steer - 1.))
        if self.nbins is not None:
            steer = discretize(steer, self.bins)
        return (img, steer)
