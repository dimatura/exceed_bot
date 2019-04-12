import time
import subprocess
from spath import Path
import hashlib

import numpy as np

import torch
import torchvision
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset

import rospy
import cv_bridge

from PIL import Image
import cv2


def save_model(net, base_dir, name):
    base_dir = Path(base_dir).expand()
    fname = base_dir/(name + '.pt')
    tmp_fname = Path(fname + '.tmp')
    torch.save(net.state_dict(), tmp_fname)
    subprocess.call(['mv', tmp_fname, fname])


def export_torchscript(net, example, base_dir, name):
    net = net.eval()
    ts_net = torch.jit.trace(net, example)

    base_dir = Path(base_dir).expand()
    fname = base_dir/(name + '.ts')
    tmp_fname = Path(fname + '.tmp')

    ts_net.save(tmp_fname)

    subprocess.call(['mv', tmp_fname, fname])


def try_load_model(base_dir, name):
    base_dir = Path(base_dir).expand()
    fname = base_dir/(name + '.pt')
    if fname.exists():
        print('found model')
        return torch.load(str(fname))
    return None


def get_hash(fname):
    h = hashlib.md5()
    h.update(fname)
    return int(h.hexdigest(), 16)


def validate_discrete(net, valid_loader, max_itr=200):
    with torch.no_grad():
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
            if val_itr == max_itr:
                break
        toc = time.time()
        acc = float(correct)/val_itr
        print('acc: %f' % acc)
        print('avg time: %f' % ((toc - tic)/val_itr))
    net = net.train()


def validate_continuous(net, valid_loader, max_itr=200):
    with torch.no_grad():
        net = net.eval()
        mad = 0.
        tic = time.time()
        val_itr = 0
        for bx, by in valid_loader:
            bx = bx.cuda()
            by = by.cuda()
            yhat = net(bx)
            l1 = np.abs(by.item() - yhat.item())
            mad += l1
            val_itr += 1
            if val_itr == max_itr:
                break
        toc = time.time()
        mad = mad/float(val_itr)
        print('mad: %f' % mad)
        print('avg time: %f' % ((toc - tic)/val_itr))
    net = net.train()


def partial_load(model, state_dict):
    """ allows state_dict that doesn't have all keys in model.
    """
    model_state = model.state_dict()
    for name, param in state_dict.iteritems():
        if name not in model_state:
            print('Parameter %s not part of model, skipping' % name)
            continue
        if isinstance(param, torch.nn.Parameter):
            # backwards compatibility for serialized parameters
            param = param.data
        if param.size() != model_state[name].size():
            print('wrong size for parameter %s, skipping' % name)
            continue
        print('loading parameter %s' % name)
        model_state[name].copy_(param)
