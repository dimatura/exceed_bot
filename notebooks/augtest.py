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
import numpy as np
import Augmentor as aug
from PIL import Image
import torch
import torchvision

# %%

# %%
p = aug.Pipeline()
#p.flip_left_right(0.5)
#p.random_brightness(0.95, 0.6, 1.6)
#p.random_erasing(0.5, 0.2)
#p.histogram_equalisation(0.9)
#p.random_color(0.9, 0.6, 1.6)
p.random_contrast(0.9, 0.6, 1.6)

# %%
#pimg  = Image.fromarray(img)
pimg = Image.open('/home/dmaturan/data/exceed_bot/2016-02-11-11-30-10/2016-02-11-11-30-10@025277.jpg')
#pimg

# %%
tf = p.torch_transform()
#tf = torchvision.transforms.Compose([p.torch_transform()

# %%
tf(pimg)

# %%
