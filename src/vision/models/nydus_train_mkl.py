#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import time
import numpy as np
from tqdm import tqdm

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader

import albumentations as A
from albumentations.pytorch import ToTensorV2

import multiprocessing as mp
os.environ.setdefault("OMP_NUM_THREADS", "1")        # hindari nested threads di worker
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import torch.backends.mkldnn as mkldnn
mkldnn.enabled = True   # aktifkan oneDNN

def configure_cpu(num_cores: int = 30):
    # PyTorch threading utk operator (intra) & koordinasi (inter)
    torch.set_num_threads(max(1, num_cores - 2))     # misal 28
    torch.set_num_interop_threads(2)
    try:
        import cv2
        cv2.setNumThreads(0)  # cegah OpenCV ngambil thread banyak
    except Exception:
        pass

configure_cpu(28)

# zzzzzz malassss