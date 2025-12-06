#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import argparse
import numpy as np
import cv2
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset
import albumentations as A
from albumentations.pytorch import ToTensorV2

import os
import cv2
import time
import math
import argparse
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import albumentations as A
from albumentations.pytorch import ToTensorV2


# =========================
# IMPORT / COPY dari file kamu:
# - gray_world_wb
# - clahe_rgb
# - VFastSCNN_GRU
# - load_weights_flex
# =========================

layer1 = 32
layer2 = 48
layer3 = 64
layer4 = 96
layer5 = 128  # juga dim GRU
layer6 = 64

DEFAULT_W = 640
DEFAULT_H = 360

def _strip_prefix_if_present(state_dict, prefixes=("module.", "ema.", "model_ema.", "model.", "net.")):
    out = {}
    for k, v in state_dict.items():
        nk = k
        for p in prefixes:
            if nk.startswith(p):
                nk = nk[len(p):]
        out[nk] = v
    return out

def _pick_state_dict(ckpt):
    # ckpt bisa: dict tensor langsung, atau dict berisi subkeys.
    if isinstance(ckpt, dict):
        # kandidat umum
        for key in ("model_ema", "ema", "state_dict", "model", "net", "weights"):
            if key in ckpt and isinstance(ckpt[key], dict):
                return ckpt[key]
        # kalau dict ini sendiri sudah tensor dict
        if all(isinstance(v, torch.Tensor) for v in ckpt.values()):
            return ckpt
    raise ValueError("Tidak menemukan state_dict yang valid di checkpoint.")

def load_weights_flex(model, path, device="cpu", strict=False, verbose=True):
    # map_location aman di CPU
    ckpt = torch.load(path, map_location=device)
    sd = _pick_state_dict(ckpt)
    sd = _strip_prefix_if_present(sd)
    # Optional: pastikan dtype & device cocok
    sd = {k: (v.to(device) if isinstance(v, torch.Tensor) else v) for k, v in sd.items()}

    missing, unexpected = model.load_state_dict(sd, strict=strict)
    if verbose:
        print(f"[LOAD] strict={strict}  missing={len(missing)}  unexpected={len(unexpected)}")
        if missing:   print("  missing keys (truncated):", missing[:8], "...")
        if unexpected:print("  unexpected keys (truncated):", unexpected[:8], "...")
    return model


# =========================
# Util: Lighting Robustness
# =========================
def gray_world_wb(img_rgb: np.ndarray) -> np.ndarray:
    # img_rgb uint8 RGB
    img = img_rgb.astype(np.float32)
    mean_r = img[:, :, 0].mean()
    mean_g = img[:, :, 1].mean()
    mean_b = img[:, :, 2].mean()
    mean_gray = (mean_r + mean_g + mean_b) / 3.0 + 1e-6
    gains = np.array([mean_gray/(mean_r+1e-6),
                      mean_gray/(mean_g+1e-6),
                      mean_gray/(mean_b+1e-6)], dtype=np.float32)
    img *= gains.reshape(1,1,3)
    return np.clip(img, 0, 255).astype(np.uint8)

def clahe_rgb(img_rgb: np.ndarray, clip=2.0, tile=8) -> np.ndarray:
    lab = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(tile, tile))
    l2 = clahe.apply(l)
    lab2 = cv2.merge([l2, a, b])
    return cv2.cvtColor(lab2, cv2.COLOR_LAB2RGB)

def adaptive_gamma(img_rgb: np.ndarray, target_mean=0.5, lo=0.5, hi=2.0):
    # Set gamma agar mean luminance ~ target_mean
    ycc = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2YCrCb)
    y = ycc[:, :, 0].astype(np.float32) / 255.0
    m = float(y.mean() + 1e-6)
    gamma = np.log(target_mean)/(np.log(m))
    gamma = float(np.clip(gamma, lo, hi))
    out = np.clip((img_rgb.astype(np.float32)/255.0) ** gamma, 0, 1)
    return (out*255).astype(np.uint8), gamma

# =========================
# Albumentations (resize+norm)
# =========================
def get_preproc(img_h, img_w):
    return A.Compose([
        A.Resize(img_h, img_w),
        A.Normalize(mean=(0.485, 0.456, 0.406),
                    std=(0.229, 0.224, 0.225)),
        ToTensorV2(),
    ])

# ===========================================================
# VFast-SCNN building blocks
# ===========================================================
class ConvBNReLU(nn.Module):
    def __init__(self, in_channels, out_channels, kernel_size, stride, padding):
        super().__init__()
        self.block = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, kernel_size, stride, padding, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True)
        )

    def forward(self, x):
        return self.block(x)


class DepthwiseSeparableConv(nn.Module):
    def __init__(self, in_channels, out_channels, kernel_size, stride, padding):
        super().__init__()
        self.depthwise = nn.Sequential(
            nn.Conv2d(in_channels, in_channels, kernel_size=kernel_size, stride=stride,
                      padding=padding, groups=in_channels, bias=False),
            nn.BatchNorm2d(in_channels),
            nn.ReLU(inplace=True)
        )
        self.pointwise = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, kernel_size=1, stride=1, padding=0, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True)
        )

    def forward(self, x):
        return self.pointwise(self.depthwise(x))

class DepthwiseConv(nn.Module):
    def __init__(self, dw_channels, out_channels, stride=1, **kwargs):
        super(DepthwiseConv, self).__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(dw_channels, out_channels, 3, stride, 1, groups=dw_channels, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(True)
        )

    def forward(self, x):
        return self.conv(x)

class PyramidPoolingModule(nn.Module):
    def __init__(self, in_channels, out_channels, pool_sizes=(1, 2, 3, 6)):
        super().__init__()
        self.pool_sizes = pool_sizes
        inter_channels = in_channels // len(pool_sizes)

        self.stages = nn.ModuleList([
            nn.Sequential(
                nn.AdaptiveAvgPool2d(ps),
                nn.Conv2d(in_channels, inter_channels, 1, bias=False),
                nn.BatchNorm2d(inter_channels),
                nn.ReLU(inplace=True)
            ) for ps in pool_sizes
        ])

        # final 1×1 to fuse concatenated channels
        self.bottleneck = nn.Sequential(
            nn.Conv2d(in_channels + inter_channels * len(pool_sizes), out_channels, 1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True)
        )

    def forward(self, x):
        h, w = x.size(2), x.size(3)
        pooled_outputs = [x]
        for stage in self.stages:
            pooled = stage(x)
            upsampled = F.interpolate(pooled, size=(h, w), mode='bilinear', align_corners=True)
            pooled_outputs.append(upsampled)
        out = torch.cat(pooled_outputs, dim=1)
        return self.bottleneck(out)

class LinearBottleneck(nn.Module):
    """LinearBottleneck used in MobileNetV2"""

    def __init__(self, in_channels, out_channels, t=6, stride=2, **kwargs):
        super(LinearBottleneck, self).__init__()
        self.use_shortcut = stride == 1 and in_channels == out_channels
        self.block = nn.Sequential(
            # pw
            ConvBNReLU(in_channels, in_channels * t, 1, 1, 1),
            # dw
            DepthwiseConv(in_channels * t, in_channels * t, stride),
            # pw-linear
            nn.Conv2d(in_channels * t, out_channels, 1, bias=False),
            nn.BatchNorm2d(out_channels)
        )

    def forward(self, x):
        out = self.block(x)
        if self.use_shortcut:
            out = x + out
        return out

# ===========================================================
# VFast-SCNN + GRU (temporal head)
# ===========================================================
class VFastSCNN_GRU(nn.Module):
    def __init__(self, num_classes=1):
        super().__init__()

        # ====== Learning to Downsample ======
        self.downsample = nn.Sequential(
            ConvBNReLU(3, layer1, 3, 2, 1),
            DepthwiseSeparableConv(layer1, layer2, 3, 2, 1),
            DepthwiseSeparableConv(layer2, layer3, 3, 2, 1),
        )

        # ====== Global Feature Extractor (depthwise 9x9 -> pw 1x1 -> GAP -> 1x1) ======
        # self.gfe_dw_pw = nn.Sequential(
        #     nn.Conv2d(layer3, layer3, 3, padding=1, groups=layer3, bias=False),
        #     nn.BatchNorm2d(layer3),
        #     nn.ReLU(inplace=True),
        #     nn.Conv2d(layer3, layer4, 1, bias=False),
        #     nn.BatchNorm2d(layer4),
        #     nn.ReLU(inplace=True),
        # )

        self.gfe_dw_pw = nn.Sequential(
            LinearBottleneck(layer3, layer3, 3, 6),
            LinearBottleneck(layer3, layer4, 3, 6),
            LinearBottleneck(layer4, layer5, 3, 6),
        )

        self.ppm = PyramidPoolingModule(in_channels=layer5, out_channels=layer5)

        # ====== GRU temporal head ======
        # Input = layer5, Hidden = layer5 (ringan & konsisten)
        self.gru = nn.GRU(input_size=layer5, hidden_size=layer5, num_layers=1, batch_first=True)

        # ====== Classifier ======
        self.classifier = nn.Sequential(
            DepthwiseSeparableConv(layer3 + layer5, layer6, 3, 1, 1),
            DepthwiseSeparableConv(layer6, layer6, 3, 1, 1),
            nn.Dropout(0.1),
            nn.Conv2d(layer6, num_classes, 1),
        )

    @torch.jit.unused
    def reset_state(self):
        # Helper kalau mau simpan state internal (tidak dipakai default).
        self._h = None

    def forward(self, x, h: torch.Tensor = None, return_state: bool = False):
        """
        x: (B, 3, H, W)
        h: optional hidden state untuk GRU, shape (1, B, layer5)
        return_state: kalau True, kembalikan (out, h_new)
        """
        size = x.size()[2:]

        # 1) Downsample
        x_down = self.downsample(x)

        # 2) Global features via Pyramid Pooling
        x_g = self.gfe_dw_pw(x_down)
        x_g = self.ppm(x_g)   # (B, layer5, H', W')

        # 3) GRU over global (mean) vector
        B, C, H_, W_ = x_g.shape
        x_vec = x_g.mean(dim=[2, 3])  # global vector (B, C)
        seq = x_vec.unsqueeze(1)      # (B, T=1, C)
        out_seq, h_new = self.gru(seq, h)

        # 4) Expand GRU output to spatial size and fuse
        x_global_up = out_seq.view(B, C, 1, 1)
        x_global_up = F.interpolate(x_global_up, size=x_down.size()[2:], mode='bilinear', align_corners=True)
        x_cat = torch.cat([x_down, x_global_up], dim=1)

        # 5) Classifier + upsample to input size
        out = self.classifier(x_cat)
        out = F.interpolate(out, size=size, mode='bilinear', align_corners=True)

        if return_state:
            return out, h_new
        return out

# ------------ Dataset ------------
class DatasetLoader(Dataset):
    def __init__(self, image_dir, mask_dir, transform=None):
        self.image_dir = image_dir
        self.mask_dir = mask_dir
        self.images = sorted(os.listdir(image_dir))
        self.transform = transform

    def __len__(self):
        return len(self.images)

    def __getitem__(self, idx):
        img_path = os.path.join(self.image_dir, self.images[idx])
        mask_path = os.path.join(self.mask_dir, self.images[idx])

        image = cv2.imread(img_path)
        if image is None:
            raise FileNotFoundError(f"Gambar tidak ditemukan: {img_path}")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # sama seperti inference video
        image = gray_world_wb(image)
        image = clahe_rgb(image, clip=2.0, tile=8)

        mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
        if mask is None:
            raise FileNotFoundError(f"Mask tidak ditemukan: {mask_path}")

        if self.transform:
            augmented = self.transform(image=image, mask=mask)
            image = augmented['image']   # Tensor (C,H,W), float
            mask = augmented['mask']     # Tensor (H,W), float (0..1)

        # pastikan biner
        if isinstance(mask, torch.Tensor):
            mask = (mask > 0.5).float()
        else:
            mask = torch.from_numpy((mask > 127).astype(np.float32))

        return image, mask


def get_val_transform(img_h, img_w):
    # sama konsepnya dengan get_preproc di script video:
    return A.Compose([
        A.Resize(img_h, img_w),
        A.Normalize(mean=(0.485, 0.456, 0.406),
                    std=(0.229, 0.224, 0.225)),
        ToTensorV2(),
    ])


# ------------ Metric helper ------------
def update_iou_stats(pred_bin, gt_bin, inter, union):
    """
    pred_bin, gt_bin: Tensor [1,H,W] dengan nilai {0,1}
    Kita ambil IoU utk foreground (kelas 1) saja.
    """
    pred_np = pred_bin.view(-1).cpu().numpy().astype(np.uint8)
    gt_np   = gt_bin.view(-1).cpu().numpy().astype(np.uint8)

    # foreground = 1
    pred_fg = pred_np == 1
    gt_fg   = gt_np == 1

    inter[0] += np.logical_and(pred_fg, gt_fg).sum()
    union[0] += np.logical_or(pred_fg, gt_fg).sum()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", type=str, required=True, help="Path .pth (state_dict)")
    ap.add_argument("--image_dir", type=str, required=True, help="Folder gambar (frame)")
    ap.add_argument("--mask_dir", type=str, required=True, help="Folder mask GT")
    ap.add_argument("--h", type=int, default=360, help="Height inference (sesuai training)")
    ap.add_argument("--w", type=int, default=640, help="Width inference (sesuai training)")
    ap.add_argument("--thr", type=float, default=0.5, help="Threshold sigmoid -> biner")
    ap.add_argument("--cuda", action="store_true", help="Pakai CUDA kalau tersedia")
    ap.add_argument("--use_gru_state", action="store_true",
                    help="Kalau True, GRU state di-pass antar frame (urutan file diasumsikan urut video).")
    args = ap.parse_args()

    device = torch.device("cuda" if (args.cuda and torch.cuda.is_available()) else "cpu")
    print("Device:", device)

    # --- Model ---
    model = VFastSCNN_GRU(num_classes=1).to(device)

    # Bias last conv ke negatif (sama seperti scriptmu)
    last = None
    for m in model.classifier.modules():
        if isinstance(m, nn.Conv2d) and m.out_channels == 1:
            last = m
    if last is not None and last.bias is not None:
        nn.init.constant_(last.bias, -2.884)

    model = load_weights_flex(model, args.weights, device=device, strict=False, verbose=True)
    model.eval()
    print(f"Loaded weights: {args.weights}")

    # --- Dataset + Loader ---
    transform = get_val_transform(args.h, args.w)
    dataset = DatasetLoader(args.image_dir, args.mask_dir, transform=transform)

    # untuk temporal GRU, batch_size=1 & shuffle=False
    loader = DataLoader(dataset, batch_size=1, shuffle=False, num_workers=4)

    # --- Metric accumulator ---
    inter = np.zeros(1, dtype=np.float64)   # foreground
    union = np.zeros(1, dtype=np.float64)
    total_correct = 0
    total_pixels  = 0

    h_state = None

    print(f"Total frame: {len(dataset)}")
    with torch.no_grad():
        for idx, (img, gt) in enumerate(loader):
            img = img.to(device)          # [1,3,H,W]
            gt  = gt.to(device)           # [1,H,W]

            if args.use_gru_state:
                logits, h_state = model(img, h=h_state, return_state=True)
            else:
                logits = model(img, return_state=False)
                h_state = None

            prob = torch.sigmoid(logits)  # [1,1,H,W]
            pred_bin = (prob > args.thr).float().squeeze(1)  # [1,H,W]

            # pixel accuracy
            total_correct += (pred_bin == gt).sum().item()
            total_pixels  += gt.numel()

            # IoU stats
            update_iou_stats(pred_bin, gt, inter, union)

    # --- Final metrics ---
    eps = 1e-7
    iou_fg = inter[0] / (union[0] + eps)
    miou   = iou_fg  # cuma 1 kelas foreground, jadi mIoU = IoU_fg
    pixel_acc = total_correct / (total_pixels + eps)

    # Dice dari IoU (opsional)
    dice_fg = (2 * iou_fg) / (1 + iou_fg + eps)

    print("==== HASIL EVALUASI ====")
    print(f"IoU foreground       : {iou_fg:.4f}")
    print(f"mIoU                 : {miou:.4f}")
    print(f"Dice foreground      : {dice_fg:.4f}")
    print(f"Pixel accuracy (all) : {pixel_acc:.4f}")


if __name__ == "__main__":
    main()
