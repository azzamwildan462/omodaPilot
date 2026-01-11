#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nydus_network — VFast‑SCNN backbone + GRU temporal head (network‑only)
=====================================================================
Pure PyTorch module: no ROS, no OpenCV, no Albumentations. Import this in your
own runtime (ROS node, CLI, etc.).

Exports
-------
- `NydusNetwork`: nn.Module implementing a compact road‑segmentation network
  with a VFast‑SCNN style encoder and a lightweight GRU operating on a global
  context vector.
- `make_nydus_network(...)`: convenience factory with sensible defaults.

Forward
-------
- `forward(x, h: Optional[Tensor] = None, return_state: bool = False)`
  * x: (B, 3, H, W)
  * h: (1, B, C5) — optional GRU hidden state
  * returns logits (B, num_classes, H, W) or (logits, h_new) if `return_state=True`

Notes
-----
- To keep temporal continuity across frames, pass the returned `h_new` into the
  next call.
- For conservative initialization you can set the final conv bias to a negative
  value externally (e.g., −2.884) before training/finetuning.
"""
from typing import Optional, Tuple
import torch
import torch.nn as nn
import torch.nn.functional as F

__all__ = ["NydusNetwork", "make_nydus_network"]

# ------------------------------------------------------------
# Blocks
# ------------------------------------------------------------
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
class NydusNetwork(nn.Module):
    def __init__(
            self,
            num_classes: int = 1,
            c1: int = 32,
            c2: int = 48,
            c3: int = 64,
            c4: int = 96,
            c5: int = 128,
            c6: int = 64,
            pools: Tuple[int, int, int, int] = (1, 2, 3, 6),
        ):
        super().__init__()

        # ====== Learning to Downsample ======
        self.downsample = nn.Sequential(
            ConvBNReLU(3, c1, 3, 2, 1),
            DepthwiseSeparableConv(c1, c2, 3, 2, 1),
            DepthwiseSeparableConv(c2, c3, 3, 2, 1),
        )

        self.gfe_dw_pw = nn.Sequential(
            LinearBottleneck(c3, c3, 3, 6),
            LinearBottleneck(c3, c4, 3, 6),
            LinearBottleneck(c4, c5, 3, 6),
        )

        self.ppm = PyramidPoolingModule(in_channels=c5, out_channels=c5)

        # ====== GRU temporal head ======
        # Input = c5, Hidden = c5 (ringan & konsisten)
        self.gru = nn.GRU(input_size=c5, hidden_size=c5, num_layers=1, batch_first=True)

        # ====== Classifier ======
        self.classifier = nn.Sequential(
            DepthwiseSeparableConv(c3 + c5, c6, 3, 1, 1),
            DepthwiseSeparableConv(c6, c6, 3, 1, 1),
            nn.Dropout(0.1),
            nn.Conv2d(c6, num_classes, 1),
        )

    @torch.jit.unused
    def reset_state(self):
        # Helper kalau mau simpan state internal (tidak dipakai default).
        self._h = None

    def forward(self, x, h: torch.Tensor = None, return_state: bool = False):
        """
        x: (B, 3, H, W)
        h: optional hidden state untuk GRU, shape (1, B, c5)
        return_state: kalau True, kembalikan (out, h_new)
        """
        size = x.size()[2:]

        # 1) Downsample
        x_down = self.downsample(x)

        # 2) Global features via Pyramid Pooling
        x_g = self.gfe_dw_pw(x_down)
        x_g = self.ppm(x_g)   # (B, c5, H', W')

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


# ------------------------------------------------------------
# Factory
# ------------------------------------------------------------

def make_nydus_network(
    num_classes: int = 1,
    c1: int = 32,
    c2: int = 48,
    c3: int = 64,
    c4: int = 96,
    c5: int = 128,
    c6: int = 64,
    pools=(1, 2, 3, 6),
) -> NydusNetwork:
    return NydusNetwork(num_classes=num_classes, c1=c1, c2=c2, c3=c3, c4=c4, c5=c5, c6=c6, pools=pools)


