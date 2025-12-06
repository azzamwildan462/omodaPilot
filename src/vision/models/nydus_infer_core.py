#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import Optional, List, Tuple, Dict
import os
import time
import numpy as np
import cv2
import torch
import torch.nn as nn
from albumentations import Compose, Resize, Normalize
from albumentations.pytorch import ToTensorV2

# ------------------------------------------------------------
# Flexible state_dict loader
# ------------------------------------------------------------

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
    # map_location aman di CPU/GPU
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

# ------------------------------------------------------------
# Light & image helpers (kept minimal; no I/O)
# ------------------------------------------------------------

def gray_world_wb(img_rgb: np.ndarray) -> np.ndarray:
    img = img_rgb.astype(np.float32)
    mr, mg, mb = img[:,:,0].mean(), img[:,:,1].mean(), img[:,:,2].mean()
    m = (mr + mg + mb) / 3.0 + 1e-6
    gains = np.array([m/(mr+1e-6), m/(mg+1e-6), m/(mb+1e-6)], dtype=np.float32)
    img *= gains.reshape(1,1,3)
    return np.clip(img, 0, 255).astype(np.uint8)


def clahe_rgb(img_rgb: np.ndarray, clip=2.0, tile=8) -> np.ndarray:
    lab = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(tile, tile))
    l2 = clahe.apply(l)
    return cv2.cvtColor(cv2.merge([l2, a, b]), cv2.COLOR_LAB2RGB)


def adaptive_gamma(img_rgb: np.ndarray, target_mean=0.5, lo=0.5, hi=2.0):
    ycc = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2YCrCb)
    y = ycc[:, :, 0].astype(np.float32) / 255.0
    m = float(y.mean() + 1e-6)
    g = float(np.clip(np.log(target_mean)/np.log(m), lo, hi))
    out = np.clip((img_rgb.astype(np.float32)/255.0) ** g, 0, 1)
    return (out*255).astype(np.uint8), g


def overlay_mask(bgr: np.ndarray, mask: np.ndarray, alpha=0.35) -> np.ndarray:
    mb = mask.astype(bool)
    overlay = np.zeros_like(bgr)
    overlay[:, :, 1] = 255
    mixed = cv2.addWeighted(bgr, 1 - alpha, overlay, alpha, 0)
    out = bgr.copy()
    if mb.any():
        out[mb] = mixed[mb]
    return out


def _line_kernel(size: int, angle_deg: int) -> np.ndarray:
    size = max(3, int(size) | 1)
    k = np.zeros((size, size), np.uint8)
    c = size // 2
    ang = np.deg2rad(angle_deg)
    r = c
    x0 = int(round(c - r * np.cos(ang)))
    y0 = int(round(c - r * np.sin(ang)))
    x1 = int(round(c + r * np.cos(ang)))
    y1 = int(round(c + r * np.sin(ang)))
    cv2.line(k, (x0, y0), (x1, y1), 1, 1)
    return k


def cut_thin_connections(mask_bin: np.ndarray, max_bridge_px: int = 7, iters: int = 1, orientations=(0, 45, 90, 135)) -> np.ndarray:
    m = (mask_bin*255).astype(np.uint8) if mask_bin.max() <= 1 else mask_bin.astype(np.uint8)
    out = m.copy()
    for _ in range(max(1, iters)):
        for ang in orientations:
            out = cv2.morphologyEx(out, cv2.MORPH_OPEN, _line_kernel(max_bridge_px, ang))
    return (out > 127).astype(np.uint8)


def cut_thin_necks_disk(mask_bin: np.ndarray, radius_px: int = 3, iters: int = 1) -> np.ndarray:
    m = (mask_bin*255).astype(np.uint8) if mask_bin.max() <= 1 else mask_bin.astype(np.uint8)
    ksz = 2*radius_px + 1
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksz, ksz))
    out = cv2.morphologyEx(m, cv2.MORPH_OPEN, k, iterations=max(1, iters))
    return (out > 127).astype(np.uint8)

def largest_contour_filled(bin_mask, min_area=500, touch_bottom=True, ksize=5):
    """
    Pilih kontur terbesar (opsional: yang menyentuh bawah), isi penuh (holes tertutup).
    bin_mask: HxW uint8 {0,1/255} -> return {0,1}
    """
    m = bin_mask.astype(np.uint8)
    if m.max() == 1: m *= 255
    H, W = m.shape[:2]
    if ksize and ksize > 1:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
    contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return (m > 0).astype(np.uint8)

    def touches_bottom(cnt):
        x, y, w, h = cv2.boundingRect(cnt)
        return (y + h) >= (H - 2)

    cands = contours
    if touch_bottom:
        tb = [c for c in contours if touches_bottom(c)]
        if tb: cands = tb

    best, best_area = None, -1
    for c in cands:
        area = cv2.contourArea(c)
        if area >= min_area and area > best_area:
            best, best_area = c, area
    if best is None:
        for c in cands:
            area = cv2.contourArea(c)
            if area > best_area:
                best, best_area = c, area
        if best is None:
            return (m > 0).astype(np.uint8)

    filled = np.zeros((H, W), dtype=np.uint8)
    cv2.drawContours(filled, [best], -1, 255, thickness=cv2.FILLED)
    return (filled > 0).astype(np.uint8)


# ------------------------------------------------------------
# Nydus inference core (stateful, no I/O) — CUDA optimized
# ------------------------------------------------------------
class NydusInferCore:
    def __init__(self,
                 model: nn.Module,
                 device: str = "cpu",
                 weights: str = "",
                 in_w: int = 640, in_h: int = 360,
                 thr: float = 0.5,
                 use_wb: bool = False,
                 use_clahe: bool = False,
                 use_adaptive_gamma: bool = False,
                 tta_gamma: Optional[List[float]] = None,
                 touch_bottom: bool = True,
                 min_area: int = 500,
                 ksize: int = 5,
                 lp_alpha: float = 0.7,
                 crop_upper: int = 0, crop_lower: int = 0,
                 scene_cut: bool = True, scene_cut_thresh: float = 0.35,
                 use_amp: Optional[bool] = None):
        """Initialize the inference core.

        - device: "cuda" / "cuda:0" / "cpu"
        - use_amp: None -> True if CUDA available, else False
        """
        # ----- device & AMP -----
        requested_device = device
        if device.startswith("cuda") and not torch.cuda.is_available():
            print(f"[WARN] CUDA tidak tersedia, fallback ke CPU (request={requested_device})")
            device = "cpu"

        self.device = torch.device(device)

        if use_amp is None:
            self.use_amp = (self.device.type == "cuda")
        else:
            self.use_amp = bool(use_amp) and (self.device.type == "cuda")

        if self.device.type == "cuda":
            torch.backends.cudnn.benchmark = True
            print(f"[INFO] Using CUDA device: {self.device}, AMP={self.use_amp}")
        else:
            print(f"[INFO] Using CPU device: {self.device}, AMP={self.use_amp}")

        # ----- model -----
        self.model = model.to(self.device)
        self.model.eval()

        # conservative negative bias (optional; harmless if missing)
        last = None
        for m in self.model.modules():
            if isinstance(m, nn.Conv2d) and getattr(m, 'out_channels', None) == 1:
                last = m
        if last is not None and last.bias is not None:
            try:
                nn.init.constant_(last.bias, -2.884)
            except Exception:
                pass

        if weights and os.path.isfile(weights):
            load_weights_flex(self.model, weights, device=self.device, strict=False, verbose=True)
        elif weights:
            print(f"[WARN] weights tidak ditemukan: {weights}")

        self.preproc = Compose([
            Resize(in_h, in_w),
            Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225)),
            ToTensorV2(),
        ])

        self.thr = float(thr)
        self.use_wb = bool(use_wb)
        self.use_clahe = bool(use_clahe)
        self.use_adg = bool(use_adaptive_gamma)
        self.tta = list(tta_gamma) if tta_gamma else []
        self.touch_bottom = bool(touch_bottom)
        self.min_area = int(min_area)
        self.ksize = int(ksize)
        self.lp_alpha = float(lp_alpha)
        self.crop_upper = int(crop_upper)
        self.crop_lower = int(crop_lower)
        self.scene_cut = bool(scene_cut)
        self.scene_cut_thresh = float(scene_cut_thresh)

        # temporal states
        self.h_state: Optional[torch.Tensor] = None
        self.prev_gray: Optional[np.ndarray] = None
        self.prev_mask_f32: Optional[np.ndarray] = None

    # ---------- state utils ----------
    def reset_state(self):
        self.h_state = None
        self.prev_gray = None
        self.prev_mask_f32 = None

    # ---------- pre/post ----------
    def _maybe_scene_cut(self, bgr: np.ndarray):
        if not self.scene_cut:
            return
        g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if self.prev_gray is not None:
            h1 = cv2.calcHist([self.prev_gray],[0],None,[64],[0,256])
            h2 = cv2.calcHist([g],[0],None,[64],[0,256])
            cv2.normalize(h1,h1); cv2.normalize(h2,h2)
            diff = cv2.compareHist(h1,h2,cv2.HISTCMP_BHATTACHARYYA)
            if diff > self.scene_cut_thresh:
                self.reset_state()
        self.prev_gray = g

    def _pre_light(self, bgr: np.ndarray) -> np.ndarray:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        if self.use_wb:
            rgb = gray_world_wb(rgb)
        if self.use_clahe:
            rgb = clahe_rgb(rgb, clip=2.0, tile=8)
        if self.use_adg:
            rgb, _ = adaptive_gamma(rgb, target_mean=0.5)
        return rgb

    # ---------- core forward ----------
    @torch.no_grad()
    def forward_prob(self, bgr_or_rgb: np.ndarray, assume_bgr: bool = True) -> torch.Tensor:
        """Return probability map tensor (1,1,h,w) in [0,1] without post-processing."""
        rgb = cv2.cvtColor(bgr_or_rgb, cv2.COLOR_BGR2RGB) if assume_bgr else bgr_or_rgb
        x = self.preproc(image=rgb)["image"].unsqueeze(0).to(self.device, non_blocking=True)

        if self.use_amp:
            with torch.cuda.amp.autocast():
                logits, self.h_state = self.model(x, h=self.h_state, return_state=True)
        else:
            logits, self.h_state = self.model(x, h=self.h_state, return_state=True)

        prob = torch.sigmoid(logits)
        return prob

    def __call__(self, frame_bgr: np.ndarray) -> Tuple[np.ndarray, np.ndarray, Dict]:
        """Process a single BGR frame and return (mask_u8{0,1}, overlay_bgr, stats)."""
        assert frame_bgr.ndim == 3 and frame_bgr.shape[2] == 3, "Input must be HxWx3 BGR"
        H0, W0 = frame_bgr.shape[:2]
        t0 = time.time()

        # scene cut + lighting
        self._maybe_scene_cut(frame_bgr)
        frame_rgb = self._pre_light(frame_bgr)

        # TTA gamma (optional) + CUDA/AMP
        with torch.no_grad():
            if self.tta:
                probs = []
                if self.use_amp:
                    with torch.cuda.amp.autocast():
                        for g in self.tta:
                            g_img = np.clip((frame_rgb.astype(np.float32)/255.0) ** g, 0, 1)
                            g_img = (g_img*255).astype(np.uint8)
                            x = self.preproc(image=g_img)["image"].unsqueeze(0).to(self.device, non_blocking=True)
                            logits, self.h_state = self.model(x, h=self.h_state, return_state=True)
                            probs.append(torch.sigmoid(logits))
                else:
                    for g in self.tta:
                        g_img = np.clip((frame_rgb.astype(np.float32)/255.0) ** g, 0, 1)
                        g_img = (g_img*255).astype(np.uint8)
                        x = self.preproc(image=g_img)["image"].unsqueeze(0).to(self.device, non_blocking=True)
                        logits, self.h_state = self.model(x, h=self.h_state, return_state=True)
                        probs.append(torch.sigmoid(logits))

                prob = torch.mean(torch.stack(probs, dim=0), dim=0)
            else:
                x = self.preproc(image=frame_rgb)["image"].unsqueeze(0).to(self.device, non_blocking=True)
                if self.use_amp:
                    with torch.cuda.amp.autocast():
                        logits, self.h_state = self.model(x, h=self.h_state, return_state=True)
                else:
                    logits, self.h_state = self.model(x, h=self.h_state, return_state=True)
                prob = torch.sigmoid(logits)

        # binarize & resize (di CPU)
        mask_small = (prob.squeeze().detach().cpu().numpy() > self.thr).astype(np.uint8)
        mask = cv2.resize(mask_small, (W0, H0), interpolation=cv2.INTER_NEAREST)

        # cut bridges & necks
        mask = cut_thin_connections(mask, max_bridge_px=7, iters=1)
        mask = cut_thin_necks_disk(mask, radius_px=3, iters=1)

        # optional crop bands
        if self.crop_lower > 0:
            rect = np.ones_like(mask); rect[H0-self.crop_lower:H0,:] = 1; mask &= rect
        if self.crop_upper > 0:
            rect = np.ones_like(mask); rect[:self.crop_upper,:] = 0; mask &= rect

        # largest contour + fill
        mask = largest_contour_filled(mask, min_area=self.min_area,
                                      touch_bottom=self.touch_bottom, ksize=self.ksize)

        # temporal low-pass on mask
        if self.prev_mask_f32 is None:
            self.prev_mask_f32 = mask.astype(np.float32)
        else:
            mask = cv2.addWeighted(self.prev_mask_f32, self.lp_alpha,
                                   mask.astype(np.float32), 1-self.lp_alpha, 0)
            self.prev_mask_f32 = mask.copy()
        mask = (mask > 0.5).astype(np.uint8)

        overlay = overlay_mask(frame_bgr, mask, alpha=0.35)
        t_ms = (time.time()-t0)*1000.0
        return mask, overlay, {"t_ms": t_ms, "thr": self.thr, "device": str(self.device), "amp": self.use_amp}
