#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import Optional, List
import numpy as np

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

from utils.px2m import PX2M


class Mask2LaserScan:
    """
    Convert segmentation mask jalan -> LaserScan di frame base_link.

    Flow:
      - Pilih piksel representatif per kolom (scan dari bawah / edge).
      - Konversi pixel (u,v) -> (X_base, Y_base, Z_base) pakai PX2M.
      - Proyeksi ke bidang 2D (X,Y) untuk hitung range & angle.
      - Isi LaserScan.ranges dengan range minimal per beam.
    """

    def __init__(
        self,
        logger,
        scan_strategy: int,
        px2m_strategy: int,
        base_frame: str = "base_link",
        max_range: float = 50.0,
    ):
        """
        scan_strategy:
          0 = scan dari bawah (tiap kolom ambil pixel mask terbawah)
          1 = (reserved) scan dari pinggir
          2 = high-pass (edge) lalu scan dari bawah

        px2m_strategy:
          0 = depth 
          1 = IPM

        base_frame:
          Nama frame LaserScan (default: "base_link").
        """
        self.logger = logger
        self.scan_strategy = scan_strategy
        self.px2m_strategy = px2m_strategy
        self.base_frame = base_frame

        self.px2m_converter = PX2M(logger, px2m_strategy, base_frame=base_frame)

        # Parameter scan
        self.angle_min = -1.57      # -90 deg
        self.angle_max = 1.57       #  +90 deg
        self.angle_increment = 0.002 # ~0.57 deg
        self.range_min = 8.0
        self.range_max = max_range
        self.y_batas_bawah_scan_pixel = 240 # Pixel diatas ini akan diabaikan

        # Untuk mengurangi beban: skip beberapa kolom
        self.column_stride = 1

        self.has_init = True
        self.logger.info(
            f"Mask2LaserScan initialized with scan_strategy={scan_strategy}, "
            f"px2m_strategy={px2m_strategy}, base_frame={base_frame}"
        )

    # ------------------------------------------------------------------
    # High-pass / edge detection untuk mask
    # ------------------------------------------------------------------
    def _high_pass_edge(self, mask: np.ndarray) -> np.ndarray:
        """
        High-pass filter (morphological edge) sederhana (3x3).

        Input:  mask (H x W), 0 / non-0
        Output: edge_mask (H x W), dtype sama dengan mask
        """
        if mask.ndim != 2:
            raise ValueError("Mask2LaserScan._high_pass_edge: mask harus 2D")

        mask_bin = mask > 0
        if not np.any(mask_bin):
            return np.zeros_like(mask, dtype=mask.dtype)

        p = np.pad(mask_bin, 1, mode="constant", constant_values=False)
        eroded = (
            p[0:-2, 0:-2] & p[0:-2, 1:-1] & p[0:-2, 2:] &
            p[1:-1, 0:-2] & p[1:-1, 1:-1] & p[1:-1, 2:] &
            p[2:,   0:-2] & p[2:,   1:-1] & p[2:,   2:]
        )

        edge_bin = mask_bin & (~eroded)

        if np.issubdtype(mask.dtype, np.bool_):
            edge_mask = edge_bin.astype(mask.dtype)
        elif np.issubdtype(mask.dtype, np.integer):
            max_val = np.iinfo(mask.dtype).max
            edge_mask = (edge_bin.astype(mask.dtype) * max_val)
        else:
            edge_mask = edge_bin.astype(mask.dtype)
        
        return edge_mask

    # ------------------------------------------------------------------
    # Helper: pilih (u,v) per kolom (scan dari bawah)
    # ------------------------------------------------------------------
    def _select_pixels_per_column(self, mask: np.ndarray) -> List[tuple]:
        """
        Untuk setiap kolom u, pilih pixel v terbawah yang termasuk mask (>0).

        Return: list of (u,v) sepanjang width (dengan stride).
        """
        H, W = mask.shape
        uv_list: List[tuple] = []

        for u in range(0, W, self.column_stride):
            col = mask[:, u]
            ys = np.where(col > 0)[0]
            if ys.size == 0:
                continue
            v = ys.max()  # terbawah
            uv_list.append((u, v))

        return uv_list

    def mask_to_uv_list(self, mask: np.ndarray, mode: str = "bottom_per_column") -> List[tuple]:
        """
        Convert mask (0 / non-zero) -> list[(u,v)].

        mode:
          - "bottom_per_column" : untuk tiap kolom, ambil pixel terbawah (default)
          - "all_nonzero"       : ambil semua pixel non-zero (dengan column_stride)

        Contoh:
          uv_list = m2ls.mask_to_uv_list(mask_hp, mode="all_nonzero")
        """
        if mask.ndim != 2:
            raise ValueError("mask_to_uv_list: mask harus 2D (H x W)")

        if mode == "bottom_per_column":
            return self._select_pixels_per_column(mask)

        elif mode == "all_nonzero":
            H, W = mask.shape
            ys, xs = np.nonzero(mask)  # semua non-zero

            # optional: gunakan column_stride untuk kurangi density
            if self.column_stride > 1:
                # filter berdasarkan kolom
                keep = (xs % self.column_stride) == 0
                xs = xs[keep]
                ys = ys[keep]

            uv_list: List[tuple] = [(int(u), int(v)) for v, u in zip(ys, xs)]
            return uv_list

        else:
            raise ValueError(f"mask_to_uv_list: mode tidak dikenal: {mode!r}")

    # ------------------------------------------------------------------
    # Main call: mask -> LaserScan
    # ------------------------------------------------------------------
    def __call__(self, mask: np.ndarray, header: Optional[Header] = None) -> LaserScan:
        """
        Convert mask (jalan) -> LaserScan di base_frame (misal base_link).

        mask: H x W (0 / non-zero)
        header: opsional, kalau disediakan akan dipakai stamp-nya.
        """
        if mask.ndim != 2:
            raise ValueError("Mask2LaserScan.__call__: mask harus 2D (H x W)")

        # self.logger.info(
        #     f"Mask2LaserScan called, scan_strategy={self.scan_strategy}, "
        #     f"mask shape={mask.shape}"
        # )

        # 1) Pre-process sesuai strategi
        if self.scan_strategy == 2:
            proc_mask = self._high_pass_edge(mask)
        else:
            proc_mask = mask

        # 2) Pilih pixel per kolom
        # uv_list = self._select_pixels_per_column(proc_mask)
        uv_list = self.mask_to_uv_list(proc_mask, mode="all_nonzero")
        # self.logger.debug(
        #     f"Mask2LaserScan: selected {len(uv_list)} pixels from mask"
        # )

        # 3) Siapkan LaserScan
        scan = LaserScan()
        if header is not None:
            scan.header = header
        scan.header.frame_id = self.base_frame  # pastikan base_link

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        num_beams = int(
            (scan.angle_max - scan.angle_min) / scan.angle_increment
        ) + 1

        ranges = [(scan.range_max - 0.1)] * num_beams
        intensities = [0.0] * num_beams

        # 4) Proyeksikan setiap pixel ke base_link dan isi ranges
        x_offset = 6.0
        y_offset = 0.0
        for (u, v) in uv_list:
            
            if v > self.y_batas_bawah_scan_pixel:
                continue
            
            if self.px2m_converter.depth_frame is None:
                break

            pt_base = self.px2m_converter.pixel_to_base(u, v)
            if pt_base is None:
                continue

            Xb, Yb, Zb = pt_base

            # Kita pakai plane X-Y (REP 105: X forward, Y left, Z up)
            r = float(np.hypot(Xb, Yb))
            if r < self.range_min or r > self.range_max:
                continue

            angle = float(np.arctan2(Yb, Xb))  # radian

            # Cek apakah sudut masuk ke FOV LaserScan
            if angle < scan.angle_min or angle > scan.angle_max:
                continue
            
            # self.logger.info(f"Mask2LaserScan: pixel (u={u},v={v}) -> "
            #                   f"base (X={Xb:.2f},Y={Yb:.2f},Z={Zb:.2f}) -> "
            #                   f"(r={r:.2f}, angle={angle:.2f} rad)")

            idx = int((angle - scan.angle_min) / scan.angle_increment)
            if 0 <= idx < num_beams:
                # Simpan range minimum (obstacle terdekat)
                if r < ranges[idx]:
                    # self.logger.debug(f"Masuk {angle:.2f} dan {r:.2f} rad ke beam idx={idx}, ")
                    ranges[idx] = r
                    intensities[idx] = 1.0  # atau isi lain (misal confidence)
        
        scan.ranges = ranges
        scan.intensities = intensities

        # self.logger.info(
        #     f"Mask2LaserScan: num_beams={num_beams}, "
        #     f"valid_ranges={sum(np.isfinite(ranges))}"
        # )

        return scan
