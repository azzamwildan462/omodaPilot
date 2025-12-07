#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import Optional, Tuple
import numpy as np

from sensor_msgs.msg import CameraInfo


class PX2M:
    """
    Pixel-to-Meter converter:
    - Ambil depth + intrinsics kamera
    - Unproject pixel (u,v) -> 3D (X_cam, Y_cam, Z_cam)
    - Transform ke frame base_link (X_base, Y_base, Z_base)
    """

    def __init__(self, logger, px2m_strategy: int, base_frame: str = "base_link"):
        """
        px2m_strategy:
          0 = IPM / pure depth (kamera -> base_link)
          1 = Dari lidar (BELUM diimplementasi, sementara pakai depth kamera dulu)

        base_frame:
          Nama frame target (misal "base_link").
        """
        self.logger = logger
        self.px2m_strategy = px2m_strategy
        self.base_frame = base_frame

        # Camera intrinsics
        self.camera_info: Optional[CameraInfo] = None
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None

        # Depth frame (H x W, meter)
        self.depth_frame: Optional[np.ndarray] = None

        # Extrinsic transform: T_cam2base (4x4)
        # Default: identity (kamera == base_link)
        self.T_cam2base: np.ndarray = np.eye(4, dtype=float)

        self.has_init = True
        self.logger.info(
            f"PX2M initialized with px2m_strategy={px2m_strategy}, "
            f"base_frame={base_frame}"
        )

    # ------------------------------------------------------------------
    # Setters
    # ------------------------------------------------------------------
    def set_camera_info(self, camera_info: CameraInfo) -> None:
        """
        Simpan CameraInfo dan ambil intrinsics (fx, fy, cx, cy).
        """
        self.camera_info = camera_info

        K = camera_info.k  # [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = float(K[0])
        self.fy = float(K[4])
        self.cx = float(K[2])
        self.cy = float(K[5])

        self.logger.info(
            f"PX2M: CameraInfo set: fx={self.fx:.3f}, fy={self.fy:.3f}, "
            f"cx={self.cx:.3f}, cy={self.cy:.3f}, "
            f"cam_frame={camera_info.header.frame_id}"
        )

    def set_depth_frame(self, depth: np.ndarray) -> None:
        """
        Simpan depth frame (H x W), dalam meter (atau unit konsisten).
        """
        if depth.ndim != 2:
            raise ValueError("PX2M: depth frame harus 2D (H x W)")

        self.depth_frame = depth
        # self.logger.debug(
        #     f"PX2M: depth frame set: shape={depth.shape}, dtype={depth.dtype}"
        # )

    def set_extrinsic_cam_to_base(self, T_cam2base: np.ndarray) -> None:
        """
        Set transform 4x4 dari frame kamera ke frame base_link.

        T_cam2base:
          np.ndarray (4,4), homogeneous transform:
            [R t]
            [0 1]
        """
        T_cam2base = np.asarray(T_cam2base, dtype=float)
        if T_cam2base.shape != (4, 4):
            raise ValueError("PX2M: T_cam2base harus 4x4")

        self.T_cam2base = T_cam2base
        self.logger.info(
            "PX2M: extrinsic T_cam2base updated:\n"
            + str(self.T_cam2base)
        )

    # ------------------------------------------------------------------
    # Core: pixel (u,v) -> 3D di base_link
    # ------------------------------------------------------------------
    def pixel_to_base(
        self, u: int, v: int
    ) -> Optional[Tuple[float, float, float]]:
        """
        Konversi satu pixel (u,v) menjadi titik 3D di frame base_link.

        Return: (X_base, Y_base, Z_base) atau None kalau depth tidak valid
        """
        if self.camera_info is None or self.fx is None:
            self.logger.error("PX2M.pixel_to_base: CameraInfo belum diset")
            return None

        if self.depth_frame is None:
            self.logger.error("PX2M.pixel_to_base: depth frame belum diset")
            return None

        H, W = self.depth_frame.shape
        if not (0 <= u < W and 0 <= v < H):
            self.logger.debug(
                f"PX2M.pixel_to_base: pixel ({u},{v}) di luar size depth (W={W},H={H})"
            )
            return None

        # Ambil depth (meter)
        Z_cam = float(self.depth_frame[v, u])
        if Z_cam <= 0 or np.isinf(Z_cam) or np.isnan(Z_cam):
            # Depth invalid
            return None

        # Unproject ke kamera
        uu = float(u)
        vv = float(v)

        X_cam = (uu - self.cx) / self.fx * Z_cam
        Y_cam = (vv - self.cy) / self.fy * Z_cam

        # Homogeneous
        p_cam = np.array([X_cam, Y_cam, Z_cam, 1.0], dtype=float)
        p_base = self.T_cam2base @ p_cam

        X_base = float(p_base[0])
        Y_base = float(p_base[1])
        Z_base = float(p_base[2])

        return X_base, Y_base, Z_base

    # Convenience (kalau mau test 1 titik dengan log)
    def point_test_px2m(
        self, u: int, v: int
    ) -> Optional[Tuple[float, float, float]]:
        pt = self.pixel_to_base(u, v)
        if pt is None:
            self.logger.info(
                f"PX2M.point_test_px2m: pixel ({u},{v}) -> NO POINT (invalid depth)"
            )
        else:
            Xb, Yb, Zb = pt
            self.logger.info(
                f"PX2M.point_test_px2m: pixel ({u},{v}) -> "
                f"base_link (X={Xb:.3f}, Y={Yb:.3f}, Z={Zb:.3f})"
            )
        return pt
