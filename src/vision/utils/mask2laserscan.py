#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dataclasses import dataclass
from typing import Tuple, Optional, List
import numpy as np
from sensor_msgs.msg import LaserScan

from utils.px2m import PX2M

class Mask2LaserScan:
    # Abstract class to convert segmentation mask to laser scan 
    def __init__(self, logger, scan_strategy, px2m_strategy):
        self.logger = logger
        self.scan_strategy = scan_strategy # 0=scan dari tengah, 1=scan dari pinggir, 2dst=terserah
        self.px2m_strategy = px2m_strategy # 0=IPM, 1=Dari lidar

        self.px2m_converter = PX2M(logger, px2m_strategy)

        self.has_init = True

        self.logger.info(f"Mask2LaserScan initialized with scan_strategy={scan_strategy}, px2m_strategy={px2m_strategy}")
    
    def __call__(self, mask: np.ndarray, header: Optional[object] = None) -> LaserScan:
        self.logger.info("Mask2LaserScan called")

        dummy_px2m = self.px2m_converter(mask)

        laser_scan = LaserScan()
        return laser_scan