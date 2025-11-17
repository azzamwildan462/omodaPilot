#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dataclasses import dataclass
from typing import Tuple, Optional, List
import numpy as np

class PX2M:
    # Abstract class to convert segmentation mask to laser scan 
    def __init__(self, logger, px2m_strategy):
        self.logger = logger
        self.px2m_strategy = px2m_strategy # 0=IPM, 1=Dari lidar
        self.has_init = True

        self.logger.info(f"PX2M initialized with px2m_strategy={px2m_strategy}")
    
    def __call__(self, mask: np.ndarray) -> Tuple[List[float], List[float]]:
        self.logger.info("PX2M called")
        distances = []
        angles = []
        return distances, angles
    
    def point_test_px2m(self, x: int, y: int) -> Tuple[float, float]:
        self.logger.info(f"PX2M point_test called with x={x}, y={y}")
        distance = 0.0
        angle = 0.0
        return distance, angle