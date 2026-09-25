"""Edge AI Navigation Classifier Engine."""

import time
from typing import Tuple, Dict
import numpy as np
from ..sensor_layer.data_types import SensorFrame
from .feature_extractor import NavigationFeatureExtractor
from .model import EdgeAIEnvironmentModel


class EdgeAIClassifier:
    """Classifies real-time navigation operating condition and provides adaptive filter hints."""

    def __init__(self, window_size: int = 15):
        self.extractor = NavigationFeatureExtractor(window_size=window_size)
        self.model = EdgeAIEnvironmentModel()
        self.last_condition = "OPEN_SKY_NORMAL"
        self.last_confidence = 1.0
        self.last_latency_ms = 0.0

    def classify(self, frame: SensorFrame, est_speed: float = 0.0) -> Tuple[str, float, float, Dict[str, float]]:
        """Run classification pipeline on incoming frame.
        
        Returns:
            (predicted_class, confidence, adaptive_scale, probabilities_dict)
        """
        t_start = time.perf_counter()

        # 1. Feature Extraction
        features = self.extractor.push_and_extract(frame, est_speed)

        # 2. Local Neural Inference
        probs = self.model.forward(features)
        
        t_end = time.perf_counter()
        self.last_latency_ms = (t_end - t_start) * 1000.0

        # 3. Class decision
        class_idx = int(np.argmax(probs))
        predicted_class = self.model.CLASSES[class_idx]
        confidence = float(probs[class_idx])

        # 4. Adaptive EKF scaling recommendation based on predicted environment
        if predicted_class == "OPEN_SKY_NORMAL":
            adaptive_scale = 1.0
        elif predicted_class == "URBAN_CANYON_WEAK":
            adaptive_scale = 4.0  # Inflate GNSS measurement noise due to multipath
        elif predicted_class == "TUNNEL_OUTAGE":
            adaptive_scale = 100.0 # GNSS effectively ignored
        else:
            adaptive_scale = 10.0 # Anomaly / Kinematic divergence

        prob_dict = {
            self.model.CLASSES[i]: round(float(probs[i]), 4)
            for i in range(len(self.model.CLASSES))
        }

        self.last_condition = predicted_class
        self.last_confidence = confidence

        return predicted_class, confidence, adaptive_scale, prob_dict
