"""Pure NumPy Edge AI Deep MLP Classifier with zero external dependencies."""

import numpy as np


class EdgeAIEnvironmentModel:
    """Lightweight 3-layer neural network running local microsecond inference."""

    CLASSES = [
        "OPEN_SKY_NORMAL",
        "URBAN_CANYON_WEAK",
        "TUNNEL_OUTAGE",
        "ANOMALOUS_DEVIATION"
    ]

    def __init__(self):
        # Layer 1: (8, 16)
        self.W1 = np.zeros((8, 16), dtype=np.float32)
        self.b1 = np.zeros((16,), dtype=np.float32)

        # Calibrate Layer 1 sensitivity
        # Features: [accel_var, gyro_var, hdop_mean, hdop_rate, sat_norm, valid_ratio, speed_disc, is_outage]

        # Nodes 0-3: Outage sensitive (dominant positive weight on is_outage, negative on valid_ratio)
        self.W1[7, 0:4] = 12.0
        self.W1[5, 0:4] = -8.0
        self.b1[0:4] = 2.0

        # Nodes 4-7: Urban canyon sensitive (requires valid GNSS packets, inhibited on outage)
        self.W1[2, 4:8] = 2.0
        self.W1[3, 4:8] = 1.0
        self.W1[4, 4:8] = -1.5
        self.W1[5, 4:8] = 3.0
        self.W1[7, 4:8] = -40.0
        self.b1[4:8] = -2.0

        # Nodes 8-11: Open sky sensitive (positive on sat_norm, valid_ratio, negative on hdop, inhibited on outage)
        self.W1[4, 8:12] = 4.0
        self.W1[5, 8:12] = 4.0
        self.W1[2, 8:12] = -2.5
        self.W1[7, 8:12] = -40.0
        self.b1[8:12] = 0.5

        # Nodes 12-15: Anomaly sensitive (positive on speed_disc, negative on is_outage)
        self.W1[6, 12:16] = 4.0
        self.W1[5, 12:16] = 2.0
        self.W1[7, 12:16] = -40.0
        self.b1[12:16] = -2.0

        # Layer 2: (16, 8)
        self.W2 = np.zeros((16, 8), dtype=np.float32)
        self.b2 = np.zeros((8,), dtype=np.float32)
        self.W2[0:4, 0:2] = 2.5   # Outage branch
        self.W2[4:8, 2:4] = 2.0   # Urban Canyon branch
        self.W2[8:12, 4:6] = 2.0  # Open sky branch
        self.W2[12:16, 6:8] = 2.0 # Anomaly branch

        # Layer 3: (8, 4)
        self.W3 = np.zeros((8, 4), dtype=np.float32)
        self.b3 = np.array([0.2, 0.0, 0.0, -0.5], dtype=np.float32)
        self.W3[4:6, 0] = 3.0 # Class 0: OPEN_SKY_NORMAL
        self.W3[2:4, 1] = 3.0 # Class 1: URBAN_CANYON_WEAK
        self.W3[0:2, 2] = 4.5 # Class 2: TUNNEL_OUTAGE
        self.W3[6:8, 3] = 3.0 # Class 3: ANOMALOUS_DEVIATION

    @staticmethod
    def relu(x: np.ndarray) -> np.ndarray:
        return np.maximum(0.0, x)

    @staticmethod
    def softmax(x: np.ndarray) -> np.ndarray:
        exp_x = np.exp(x - np.max(x))
        return exp_x / np.sum(exp_x)

    def forward(self, features: np.ndarray) -> np.ndarray:
        """Run forward pass and return 4 class probabilities."""
        h1 = self.relu(np.dot(features, self.W1) + self.b1)
        h2 = self.relu(np.dot(h1, self.W2) + self.b2)
        logits = np.dot(h2, self.W3) + self.b3
        probs = self.softmax(logits)
        return probs
