from __future__ import annotations

from collections import deque
from typing import Deque, Mapping, Optional


class InfraredAngleFilter:
    def __init__(
        self,
        *,
        channel_to_angle_deg: Mapping[int, float],
        history_size: int = 8,
        ewma_alpha: float = 0.35,
        clear_after_misses: int = 3,
    ) -> None:
        self._channel_to_angle_deg = dict(channel_to_angle_deg)
        self._samples: Deque[float] = deque(maxlen=max(1, int(history_size)))
        self._ewma_alpha = _clamp(float(ewma_alpha), 0.0, 1.0)
        self._clear_after_misses = max(1, int(clear_after_misses))
        self._smoothed_angle_deg: Optional[float] = None
        self._miss_count = 0

    def update(self, channel: int) -> Optional[float]:
        raw_angle = self._channel_to_angle_deg.get(channel)
        if raw_angle is None:
            self.mark_missed()
            return self._smoothed_angle_deg

        self._samples.append(float(raw_angle))
        averaged_angle = self._weighted_average_angle()
        if self._smoothed_angle_deg is None:
            self._smoothed_angle_deg = averaged_angle
        else:
            delta = _normalize_angle_deg(averaged_angle - self._smoothed_angle_deg)
            self._smoothed_angle_deg = _normalize_angle_deg(
                self._smoothed_angle_deg + self._ewma_alpha * delta
            )
        self._miss_count = 0
        return self._smoothed_angle_deg

    def mark_missed(self) -> None:
        self._miss_count += 1
        if self._miss_count >= self._clear_after_misses:
            self.clear()

    def clear(self) -> None:
        self._samples.clear()
        self._smoothed_angle_deg = None
        self._miss_count = 0

    def _weighted_average_angle(self) -> float:
        weighted_sum = 0.0
        total_weight = 0.0
        for index, angle in enumerate(self._samples, start=1):
            weight = float(index)
            weighted_sum += angle * weight
            total_weight += weight
        if total_weight <= 0.0:
            return 0.0
        return weighted_sum / total_weight


def _normalize_angle_deg(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))
