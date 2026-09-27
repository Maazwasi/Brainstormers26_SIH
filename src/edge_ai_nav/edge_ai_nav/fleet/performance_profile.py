"""Measured per-AMR mission performance with deliberately slow bounded tuning."""
from dataclasses import dataclass, asdict, field
import json
import os


BOUNDS = {
    'speed_factor': (0.95, 1.05),
    'steering_factor': (0.95, 1.05),
    'lookahead_factor': (0.95, 1.05),
}


def _bounded(name, value):
    low, high = BOUNDS[name]
    return max(low, min(high, float(value)))


@dataclass
class PerformanceProfile:
    robot_id: str
    missions_completed: int = 0
    missions_failed: int = 0
    average_route_efficiency: float = 0.0
    average_task_time: float = 0.0
    average_heading_error: float = 0.0
    average_negotiation_delay: float = 0.0
    unnecessary_stop_count: int = 0
    reroute_overhead_m: float = 0.0
    speed_factor: float = 1.0
    steering_factor: float = 1.0
    lookahead_factor: float = 1.0
    last_mission: dict = field(default_factory=dict)

    @classmethod
    def load(cls, robot_id, path):
        try:
            with open(path, encoding='utf-8') as stream:
                raw = json.load(stream)
            if raw.get('robot_id') != robot_id:
                raise ValueError('profile belongs to another AMR')
            allowed = cls.__dataclass_fields__
            profile = cls(**{k: v for k, v in raw.items() if k in allowed})
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            profile = cls(robot_id=robot_id)
        for name in BOUNDS:
            setattr(profile, name, _bounded(name, getattr(profile, name)))
        return profile

    @staticmethod
    def _mean(previous, count, value):
        return float(value) if count == 1 else previous + (float(value)-previous)/count

    def record(self, metrics):
        """Update only after a completed mission, never from controller ticks."""
        self.missions_completed += 1
        count = self.missions_completed
        self.average_route_efficiency = self._mean(
            self.average_route_efficiency, count, metrics['route_efficiency'])
        self.average_task_time = self._mean(
            self.average_task_time, count, metrics['completion_time_s'])
        self.average_heading_error = self._mean(
            self.average_heading_error, count, metrics['average_heading_error_rad'])
        self.average_negotiation_delay = self._mean(
            self.average_negotiation_delay, count, metrics['negotiation_wait_time_s'])
        self.unnecessary_stop_count += int(metrics['unnecessary_stop_count'])
        self.reroute_overhead_m += float(metrics['distance_rerouted_m'])
        self.last_mission = dict(metrics)

        # Shared baseline remains untouched for the first two missions.  Each
        # later mission can move a factor by only 1%, with a total ±5% bound.
        if count >= 3:
            if (metrics['heading_correction_count'] >= 5 or
                    metrics['turn_overshoot_rad'] > 0.30):
                self.speed_factor = _bounded('speed_factor', self.speed_factor-0.01)
                self.steering_factor = _bounded('steering_factor', self.steering_factor-0.01)
            elif (metrics['average_heading_error_rad'] < 0.12 and
                  metrics['unnecessary_stop_count'] == 0 and
                  metrics['task_success']):
                self.speed_factor = _bounded('speed_factor', self.speed_factor+0.01)
                self.lookahead_factor = _bounded('lookahead_factor', self.lookahead_factor+0.01)

    def save(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        temporary = path + '.tmp'
        with open(temporary, 'w', encoding='utf-8') as stream:
            json.dump(asdict(self), stream, indent=2, sort_keys=True)
        os.replace(temporary, path)

    def dictionary(self):
        return asdict(self)
