"""CTD station configuration validation (offline, read-only prototype).

No MAVLink commands or winch motor actions are performed by this module.
"""

import json
import math
from dataclasses import dataclass
from pathlib import Path
from threading import RLock


@dataclass(frozen=True)
class CTDStation:
    waypoint_sequence: int
    loiter_sequence: int
    latitude: float
    longitude: float
    target_depth: float
    sampling_duration: int

    def to_dict(self) -> dict:
        return {
            "waypointSequence": self.waypoint_sequence,
            "loiterSequence": self.loiter_sequence,
            "latitude": self.latitude,
            "longitude": self.longitude,
            "targetDepth": self.target_depth,
            "samplingDuration": self.sampling_duration,
        }


def _integer(value, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    return value


def _number(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def validate_config(data: dict) -> list[CTDStation]:
    if not isinstance(data, dict) or type(data.get("version")) is not int or data["version"] != 1:
        raise ValueError("CTD configuration version must be 1")
    raw_stations = data.get("ctdStations")
    if not isinstance(raw_stations, list):
        raise ValueError("ctdStations must be a list")

    stations = []
    used_sequences = set()
    previous_loiter = -1
    required = {
        "waypointSequence", "loiterSequence", "latitude", "longitude",
        "targetDepth", "samplingDuration",
    }
    for index, raw in enumerate(raw_stations):
        if not isinstance(raw, dict) or set(raw) != required:
            raise ValueError(f"Station {index}: missing or unexpected fields")
        wp = _integer(raw["waypointSequence"], "waypointSequence")
        loiter = _integer(raw["loiterSequence"], "loiterSequence")
        lat = _number(raw["latitude"], "latitude")
        lon = _number(raw["longitude"], "longitude")
        depth = _number(raw["targetDepth"], "targetDepth")
        duration = _integer(raw["samplingDuration"], "samplingDuration")
        if wp < 0 or loiter != wp + 1 or wp <= previous_loiter:
            raise ValueError(f"Station {index}: invalid or unordered sequence numbers")
        if wp in used_sequences or loiter in used_sequences:
            raise ValueError(f"Station {index}: duplicate sequence number")
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError(f"Station {index}: coordinates out of range")
        if not (0.5 <= depth <= 2.0):
            raise ValueError(f"Station {index}: targetDepth must be 0.5 to 2.0 m")
        if not (1 <= duration <= 3600):
            raise ValueError(f"Station {index}: samplingDuration must be 1 to 3600 s")
        stations.append(CTDStation(wp, loiter, lat, lon, depth, duration))
        used_sequences.update((wp, loiter))
        previous_loiter = loiter
    return stations


def load_config(path: str | Path) -> list[CTDStation]:
    path = Path(path)
    with path.open("r", encoding="utf-8") as handle:
        # Reject NaN/Infinity in JSON, even though Python's parser accepts them by default.
        data = json.load(handle, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(f"Invalid JSON constant: {value}")))
    return validate_config(data)


class CTDMissionCoordinator:
    """Read-only station registry. Execution deliberately disabled."""

    def __init__(self):
        self._stations: list[CTDStation] = []
        self._source: str | None = None
        self._lock = RLock()

    def load(self, path: str | Path) -> dict:
        # Validate first; retain the previous configuration on failure.
        stations = load_config(path)
        with self._lock:
            self._stations = stations
            self._source = str(path)
        return self.status()

    def load_data(self, data: dict) -> dict:
        """Validate and replace the in-memory configuration received from QGC."""
        stations = validate_config(data)
        with self._lock:
            self._stations = stations
            self._source = "http_upload"
        return self.status()

    def match_sequence(self, sequence: int | None, fresh: bool) -> dict | None:
        """Identify a configured item, not physical arrival or sampling readiness."""
        if not fresh or sequence is None:
            return None
        with self._lock:
            for index, station in enumerate(self._stations, start=1):
                if sequence == station.waypoint_sequence:
                    phase = "waypoint_sequence_observed"
                elif sequence == station.loiter_sequence:
                    phase = "loiter_sequence_observed"
                else:
                    continue
                return {"stationIndex": index, "phase": phase, **station.to_dict()}
        return None

    def status(self) -> dict:
        with self._lock:
            return {
                "mode": "configuration_only",
                "executionEnabled": False,
                "source": self._source,
                "stationCount": len(self._stations),
                "stations": [station.to_dict() for station in self._stations],
        }
