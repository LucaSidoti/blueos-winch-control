"""Terminal-injected GPS CTD rehearsal. Never writes MAVLink or motor commands.

QGC/ArduPilot mission telemetry and uploaded mission validation are read from
the real monitors. GPS, depth and HOME are *simulated* and isolated from those
monitors. The simulation cannot authorize hardware operation.
"""
import math
import time
from threading import RLock
from ctd_lifecycle import CTDLifecycle, Observation

class CTDTerminalRehearsal:
    def __init__(self):
        self.lock = RLock()
        self.reset()

    def reset(self):
        with self.lock:
            self.engine = None
            self.station = None
            self.last_fix = None
            self.last_fix_at = None
            self.depth = None
            self.home = False
            self.events = []
            self.mission_key = None
            self.stations = []
            self.station_index = None
            self.virtual_sequence = None
            self.completed = []

    def _event(self, name):
        self.events.append({"time": round(time.monotonic(), 3), "event": name})
        self.events = self.events[-50:]

    def start(self, *, stations, audit, mission, station_index=0):
        with self.lock:
            if self.engine is not None:
                raise ValueError("Reset rehearsal before restarting")
            if type(station_index) is not int or not 0 <= station_index < len(stations):
                raise ValueError("Invalid station index")
            if not audit.get("missionValidated") or not mission.get("telemetryFresh"):
                raise ValueError("Real mission readback and telemetry must be valid")
            station = dict(stations[station_index])
            # Indoor-only virtual navigation: real autopilot mission sequence is never changed.
            self.stations = [dict(x) for x in stations]
            self.station_index = station_index
            self.virtual_sequence = station["waypointSequence"]
            self.station = station
            self.mission_key = (mission.get("sourceSystem"),
                                mission.get("sourceComponent"))
            self.engine = CTDLifecycle(station, stable_s=5.0)
            self.engine.start(time.monotonic())
            self._event("rehearsal_started")
            return self.snapshot()

    def advance(self, *, mission, audit, physical_cycle):
        with self.lock:
            if self.engine is None:
                raise ValueError("Start rehearsal first")
            if not audit.get("missionValidated") or not mission.get("telemetryFresh"):
                raise ValueError("Real mission validation or telemetry missing")
            if (mission.get("sourceSystem"), mission.get("sourceComponent")) != self.mission_key:
                raise ValueError("ArduPilot identity changed")
            if physical_cycle.get("stationIndex") != self.station_index:
                raise ValueError("Completed physical cycle belongs to another station")
            if not physical_cycle.get("motorReleased", False):
                raise ValueError("Winch command ownership not released")
            if physical_cycle.get("phase") != "complete":
                raise ValueError("Physical cycle must complete before virtual station advancement")
            if self.station_index in self.completed:
                raise ValueError("Station already completed")
            self.completed.append(self.station_index)
            if self.station_index + 1 >= len(self.stations):
                self.virtual_sequence = None
                self._event("virtual_mission_complete")
                return self.snapshot()
            self.station_index += 1
            self.station = dict(self.stations[self.station_index])
            self.virtual_sequence = self.station["waypointSequence"]
            self.engine = CTDLifecycle(self.station, stable_s=5.0)
            self.engine.start(time.monotonic())
            self.last_fix = None
            self.last_fix_at = None
            self.depth = None
            self.home = False
            self._event("virtual_navigation_next_station")
            return self.snapshot()

    def inject(self, *, latitude, longitude, speed_m_s=0.0, depth_m=None,
               home=False, mission=None, audit=None):
        with self.lock:
            if self.engine is None or self.virtual_sequence is None:
                raise ValueError("Start rehearsal first or virtual mission already complete")
            if not isinstance(latitude, (int,float)) or isinstance(latitude,bool) or not math.isfinite(latitude) or not -90 <= latitude <= 90:
                raise ValueError("Invalid latitude")
            if not isinstance(longitude, (int,float)) or isinstance(longitude,bool) or not math.isfinite(longitude) or not -180 <= longitude <= 180:
                raise ValueError("Invalid longitude")
            if not isinstance(speed_m_s, (int,float)) or isinstance(speed_m_s,bool) or not math.isfinite(speed_m_s) or speed_m_s < 0:
                raise ValueError("Invalid speed")
            if depth_m is not None and (not isinstance(depth_m,(int,float)) or isinstance(depth_m,bool) or not math.isfinite(depth_m) or depth_m < 0):
                raise ValueError("Invalid simulated depth")
            if type(home) is not bool:
                raise ValueError("home must be boolean")
            now=time.monotonic()
            self.last_fix=(float(latitude),float(longitude),float(speed_m_s))
            # Virtual waypoint arrival changes only the rehearsal sequence.
            if self.virtual_sequence == self.station["waypointSequence"]:
                a=math.radians(latitude-self.station["latitude"])
                b=math.radians(longitude-self.station["longitude"])
                h=math.sin(a/2)**2+math.cos(math.radians(latitude))*math.cos(math.radians(self.station["latitude"]))*math.sin(b/2)**2
                if 12742000*math.asin(min(1.0, math.sqrt(h))) <= 3.0 and speed_m_s <= 0.3:
                    self.virtual_sequence = self.station["loiterSequence"]
                    self._event("virtual_loiter_entered")
            self.last_fix_at=now
            self.depth=depth_m
            self.home=home
            self._event("simulated_observation_injected")
            return self._step(mission, audit, now)

    def tick(self, *, mission, audit):
        with self.lock:
            return self._step(mission, audit, time.monotonic())

    def _step(self, mission, audit, now):
        if self.engine is None:
            return self.snapshot()
        if (mission.get("sourceSystem"),
            mission.get("sourceComponent")) != self.mission_key:
            self.engine._fail("Mission identity changed", now)
        if self.engine.phase.value == "fault":
            return self.snapshot()
        fix_fresh=self.last_fix_at is not None and 0 <= now-self.last_fix_at <= 2.0
        error=None
        speed=None
        if fix_fresh:
            lat,lon,speed=self.last_fix
            a=math.radians(self.station["latitude"]-lat)
            b=math.radians(self.station["longitude"]-lon)
            x=math.sin(a/2)**2+math.cos(math.radians(lat))*math.cos(math.radians(self.station["latitude"]))*math.sin(b/2)**2
            error=6371000*2*math.asin(min(1,math.sqrt(x)))
        if not fix_fresh and self.engine.phase.value in ("descending","sampling"):
            self.engine._fail("Simulated GPS updates stale", now)
        else:
            self.engine.step(Observation(
                now=now,sequence=self.virtual_sequence,
                telemetry_fresh=bool(mission.get("telemetryFresh")),
                mission_validated=bool(audit.get("missionValidated")),
                position_error_m=error,speed_m_s=speed,
                depth_m=self.depth,depth_feedback_ok=self.depth is not None,
                home_verified=self.home))
        return self.snapshot()

    def snapshot(self):
        state=self.engine.snapshot() if self.engine else {"phase":"idle"}
        return {"mode":"terminal_gps_simulation_isolated",
                "virtualSequence":self.virtual_sequence,
                "stationIndex":self.station_index,
                "completedStationIndices":list(self.completed),
                "stationCount":len(self.stations),
                "rehearsal":state,"station":self.station,
                "simulatedGPS":({"latitude":self.last_fix[0],"longitude":self.last_fix[1],
                                 "speedMps":self.last_fix[2],
                                 "ageSeconds":round(time.monotonic()-self.last_fix_at,2)}
                                if self.last_fix else None),
                "simulatedDepthMeters":self.depth,"simulatedHome":self.home,
                "events":list(self.events),
                "realGPSOverridden":False,"hardwareCommandsSent":False,
                "mavlinkCommandsSent":False,"automaticWinchEnabled":False,
                "automaticAdvanceEnabled":False}

terminal_rehearsal=CTDTerminalRehearsal()
