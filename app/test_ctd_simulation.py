from ctd_simulation import CTDSimulator
from types import SimpleNamespace

sim = CTDSimulator()
assert sim.snapshot()['executionEnabled'] is False
assert sim.snapshot()['hardwareCommandsSent'] is False
stations = [SimpleNamespace(to_dict=lambda i=i: {'waypointSequence': i}) for i in (1, 3)]
assert sim.start(stations)['phase'] == 'arriving'
phases = [sim.advance()['phase'] for _ in range(8)]
assert phases == ['descending','sampling','retracting','arriving','descending','sampling','retracting','complete'], phases
assert sim.snapshot()['stationIndex'] == 1
sim.reset()
assert sim.start(stations)['phase'] == 'arriving'
assert sim.abort()['phase'] == 'aborted'
assert sim.snapshot()['hardwareCommandsSent'] is False
print('PASS: simulated CTD lifecycle, multi-station, abort, no hardware')
