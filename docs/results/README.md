# Simulation evidence

These files record autonomous Gazebo missions. Event sensing, radios, batteries
and assistance actions are simulated, as described in the technical report.

| Evidence | Scenario | What it demonstrates |
|---|---|---|
| [Destroyed run log](run_destroyed.log) and [snapshot](run_destroyed_state.json) | Writer destroyed; 4 October 2026 | Fire and both victims resolved after Writer loss |
| [Stuck run log](run_stuck.log) | Writer immobilised; 4 October 2026 | Mission continuity while the Writer remains a relay |
| [Return run log](run_return.log) and [snapshot](run_return_state.json) | Writer returns home; 4 October 2026 | Map inheritance and fire/victim mission completion |
| [Relay destruction log](beacon_destroyed.log) | Beacon removed; 4 October 2026 | Mesh routes recover around a destroyed relay |
| [Spacious-layout snapshot](run_spacious_state.json) | Writer destroyed; 5 October 2026 | Fire and both victims resolved; both Executor robots report back at the entrance |

The 4 October snapshots were captured at objective completion; they do not
establish that both Executor robots had finished returning to the entrance.
The 5 October snapshot includes completed return journeys.

These recordings use earlier event configurations with one gas hazard. The
current default includes two gas hazards and waits for both before automatically
ending the Writer's sortie. Recorded results are evidence for their respective
scenarios, rather than a guarantee for every layout or exploration order.

Run the submitted configuration using the [installation and demo instructions](../../instructions.md).
The technical report is available as a [six-page PDF](../report/report.pdf).

## Reading the plots

The [results chart](../diagrams/results.svg) uses the first COMPLETE entries at
219 s, 180 s and 317 s in the respective 4 October watcher logs. Time starts at
the watcher’s scenario start, rather than ROS simulation startup. These are three
individual observations; the layouts and ending conditions differ.

The [mission panels](../diagrams/mission_maps.svg) all use the 5 October final
snapshot. Occupancy cells are decoded from its map runs; event positions are
reconstructed from beacon position plus bearing/range. Each line is the robot’s
retained history, which is bounded and can include only the end of exploration or
the return journey. It must not be interpreted as the complete outbound route.

The [RF chart](../diagrams/radio_model.svg) is calculated from the implemented
distance curve and equivalent-distance wall loss. It is a simulation model,
not a fitted curve or a radio measurement.
