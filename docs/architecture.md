# System architecture

![LivingMap architecture](diagrams/architecture.svg)

The system is split by the **radio boundary**. Inside the mine there is no GPS and no network,
only short-range radio. Outside, the Outside Network Area connects to the world. Nothing
crosses the boundary except through the gateway, and nothing inside reaches the gateway
except over the radio.

## Components

| Component | Where | Responsibility | Package |
|---|---|---|---|
| **Writer robot** | Inside | Explores autonomously (LiDAR, SLAM Toolbox, frontier exploration, Nav2), senses events, decides where to drop beacons, reports its state and battery. Detects when it is stuck; turns back when its battery reaches the reserve needed to get home, or when the mine is mapped. | `living_map_writer`, `living_map_events` |
| **Beacon field** | Inside | Stores each memory in a 33-byte frame, persists it independently of the Writer, advertises it every 3 s, relays neighbours' frames, and accepts one kind of update from robots: "this event is resolved". | `living_map_beacons` |
| **Radio mesh** | Inside | Models the channel: free-space-like decay plus 8 dB per rock wall. Floods beacon frames hop by hop; routes robot telemetry along the most reliable path; drops what has no route. | `living_map_radio` |
| **Executors** | Inside | Two Executor robots running the same software with their own identity: the **Executor** (rescue robot: reaches and assists victims) and the **fire robot** (a second Executor: extinguishes fires). Each waits at the entrance, receives one brief, follows the beacons to its targets, acts on arrival and writes the result back into the beacon. | `living_map_executor` |
| **Outside Network Area** (gateway) | Boundary | The only bridge, with four roles. **RECEIVE**: validates frames (CRC, duplicates, sequence) and keeps the Writer's uploaded map. **TRANSLATE**: converts map positions and bearings to GPS and compass. **CARRY**: sends the latest state over the long-distance link and keeps it through outages. **BRIEF**: validates each mission and delivers one brief to the right robot at the entrance. | `living_map_gateway` |
| **Satellite / wireless link** | Outside | Duplex link to the distant command post: 600 ms latency, jitter, 2 % loss, 32 KB/s bandwidth cap, outages (commands sent during an outage are delivered on recovery). | `living_map_link_sim` |
| **Command Post** | Outside | Read-only live map. Decides **who** goes (a ready robot whose capability matches the event) and **what** the targets are, in priority order. Never sends a route. | `living_map_command_post`, `living_map_dashboard` |
| **Simulation harness** | Outside the system | A separate page (`/harness`) for scenario setup and fault injection: destroy or immobilise the Writer, destroy a beacon, cut the satellite link. These are simulated physical events, not messages of the system. For convenience the gateway node executes them (it holds the simulator controls `writer/fail`, `writer/set_stuck` and `radio/disable_beacon`), but they are not system traffic: in a real mine they would be a rock fall or a flat battery, and they bypass the radio on purpose. The Writer's start / pause command, by contrast, is a real radio command and is refused when the mesh cannot reach the Writer. | `living_map_command_post`, `living_map_gateway` |

## Design rules

1. **The beacons outlive the Writer.** Beacon memories are owned by the beacon store, written
   atomically with a backup. Destroying the Writer deletes nothing.
2. **No shortcut across the radio boundary.** Robot status, the Writer's SLAM map, beacon
   confirmations and beacon updates all go to the robot's own radio. The mesh delivers them only
   if a route exists. A destroyed Writer goes silent after a 3-second "dying gasp".
3. **No direct robot ↔ command post link.** The command post only sees gateway snapshots and
   only sends commands to the gateway, both over the long-distance link.
4. **A robot knows only what the outside knows.** Its brief is the gateway's copy of every beacon
   frame plus the last map the Writer uploaded, delivered in one message at the entrance (direct
   radio range of the gateway). After that it learns only from beacons it hears.
5. **WHO and WHAT outside, HOW inside.** The command post names a robot and its ordered targets.
   Each robot owns its hazard policy and its route.
6. **Robots may only mark an event resolved.** A beacon applies an update only if it names the
   same memory, and only by setting the `RESOLVED` flag on its own copy. Robots can never move or
   rewrite a memory.
7. **Bandwidth is respected.** The command post receives a compact routing tree instead of every
   radio link, and the map only when it changes. A 30-beacon mission stays under 20 KB per
   snapshot on a 32 KB/s link.

## How the Executors move and act

**The beacons are the route.** Each Executor builds a graph from the inherited beacons (every
beacon is a place a robot has physically stood) and drives the shortest path through it, beacon
by beacon, to its target. The Writer's map in the brief only serves to check that each leg is
clear of walls, to skip a beacon when a straight leg is clear, and to let Nav2 drive between
beacons. If the graph does not reach the target, the robot falls back to a hazard-aware detour
toward the event position.

- **Driven links first.** Links between consecutive beacons (the Writer drove them) cost their
  length; links inferred from proximity cost 25 % more. Every link is checked against the walls.
- **Fresh memories first.** Ageing memories cost 15 % more, stale ones 40 % more. Expired
  memories are ignored.
- **Hazards are hard limits.** Every recorded, unresolved gas, fire or blocked path is stamped as a
  lethal disc in the robot's Nav2 planning map (gas 2.0 m, fire 1.8 m, blocked path 0.7 m). A hazard
  heard mid-route, or a fire reported out, triggers a re-plan.
- **Priority order.** Targets are handled in the order the command post sent them: severity, then
  confidence, then freshness. Distance doesn't set the order: in the runs, the Executor drives
  past the nearer victim to reach the more severe one first.
- **Beacons are passed, not parked on.** Within 1 m of an intermediate beacon the robot drives on
  to the next one, instead of positioning a 1.6 m-long body exactly on the spot.
- **Arrival and action.** Being within tolerance of the target is arrival (1.7 m for a victim,
  2.8 m for a fire, where the fire robot stays outside the flames). A fire is approached in a
  straight line from its standoff beacon and the robot stops 2.45 m from it, facing it. A victim's
  stop is chosen with 1.4 m of free space around it, to improve turning and departure clearance. The robot then acts (assists
  for 3 s, or extinguishes for 5 s), writes `RESOLVED` into the beacon and waits until the beacon
  advertises it. A target it cannot reach is reported and skipped; the mission fails only if no
  target was reached. MISSION_COMPLETE can therefore represent partial target success,
  and write-back timeouts must be checked separately.
- **Clear the tunnels.** After its last target, every Executor attempts to drive back to the entrance and
  stop there, reducing the chance of blocking another robot. Return completion is
  reported separately from objective completion. Its parking spot lies at the edge of the
  Writer's map, so if the spot itself is unknown it parks at the nearest mapped free place.

**Scene first, then rescue.** The command post briefs the fire robot first and briefs the
Executor once the fire is out and the fire robot reports it is back at the entrance, or that it
could not get back (a 10-minute timeout is only a safety net). This reduces simultaneous travel, but the fallback on an unsuccessful fire-robot
return or timeout can release the rescue robot before the passage is clear. It is
a dispatch policy, not a safety guarantee.

## Reliability measures

| Problem found during testing | Measure |
|---|---|
| Gazebo publishes the clock ~830 times a second; every simulation-time node decoded every message | A C++ node forwards the clock at 100 Hz; radio, gateway and beacons use wall-clock time. Python node CPU dropped by about two thirds. |
| Under load, a Nav2 lifecycle reply can be lost and a robot's navigation never activates | A bringup guard detects a stalled stack and completes its activation |
| SLAM loop closure shifted the map 1.15 m, misplacing earlier beacons | Loop closing disabled (odometry is accurate); measured drift after a full run: 4 cm |
| Leftover processes from earlier runs interfered with the next launch | Reset stops every launch's whole process tree and waits for Gazebo to exit |
| Gazebo occasionally froze at start-up (no clock, no robots) | A start-up guard restarts Gazebo if no clock arrives within 40 s; tested by freezing it on purpose: robots ready 49 s after launch |
| Waypoint goals forced robots to face east, so they stopped and spun at every beacon, often next to a wall | Each waypoint is oriented along the direction of travel |
| Two large robots blocked each other in the passages around a victim | Scene secured before the rescue; robots return home after their last target; stops keep room to turn |

## Robot platforms (simulation)

| Robot | Drive | Sensors | Extra |
|---|---|---|---|
| Writer | skid-steer | 2D LiDAR, odometry | beacon dispenser (30 beacons), battery model |
| Executor (rescue) | skid-steer | 2D LiDAR, odometry | battery model |
| Fire robot (second Executor) | skid-steer | 2D LiDAR, odometry | extinguisher, battery model |

Each robot runs its own namespaced Nav2 stack (planner, Regulated Pure Pursuit controller,
behaviours) in one Gazebo Harmonic world: a 24 m × 10 m mine with interior rock walls and an
open entrance on the west side.
