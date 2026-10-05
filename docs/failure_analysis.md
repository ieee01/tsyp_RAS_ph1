# Failure cases

Beacon memories are stored independently of the Writer so its loss does not delete them.
Each row distinguishes the failure response from its recorded or automated evidence.
The gateway remains a single point of failure.

**Evidence key:** 🟢 shown in a recorded Gazebo run (logs in [`results/`](results/)) · 🔵 covered by
automated tests (`tests/`) · ⚪ implemented, not yet demonstrated in a recorded run.

## Robots

| # | Failure | Detection | System response | Evidence |
|---|---|---|---|---|
| 1 | **Writer destroyed** | Fault injection; its radio sends FAILED for 3 s ("dying gasp"), then goes silent | Exploration goals cancelled; beacons keep advertising; the command post sends the Executors | 🟢 destroyed run: fire out and both victims assisted; Writer shown *radio silent* |
| 2 | **Writer stuck** (rubble, wheels spinning) | No 0.3 m of progress in 45 s while driving, or fault injection | Writer reports STUCK and **keeps its radio active**; the Executors are sent | 🟢 stuck run, Writer still reporting over the mesh · 🔵 |
| 3 | **Writer battery low** | Battery model: charge needed to drive home × 1.5 + 10 % | Writer turns back *before* it can no longer get home | 🔵 |
| 4 | **Writer battery depleted** | Charge reaches 0 % | Treated like a loss: dying gasp, then silence; its memories remain | 🔵 battery model |
| 5 | **Writer cannot get home** | 3 failed attempts to reach the entrance | Writer declares itself STUCK; its knowledge is already out | 🔵 return state machine |
| 6 | **Executors block each other** | A robot parked in a passage; a robot that cannot turn next to a wall | Scene secured before the Executor goes in; every robot returns to the entrance after its last target; stops keep 1.4 m to turn; arrival within tolerance instead of fighting for the exact spot | 🟢 found in testing and fixed |
| 7 | **A target cannot be reached** | No safe stop within tolerance, or Nav2 retries exhausted | That target is reported "not reached" and skipped; the robot continues with the others; MISSION FAILED only if none was reached; inspect target details because MISSION COMPLETE can include partial success | 🟢 a victim placed beside the blocked path in the far east room was reported, not faked |
| 8 | **Navigation stack fails to start** | A Nav2 lifecycle reply is lost under load; no progress for 15 s | A bringup guard completes the activation of the stalled nodes | 🟢 repaired a stalled Writer stack live |
| 9 | **Simulator freezes at start-up** | No simulation clock 40 s after launch | A start-up guard kills the frozen Gazebo; the launch respawns it | 🟢 freeze reproduced on purpose; robots ready 49 s after launch |

## Beacons and radio

| # | Failure | Detection | System response | Evidence |
|---|---|---|---|---|
| 10 | **Beacon destroyed** | Fault injection; it stops transmitting | The mesh re-routes around it; its memory remains at the gateway and in every brief | 🟢 relay destroyed, all 5 dependent beacons re-routed, none cut off ([log](results/beacon_destroyed.log)) · 🔵 |
| 11 | **Rock blocks the radio** (deep drifts) | Writer measures its link into the mesh | Drops a RELAY beacon before the link falls below 85 %; frames flood hop by hop | 🟢 flooding: memories from behind walls arrive in 2–3 hops · 🔵 relay drop |
| 12 | **Robot out of radio reach** | No telemetry for 4 s | Dashboard marks it *radio silent*; the gateway refuses commands it cannot deliver | 🟢 destroyed Writer · 🔵 |
| 13 | **Corrupted frame** | CRC32 mismatch | Frame rejected, never stored | 🔵 |
| 14 | **Duplicate or out-of-order frame** | (mission, beacon, sequence) key; sequence comparison | Duplicates dropped; an older copy never overwrites a newer memory | 🔵 |
| 15 | **Invalid beacon update** | Request does not name the same memory, or is not a resolve | Rejected; a robot can only mark an event resolved, never move or rewrite a memory | 🔵 |
| 16 | **Beacon update not acknowledged** | The robot does not hear the resolved version within 20 s | The robot resends every second, then completes anyway and says the update is unconfirmed | 🔵 · 🟢 acknowledged in every run |
| 17 | **Beacon dispenser running low** | Count vs capacity (30) | Last 4 beacons reserved for events; relays may use half the reserve | 🔵 |

## Information and links

| # | Failure | Detection | System response | Evidence |
|---|---|---|---|---|
| 18 | **Information goes stale** | Per-type age policy + TTL in each frame | FRESH → AGING → STALE → EXPIRED; expired memories ignored; stale hazards **still avoided** | 🔵 |
| 19 | **The situation changes during the mission** | A robot hears a new hazard, or hears that a fire is out | Its planning map is updated and its route re-planned | 🟢 earlier runs, recorded when both Executors were briefed together: the Executor re-planned when the fire robot reported the fire out · under the current scene-first order the Executor is briefed after the fire is out, so it starts with the fire already resolved |
| 20 | **Satellite link outage** | Link status | Gateway keeps only the newest state and sends it on recovery; commands sent meanwhile are delivered on recovery | 🟢 20 s outage during exploration, run completed · 🔵 |
| 21 | **Crash while saving memories** | Corrupt or partial store file at start-up | Atomic write + fsync + backup; corrupt files quarantined, backup restored | 🔵 |
| 22 | **SLAM correction drifts** | Map-to-odometry correction grows | Loop closing disabled with accurate odometry (measured drift 4 cm instead of 1.15 m) | 🟢 |
| 23 | **Writer destroyed out of radio reach** | Destroyed deep in the mine, its "dying gasp" cannot get out; it only falls silent | Once destroyed, a Writer that goes radio silent counts as lost: the robots are briefed anyway, since its memories are already out. An exploring Writer that is briefly silent behind a wall is not treated as lost | 🔵 · found in a recorded run |
| 24 | **A long robot cannot settle on a spot** | Nav2 reports no progress near a beacon, the fire stop or the parking spot | Intermediate beacons are passed within 1 m; a fire is approached in a straight line and the robot stops 2.45 m from it; an unknown parking spot is replaced by the nearest mapped free place | 🔵 straight fire approach · ⚪ pass-through and parking · found in recorded runs |

## Interpreting completion

`MISSION_COMPLETE` means the robot finished processing its target list. The current
implementation also uses that state when some targets were skipped, provided at
least one was handled, or when a write-back timed out after a simulated action.
The command-post summary can overstate resolution in the timeout case. Therefore
check each target detail and its `RESOLVED` memory instead of relying on the banner.

Return travel starts after this objective state. Check **back at the entrance**
separately; **could not get back** records an unsuccessful return. The scene-first
controller can release the rescue robot on that unsuccessful-return report or its
timeout, so sequential dispatch alone does not prove the passage is clear.

The reported successful scenarios are supported by the recorded resolved memories.
The spacious-layout snapshot additionally includes successful returns
for both Executor robots. See [evidence scope](results/README.md).

## Residual risks (honest limits)

- A single gateway is a single point of failure. Phase 2 could add a second gateway at another
  entrance; the beacons would flood to both.
- If every path out is destroyed, memories wait in the beacons and arrive when a robot or relay
  reconnects. They cannot reach the outside sooner.
- Sensor faults (smoke blinding the LiDAR, a drifting gas sensor) are outside Phase 1, which
  uses simulated sensing and ideal odometry.
- Gas has no robot that can neutralise it; every robot only avoids it.
- A memory contradicted by live sensing (a recorded blockage that has been cleared) is not yet
  detected: robots trust it until it expires. The SUSPECT state that fixes this is designed for
  Phase 2 ([beacon design](beacon_protocol.md#6-message-aging)).
