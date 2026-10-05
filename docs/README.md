# Documentation guide

LivingMap implements the TSYP14 **mines & tunnels** scenario in simulation.
This documentation follows the information from its creation underground to its
use by the next robot.

## Suggested reading

1. Follow [architecture](architecture.md), [beacon design](beacon_protocol.md),
   [coordinate frames](coordinate_frames.md) and [data flow](data_flow.md) for implementation detail.
2. Inspect [recorded evidence](results/README.md) and [failure analysis](failure_analysis.md)
   to understand what has been demonstrated and what remains uncertain.
3. Use the [jury instructions](../instructions.md) to reproduce the simulation.
4. Read the [physical implementation plan](implementation_plan.md) for the final phase.

## Reading the visuals

The same palette is used across the SVG diagrams, report and presentation:

| Color / role | Meaning |
|---|---|
| Blue | Writer, exploration and map coordinates |
| Teal | Beacon memory, RF relays and the gateway |
| Orange | Rescue Executor and victim assistance |
| Red | Fire response or an active hazard |
| Gray | Occupancy geometry, context or planned hardware |

Labels and shapes accompany colors. The [mission figure](diagrams/mission_maps.svg)
uses occupancy cells, event guidance and retained trails from the linked gateway
snapshot. The [radio chart](diagrams/radio_model.svg) is a model curve. The
[results chart](diagrams/results.svg) shows individual logged runs, not averages.

## Evidence conventions

- **Recorded simulation:** a scenario has a log or gateway snapshot in `results/`.
- **Automated check:** a behavior is checked by the central `tests/` suite.
- **Planned hardware:** a physical component or validation step belongs to Phase 2.

The current default scenario and historical recordings are described separately.
Objective completion, confirmed beacon write-back and completed return journeys
are separate outcomes. Their interpretation is documented in
[failure analysis](failure_analysis.md#interpreting-completion).

## Figure sources

Figures are plain, editable SVG assets in `diagrams/`; the report embeds vector
artwork and text. The slide deck reuses those figures. The
[beacon-frame PNG](diagrams/beacon_frame.png) is also included for README display
and easy reuse. Local media/report generators
are excluded from the submission; reading the documents or running the simulation
does not require them. Reveal.js attribution is recorded in
[presentation/vendor/README.md](presentation/vendor/README.md).
