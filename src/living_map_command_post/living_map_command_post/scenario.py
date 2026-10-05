"""Prepare simulated event locations in Writer odometry, before a replay."""

import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

EVENT_MODELS = {"V01": "victim_V01", "V02": "victim_V02", "H01": "gas_H01",
                "H02": "gas_H02", "F01": "fire_F01"}


def _robots(world_file: Path) -> set[str]:
    return {include.findtext("name") for include in ET.parse(world_file).findall(".//world/include")}


def _event_ids(events_file: Path) -> set[str]:
    return {event["id"] for event in yaml.safe_load(Path(events_file).read_text())["events"]}


def scenario_is_current(scenario_dir: Path, world_file: Path, events_file: Path) -> bool:
    """A saved replay is usable only if it has the current robots and events.

    A scenario saved before a robot or an event was added to the source world
    would otherwise silently replace it (and, for example, hide the fire robot).
    """
    scenario_dir = Path(scenario_dir)
    try:
        return (_robots(scenario_dir / "mine.sdf") == _robots(Path(world_file))
                and _event_ids(scenario_dir / "events.yaml") == _event_ids(Path(events_file)))
    except (OSError, ET.ParseError, yaml.YAMLError, KeyError, TypeError):
        return False


def scenario_layout(root: Path) -> dict:
    """Read active event positions and reference walls from the simulation files."""
    world = ET.parse(root / "src/living_map_sim/worlds/mine.sdf")
    walls = []
    for model in world.findall(".//world/model"):
        if not model.get("name", "").startswith("wall"):
            continue
        pose = [float(v) for v in model.findtext("pose").split()]
        size = [float(v) for v in model.findtext("link/collision/geometry/box/size").split()]
        walls.append({"x": pose[0] + 11, "y": pose[1], "width": size[0], "height": size[1]})
    scenario_dir = root / "runtime/scenario"
    config = root / "config/events.yaml"
    if scenario_is_current(scenario_dir, root / "src/living_map_sim/worlds/mine.sdf", config):
        config = scenario_dir / "events.yaml"
    events = yaml.safe_load(config.read_text())["events"]
    return {"frame": "writer/odom", "walls": walls,
            "events": [{"id": e["id"], "type": e["type"], "x": e["x"], "y": e["y"]}
                       for e in events if e["id"] in EVENT_MODELS]}


def validate_positions(events: list[dict], layout: dict) -> None:
    """Reject incomplete scenarios and points outside the mine or near walls."""
    if len(events) != len(EVENT_MODELS) or {e["id"] for e in events} != set(EVENT_MODELS):
        raise ValueError("Place exactly two victims (V01, V02), two gases (H01, H02) and one fire (F01).")
    for event in events:
        x, y = float(event["x"]), float(event["y"])
        if not math.isfinite(x) or not math.isfinite(y) or not (-2.5 <= x <= 20.5 and -4.5 <= y <= 4.5):
            raise ValueError(f"{event['id']}: choose a point inside the mine.")
        for wall in layout["walls"]:
            if (abs(x - wall["x"]) <= wall["width"] / 2 + 0.4
                    and abs(y - wall["y"]) <= wall["height"] / 2 + 0.4):
                raise ValueError(f"{event['id']}: choose a point at least 0.4 m from walls.")


def prepare_scenario(root: Path, events: list[dict]) -> None:
    """Write matching event-sensor configuration and Gazebo visuals for a replay."""
    validate_positions(events, scenario_layout(root))
    config = copy.deepcopy(yaml.safe_load((root / "config/events.yaml").read_text()))
    positions = {e["id"]: e for e in events}
    for event in config["events"]:
        if event["id"] in positions:
            event.update(x=float(positions[event["id"]]["x"]), y=float(positions[event["id"]]["y"]))
    world = ET.parse(root / "src/living_map_sim/worlds/mine.sdf")
    for event_id, model_name in EVENT_MODELS.items():
        pose = world.find(f".//world/model[@name='{model_name}']/pose")
        values = pose.text.split()
        values[:2] = [str(float(positions[event_id]["x"]) - 11), str(float(positions[event_id]["y"]))]
        pose.text = " ".join(values)
    directory = root / "runtime/scenario"
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "events.yaml").write_text(yaml.safe_dump(config, sort_keys=False))
    world.write(directory / "mine.sdf", encoding="unicode", xml_declaration=True)
