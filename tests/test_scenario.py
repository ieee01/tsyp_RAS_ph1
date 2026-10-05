"""Scenario placement must affect sensing and Gazebo consistently."""

import asyncio
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import httpx
import pytest
import yaml

from living_map_command_post.scenario import (prepare_scenario, scenario_is_current, scenario_layout,
                                                 validate_positions)
from living_map_command_post.state import CommandPostState
from living_map_command_post.web import create_app

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def scenario_root(tmp_path):
    """Copy source fixtures to a disposable workspace."""
    for relative in ("config/events.yaml", "src/living_map_sim/worlds/mine.sdf"):
        target = tmp_path / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / relative, target)
    return tmp_path


def test_positions_update_sensor_and_gazebo_in_matching_frames(scenario_root):
    events = scenario_layout(scenario_root)["events"]
    events[0].update(x=7.0, y=-2.0)
    prepare_scenario(scenario_root, events)
    config = yaml.safe_load((scenario_root / "runtime/scenario/events.yaml").read_text())
    victim = next(e for e in config["events"] if e["id"] == "V01")
    assert (victim["x"], victim["y"], victim["detection_radius"]) == (7.0, -2.0, 2.0)
    world = ET.parse(scenario_root / "runtime/scenario/mine.sdf")
    pose = world.findtext(".//model[@name='victim_V01']/pose")
    assert [float(v) for v in pose.split()[:2]] == [-4.0, -2.0]
    assert scenario_layout(scenario_root)["events"][0]["x"] == 7.0
    assert (scenario_root / "config/events.yaml").read_text() == (ROOT / "config/events.yaml").read_text()


@pytest.mark.parametrize("x,y", [(3, 2), (22, 0), (0, 5), (float('nan'), 0)])
def test_invalid_positions_do_not_write_scenario(scenario_root, x, y):
    events = scenario_layout(scenario_root)["events"]
    events[0].update(x=x, y=y)
    with pytest.raises(ValueError):
        prepare_scenario(scenario_root, events)
    assert not (scenario_root / "runtime/scenario").exists()


def test_missing_or_duplicate_events_rejected(scenario_root):
    layout = scenario_layout(scenario_root)
    with pytest.raises(ValueError):
        validate_positions(layout["events"][:2], layout)
    with pytest.raises(ValueError):
        validate_positions([layout["events"][0]] * 3, layout)


def test_scenario_api_validates_before_restart(scenario_root):
    calls = []

    def restart(mode, speed, events):
        """Validate the configuration before accepting a restart."""
        prepare_scenario(scenario_root, events)
        calls.append((mode, speed, events))
        return True

    app = create_app(CommandPostState(), lambda p: True, restart=restart,
                     scenario=lambda: scenario_layout(scenario_root))

    async def exercise():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as client:
            layout = (await client.get("/api/scenario")).json()
            assert layout["frame"] == "writer/odom"
            payload = {"mode": "autonomous", "events": layout["events"]}
            assert (await client.post("/api/demo/restart", json=payload)).status_code == 200
            payload["events"][0].update(x=3, y=2)
            assert (await client.post("/api/demo/restart", json=payload)).status_code == 422
            payload["events"][0]["id"] = "unknown"
            assert (await client.post("/api/demo/restart", json=payload)).status_code == 422

    asyncio.run(exercise())
    assert len(calls) == 1


def _paths(root):
    return root / "runtime/scenario", root / "src/living_map_sim/worlds/mine.sdf", root / "config/events.yaml"


def test_freshly_prepared_scenario_is_current(scenario_root):
    assert not scenario_is_current(*_paths(scenario_root))
    prepare_scenario(scenario_root, scenario_layout(scenario_root)["events"])
    assert scenario_is_current(*_paths(scenario_root))


def test_scenario_saved_before_a_robot_was_added_is_ignored(scenario_root):
    events = scenario_layout(scenario_root)["events"]
    events[0].update(x=7.0, y=-2.0)
    prepare_scenario(scenario_root, events)
    world_file = scenario_root / "runtime/scenario/mine.sdf"
    world = ET.parse(world_file)
    world_element = world.find(".//world")
    world_element.remove(next(i for i in world_element.findall("include") if i.findtext("name") == "firebot"))
    world.write(world_file, encoding="unicode", xml_declaration=True)
    assert not scenario_is_current(*_paths(scenario_root))
    # The harness falls back to the configured positions instead of the stale replay.
    configured = yaml.safe_load((scenario_root / "config/events.yaml").read_text())["events"][0]["x"]
    assert scenario_layout(scenario_root)["events"][0]["x"] == configured


def test_scenario_missing_an_event_is_ignored(scenario_root):
    prepare_scenario(scenario_root, scenario_layout(scenario_root)["events"])
    events_file = scenario_root / "runtime/scenario/events.yaml"
    config = yaml.safe_load(events_file.read_text())
    config["events"] = [e for e in config["events"] if e["id"] != "V02"]
    events_file.write_text(yaml.safe_dump(config))
    assert not scenario_is_current(*_paths(scenario_root))


def test_corrupt_scenario_is_ignored(scenario_root):
    prepare_scenario(scenario_root, scenario_layout(scenario_root)["events"])
    (scenario_root / "runtime/scenario/mine.sdf").write_text("<sdf><world>")
    assert not scenario_is_current(*_paths(scenario_root))
