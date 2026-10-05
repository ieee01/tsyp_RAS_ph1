from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

import yaml

ROOT = Path(__file__).resolve().parents[1]


def _world_pose(root, model_name):
    for element in root.findall(".//include"):
        name = element.findtext("name")
        if name == model_name:
            values = [float(v) for v in element.findtext("pose").split()]
            return values[0], values[1]
    for element in root.findall(".//model"):
        if element.attrib.get("name") == model_name:
            values = [float(v) for v in element.findtext("pose").split()]
            return values[0], values[1]
    raise AssertionError(f"pose for {model_name} not found")


def test_event_and_gateway_config_share_writer_map_frame():
    root = ET.parse(ROOT / "src/living_map_sim/worlds/mine.sdf").getroot()
    wx, wy = _world_pose(root, "writer")
    vx, vy = _world_pose(root, "victim_V01")
    hx, hy = _world_pose(root, "gas_H01")
    fx, fy = _world_pose(root, "fire_F01")
    gx, gy = _world_pose(root, "gateway")

    events = yaml.safe_load((ROOT / "config/events.yaml").read_text())["events"]
    event_by_id = {event["id"]: event for event in events}
    anchor = yaml.safe_load((ROOT / "config/gps_anchor.yaml").read_text())

    assert event_by_id["V01"]["x"] == pytest.approx(vx - wx)
    assert event_by_id["V01"]["y"] == pytest.approx(vy - wy)
    assert event_by_id["H01"]["x"] == pytest.approx(hx - wx)
    assert event_by_id["H01"]["y"] == pytest.approx(hy - wy)
    assert event_by_id["F01"]["x"] == pytest.approx(fx - wx)
    assert event_by_id["F01"]["y"] == pytest.approx(fy - wy)
    v2x, v2y = _world_pose(root, "victim_V02")
    assert (event_by_id["V02"]["x"], event_by_id["V02"]["y"]) == (v2x - wx, v2y - wy)
    assert anchor["map_x"] == gx - wx
    assert anchor["map_y"] == gy - wy


def test_executor_odom_origin_transform_matches_spawn_offset():
    root = ET.parse(ROOT / "src/living_map_sim/worlds/mine.sdf").getroot()
    wx, wy = _world_pose(root, "writer")
    ex, ey = _world_pose(root, "executor")
    launch_text = (ROOT / "src/living_map_bringup/launch/demo.launch.py").read_text()
    assert f'arguments=["{ex-wx:g}", "{ey-wy:g}", "0"' in launch_text


def test_fire_robot_odom_origin_transform_matches_spawn_offset():
    root = ET.parse(ROOT / "src/living_map_sim/worlds/mine.sdf").getroot()
    wx, wy = _world_pose(root, "writer")
    fx, fy = _world_pose(root, "firebot")
    launch_text = (ROOT / "src/living_map_bringup/launch/demo.launch.py").read_text()
    assert f'arguments=["{fx-wx:g}", "{fy-wy:g}", "0", "0", "0", "0", "writer/odom", "firebot/odom"]' in launch_text
    assert '"firebot": (-2.0, 2.0)' in launch_text
