import re
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODELS = ROOT / "src/living_map_sim/models"


def _pose(element):
    return [float(value) for value in element.findtext("pose").split()]


def test_robot_models_reference_existing_meshes():
    for robot, chassis in (("writer", "x2_chassis.dae"), ("executor", "base_link.stl")):
        sdf = (MODELS / robot / "model.sdf").read_text()
        uris = re.findall(r"model://([^<]+)", sdf)
        assert f"{robot}/meshes/{chassis}" in uris
        for uri in uris:
            assert (MODELS / uri).is_file(), uri


def test_skid_steer_drives_all_four_wheels():
    for robot in ("writer", "executor"):
        plugin = ET.parse(MODELS / robot / "model.sdf").getroot().find(".//plugin[@name='gz::sim::systems::DiffDrive']")
        assert len(plugin.findall("left_joint")) == 2
        assert len(plugin.findall("right_joint")) == 2


def test_launch_laser_transforms_match_model_lidar_mounts():
    launch = (ROOT / "src/living_map_bringup/launch/demo.launch.py").read_text()
    for robot in ("writer", "executor"):
        model = ET.parse(MODELS / robot / "model.sdf").getroot()
        # Laser link poses are relative to base_link (the model origin), like the TF.
        laser = next(link for link in model.iter("link") if link.attrib["name"] == "laser")
        x, y, z = _pose(laser)[:3]
        match = re.search(
            rf'arguments=\["([-\d.]+)", "([-\d.]+)", "([-\d.]+)", "0", "0", "0", "{robot}/base_link", "{robot}/laser"\]',
            launch,
        )
        assert match, robot
        assert [float(v) for v in match.groups()] == [x, y, z]


def test_diff_drive_allows_fastest_speed_preset():
    for robot in ("writer", "executor"):
        plugin = ET.parse(MODELS / robot / "model.sdf").getroot().find(".//plugin[@name='gz::sim::systems::DiffDrive']")
        assert float(plugin.findtext("max_linear_velocity")) >= 1.1
        assert float(plugin.findtext("max_angular_velocity")) >= 1.8
