from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_nav2_controller_plugins_are_explicit_and_namespaced():
    for robot in ("writer", "executor"):
        config = yaml.safe_load((ROOT / f"config/nav2/{robot}.yaml").read_text())
        node = config[f"/{robot}/controller_server"]["ros__parameters"]
        assert node["controller_plugins"] == ["FollowPath"]
        assert (
            node["FollowPath"]["plugin"]
            == "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
        )

    launch_text = (ROOT / "src/living_map_bringup/launch/demo.launch.py").read_text()
    assert 'namespace=namespace' in launch_text
    assert 'package="nav2_controller"' in launch_text
    assert 'use_namespace' not in launch_text


def test_ros_setup_is_sourced_with_nounset_disabled():
    text = (ROOT / "scripts/_env.sh").read_text()
    assert "set +u" in text
    assert "source /opt/ros/jazzy/setup.bash" in text


def test_gateway_and_beacon_ros_byte_payloads_use_bytes_not_integer_lists():
    beacon = (ROOT / "src/living_map_beacons/living_map_beacons/node.py").read_text()
    gateway = (ROOT / "src/living_map_gateway/living_map_gateway/node.py").read_text()
    assert "m.data = list(encode" not in beacon
    assert "msg.data = frame" in beacon
    assert 'batch.data = b"".join(frames)' in gateway


def test_command_post_runtime_dependencies_are_declared():
    package_xml = (ROOT / "src/living_map_command_post/package.xml").read_text()
    for dependency in ("python3-fastapi", "python3-uvicorn", "python3-pydantic", "living_map_dashboard"):
        assert f"<exec_depend>{dependency}</exec_depend>" in package_xml
