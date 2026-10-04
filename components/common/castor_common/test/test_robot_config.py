import json
import os

import pytest
import yaml

from castor_common import bridge_config, cli
from castor_common.robot_config import ConfigError, load, parse

GOOD = """
robot: {id: 2, namespace: drone2, hardware: rpi4}
team: {size: 3, index: 1}
fc: {enabled: true, transport: serial, device: /dev/ttyAMA0, baud: 921600}
zenoh: {connect: ["tcp/10.0.0.5:7447"]}
"""


def cfg_from(text):
    return parse(yaml.safe_load(text), source="test.yaml")


def test_good_config_and_defaults():
    cfg = cfg_from(GOOD)
    assert (cfg.robot_id, cfg.namespace, cfg.hardware) == (2, "drone2", "rpi4")
    assert cfg.one_hot() == [0.0, 1.0, 0.0]
    assert cfg.fc.enabled and cfg.fc.udp_port == 8888
    assert cfg.mavlink.enabled is False and cfg.mavlink.gcs_endpoints == ("127.0.0.1:14550",)
    assert cfg.zenoh.connect == ("tcp/10.0.0.5:7447",)
    assert cfg.env()["CASTOR_NS"] == "drone2"


def test_minimal_config():
    cfg = cfg_from("robot: {id: 1, namespace: drone1, hardware: laptop}\nteam: {size: 1, index: 0}\n")
    assert cfg.one_hot() == [1.0]
    assert cfg.fc.enabled is False


@pytest.mark.parametrize("text, expect", [
    ("robot: {id: 1, namespace: drone1, hardware: rpi5}\nteam: {size: 3, index: 3}", "team.index"),
    ("robot: {id: 0, namespace: drone1, hardware: rpi5}\nteam: {size: 3, index: 0}", "robot.id"),
    ("robot: {id: true, namespace: drone1, hardware: rpi5}\nteam: {size: 3, index: 0}", "robot.id"),
    ("robot: {id: 1, namespace: Drone-1, hardware: rpi5}\nteam: {size: 3, index: 0}", "robot.namespace"),
    ("robot: {id: 1, namespace: drone1, hardware: pi3}\nteam: {size: 3, index: 0}", "robot.hardware"),
    ("robot: {id: 1, namspace: drone1, hardware: rpi5}\nteam: {size: 3, index: 0}", "unknown key 'robot.namspace'"),
    ("robot: {id: 1, namespace: drone1, hardware: rpi5}", "missing required section 'team'"),
    ("robot: {id: 1, namespace: drone1, hardware: rpi5}\nteam: {size: 3, index: 0}\nfcu: {}", "unknown section 'fcu'"),
    ("robot: {id: 1, namespace: drone1, hardware: rpi5}\nteam: {size: 3, index: 0}\nfc: {transport: can}",
     "fc.transport"),
    ("robot: {id: 1, namespace: drone1, hardware: rpi5}\nteam: {size: 3, index: 0}\nzenoh: {connect: [10.0.0.5]}",
     "zenoh.connect"),
    ("robot: {id: 1, namespace: drone1, hardware: rpi5}\nteam: {size: 3, index: 0}\nmavlink: {enabled: true}",
     "mavlink.device"),
])
def test_invalid_configs_name_the_problem(text, expect):
    with pytest.raises(ConfigError) as e:
        cfg_from(text)
    assert expect in str(e.value)


def test_all_errors_reported_together():
    with pytest.raises(ConfigError) as e:
        cfg_from("robot: {id: 0, namespace: X, hardware: pi3}\nteam: {size: 2, index: 5}")
    msg = str(e.value)
    for key in ("robot.id", "robot.namespace", "robot.hardware", "team.index"):
        assert key in msg


def test_missing_file_and_directory(tmp_path):
    with pytest.raises(ConfigError, match="not found"):
        load(str(tmp_path / "nope.yaml"))
    d = tmp_path / "robot.yaml"
    d.mkdir()
    with pytest.raises(ConfigError, match="is a directory"):
        load(str(d))


def test_env_var_path(tmp_path, monkeypatch):
    p = tmp_path / "robot.yaml"
    p.write_text(GOOD)
    monkeypatch.setenv("CASTOR_ROBOT_CONFIG", str(p))
    assert load().namespace == "drone2"


def test_cli_check_env_and_exit_codes(tmp_path, capsys):
    p = tmp_path / "robot.yaml"
    p.write_text(GOOD)
    assert cli.main(["--config", str(p), "check"]) == 0
    assert cli.main(["--config", str(p), "env"]) == 0
    out = capsys.readouterr().out
    assert "export CASTOR_NS=drone2" in out and "export CASTOR_TEAM_INDEX=1" in out
    p.write_text("robot: {id: 1}\n")
    assert cli.main(["--config", str(p), "check"]) == 2


def test_bridge_config(tmp_path):
    cfg = cfg_from(GOOD)
    out = tmp_path / "run" / "bridge.json5"
    bridge_config.write(cfg, str(out))
    doc = json.loads(out.read_text())
    ros = doc["plugins"]["ros2dds"]
    assert doc["mode"] == "router"
    assert doc["connect"]["endpoints"] == ["tcp/10.0.0.5:7447"]
    assert ros["domain"] == 20            # robot.yaml's ros.domain_id, default 20
    assert doc["listen"]["endpoints"] == ["tcp/0.0.0.0:7447"]
    # LOCALHOST mode breaks discovery of late-starting Fast DDS nodes (see bridge_config.py)
    assert ros["ros_automatic_discovery_range"] == "SUBNET"
    assert all(p.startswith("/drone2/") for p in ros["allow"]["publishers"])
    assert not any("fmu" in p for p in ros["allow"]["publishers"] + ros["allow"]["subscribers"])
    assert ros["allow"]["service_servers"] == [] and ros["allow"]["action_clients"] == []
    # team commands, and the payload FC's state (drones ignore each other)
    assert ros["allow"]["subscribers"] == ["/team/.*", "/payload[a-z0-9_]*/vehicle/(odom|state)"]
    # routers only connect to what multicast finds if told to
    assert doc["scouting"]["multicast"]["autoconnect"] == {"router": ["router"]}


def test_sil_keys():
    cfg = cfg_from("robot: {id: 2, namespace: drone2, hardware: sim}\nteam: {size: 3, index: 1}\n"
                   "ros: {domain_id: 22}\nzenoh: {listen_address: 127.0.0.1, listen_port: 7449,"
                   " multicast_scouting: false, connect: ['tcp/127.0.0.1:7447']}")
    doc = bridge_config.render(cfg)
    assert doc["plugins"]["ros2dds"]["domain"] == 22
    assert doc["listen"]["endpoints"] == ["tcp/127.0.0.1:7449"]
    assert "autoconnect" not in doc["scouting"]["multicast"]
    assert cfg.env()["ROS_DOMAIN_ID"] == "22"


def test_payload_role_and_bad_values():
    cfg = cfg_from("robot: {id: 4, namespace: payload, hardware: sim}\nteam: {size: 1, index: 0}\n"
                   "zenoh: {role: payload}")
    assert bridge_config.allowed_subscribers(cfg) == ["/team/.*"]
    with pytest.raises(ConfigError, match="must start with 'payload'"):
        cfg_from("robot: {id: 4, namespace: box, hardware: sim}\nteam: {size: 1, index: 0}\n"
                 "zenoh: {role: payload}")
    with pytest.raises(ConfigError, match="ros.domain_id"):
        cfg_from("robot: {id: 1, namespace: drone1, hardware: rpi5}\nteam: {size: 3, index: 0}\n"
                 "ros: {domain_id: 150}")
    with pytest.raises(ConfigError, match="fc.baud"):
        cfg_from("robot: {id: 1, namespace: drone1, hardware: rpi5}\nteam: {size: 3, index: 0}\n"
                 "fc: {baud: -5}")
    with pytest.raises(ConfigError, match="port out of range"):
        cfg_from("robot: {id: 1, namespace: drone1, hardware: rpi5}\nteam: {size: 3, index: 0}\n"
                 "zenoh: {connect: ['tcp/h:99999']}")
    with pytest.raises(ConfigError, match="section 'robot' is empty"):
        cfg_from("robot:\nteam: {size: 3, index: 0}\n")


def test_ground_station_bridge_mirrors_drones():
    gcs = cfg_from("robot: {id: 9, namespace: gcs, hardware: laptop}\nteam: {size: 1, index: 0}\n"
                   "zenoh: {role: ground_station}")
    allow = bridge_config.render(gcs)["plugins"]["ros2dds"]["allow"]
    assert allow["publishers"] == ["/team/.*"]
    drone = bridge_config.allowed_publishers(cfg_from(GOOD))
    # Every topic a drone publishes is something the ground station subscribes to.
    import re
    for pattern in drone:
        topic = pattern.replace("[a-z_]+", "system").replace("(odom|state)", "odom")
        assert any(re.fullmatch(s, topic) for s in allow["subscribers"]), topic


def test_bad_zenoh_role():
    with pytest.raises(ConfigError, match="zenoh.role"):
        cfg_from("robot: {id: 1, namespace: drone1, hardware: rpi5}\nteam: {size: 3, index: 0}\n"
                 "zenoh: {role: relay}")
