import hashlib
import os

import pytest
import yaml

from castor_common import models

MANIFEST = {
    "name": "demo",
    "version": "v2",
    "policy": {
        "team_size": 3, "frame_dim": 45, "history": 3, "action_dim": 3, "rate_hz": 50,
        "slots": [{"file": f"policy_falcon{i}.onnx"} for i in (1, 2, 3)],
        "point_local": [0.0, 0.0, -0.1575],
    },
    "flight": {"setpoint_step_scale": 0.015, "setpoint_leash": 1.5, "setpoint_max_speed": None,
               "velocity_filter_s": 0.1, "goal_box": {"min": [-1, -1, 0.5], "max": [1, 1, 1.5]}},
    "rig": {"cable_length": 2.0, "payload_height": 0.03, "mount_local": [0, 0, -0.1875], "lift_height": 1.0,
            "anchors_local": [[0.5, 0, 0.015], [-0.25, 0.433, 0.015], [-0.25, -0.433, 0.015]]},
}

REPO_MODELS = os.path.normpath(os.path.join(os.path.dirname(__file__), "../../../../models"))


def package(root, model_id="demo/v2", manifest=MANIFEST):
    path = root / model_id
    path.mkdir(parents=True)
    (path / "model.yaml").write_text(yaml.safe_dump(manifest))
    return path


def test_default_comes_from_the_first_root_that_has_one(tmp_path):
    mounted, baked = tmp_path / "mounted", tmp_path / "baked"
    package(baked)
    (baked / "DEFAULT").write_text("demo/v2\n")
    mounted.mkdir()
    m = models.find(roots=(str(mounted), str(baked)))
    assert m.id == "demo/v2" and m.path == str(baked / "demo/v2")

    package(mounted)
    assert models.find(roots=(str(mounted), str(baked))).path == str(mounted / "demo/v2")


def test_runner_parameters_pick_the_slot_and_flight_settings(tmp_path):
    package(tmp_path)
    m = models.find("demo/v2", roots=(str(tmp_path),))
    p = m.runner_parameters(team_size=3, team_index=1)
    assert p["model_path"] == str(tmp_path / "demo/v2/policy_falcon2.onnx")
    assert p["obs_frame_base"] == 42 and p["history"] == 3 and p["rate_hz"] == 50.0
    assert p["setpoint_step_scale"] == 0.015
    assert p["rig_anchor_local"] == [-0.25, 0.433, 0.015] and p["rig_mount_local"] == [0.0, 0.0, -0.1875]
    assert p["lift_height"] == 1.0 and p["rig_cable_length"] == 2.0
    assert p["setpoint_max_speed"] == 0.0 and p["velocity_filter_s"] == 0.1 and p["velocity_gain"] == 1.0
    assert p["policy_point_local"] == [0.0, 0.0, -0.1575]
    assert p["goal_box_min"] == [-1.0, -1.0, 0.5] and p["goal_box_max"] == [1.0, 1.0, 1.5]


def test_team_mismatches_are_refused(tmp_path):
    package(tmp_path)
    m = models.find("demo/v2", roots=(str(tmp_path),))
    with pytest.raises(models.ModelError, match="trained for 3 drones"):
        m.runner_parameters(team_size=4, team_index=0)
    with pytest.raises(models.ModelError, match="slot 3"):
        m.slot_file(3)


def test_missing_models_say_where_they_looked(tmp_path):
    with pytest.raises(models.ModelError, match="no DEFAULT"):
        models.find(roots=(str(tmp_path),))
    with pytest.raises(models.ModelError, match="not found"):
        models.find("other/v1", roots=(str(tmp_path),))
    bad = dict(MANIFEST)
    del bad["flight"]
    package(tmp_path, "bad/v1", bad)
    with pytest.raises(models.ModelError, match="'flight'"):
        models.find("bad/v1", roots=(str(tmp_path),))


@pytest.mark.skipif(not os.path.isdir(REPO_MODELS), reason="the repo's models/ is not next to this checkout")
def test_repo_default_model_is_complete():
    m = models.find(roots=(REPO_MODELS,))
    team = m.manifest["policy"]["team_size"]
    for i in range(team):
        p = m.runner_parameters(team_size=team, team_index=i)
        with open(p["model_path"], "rb") as f:
            assert hashlib.sha256(f.read()).hexdigest() == m.manifest["policy"]["slots"][i]["sha256"]
