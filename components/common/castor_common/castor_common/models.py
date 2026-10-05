"""Policy model packages: models/ in the repo, /opt/castor/models in the planning image (see models/README.md).

A package is <root>/<name>/<version>/model.yaml plus the files it lists. Roots are searched in order, the mounted
override first, then the copy baked into the image. Which package: the id asked for, else the DEFAULT file of the
first root that has one.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import yaml

MOUNTED = "/var/lib/castor/models"
BAKED = "/opt/castor/models"
ROOTS = (MOUNTED, BAKED)


class ModelError(Exception):
    pass


@dataclass
class Model:
    id: str
    path: str
    manifest: dict

    def slot_file(self, team_index: int) -> str:
        slots = self.manifest["policy"]["slots"]
        if not 0 <= team_index < len(slots):
            raise ModelError(f"{self.id} has {len(slots)} slots, this drone is team slot {team_index}")
        return os.path.join(self.path, slots[team_index]["file"])

    def runner_parameters(self, team_size: int, team_index: int) -> dict:
        """castor_policy parameters for the drone in `team_index` of a team of `team_size`."""
        p = self.manifest["policy"]
        if p["team_size"] != team_size:
            raise ModelError(f"{self.id} was trained for {p['team_size']} drones, this team has {team_size}")
        return {
            "model_id": self.id,
            "model_path": self.slot_file(team_index),
            "rate_hz": float(p["rate_hz"]),
            "history": int(p["history"]),
            "obs_frame_base": int(p["frame_dim"]) - team_size,
            "setpoint_step_scale": float(self.manifest["flight"]["setpoint_step_scale"]),
            "setpoint_leash": float(self.manifest["flight"]["setpoint_leash"]),
        }


def load(path: str, model_id: str | None = None) -> Model:
    manifest_path = os.path.join(path, "model.yaml")
    try:
        with open(manifest_path) as f:
            manifest = yaml.safe_load(f)
    except OSError as e:
        raise ModelError(f"no manifest at {manifest_path}: {e.strerror}") from e
    for key in ("policy", "flight", "rig"):
        if not isinstance(manifest.get(key), dict):
            raise ModelError(f"{manifest_path} has no '{key}' section")
    return Model(model_id or f"{manifest.get('name')}/{manifest.get('version')}", path, manifest)


def default_id(roots=ROOTS) -> str | None:
    for root in roots:
        try:
            with open(os.path.join(root, "DEFAULT")) as f:
                text = f.read().strip()
        except OSError:
            continue
        if text:
            return text
    return None


def find(model_id: str | None = None, roots=ROOTS) -> Model:
    """The package `model_id` (<name>/<version>, or a directory), else the default one."""
    if model_id and os.path.isabs(model_id):
        return load(model_id)
    model_id = model_id or default_id(roots)
    if not model_id:
        raise ModelError(f"no model asked for and no DEFAULT in {', '.join(roots)}")
    for root in roots:
        path = os.path.join(root, model_id)
        if os.path.isfile(os.path.join(path, "model.yaml")):
            return load(path, model_id)
    raise ModelError(f"model {model_id} not found in {', '.join(roots)}")
