"""CASTOR simulation assets: a parametric Holybro S500 quadrotor and an N-drone cable-suspended payload rig.

Everything is generated from the YAML files in ``../config``. ``config`` and ``geometry`` are plain Python + numpy
and import anywhere; ``usd_build`` and ``runtime`` need ``pxr``, which only imports once an Isaac Sim
``SimulationApp`` exists.
"""

import os

ASSETS_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
