"""Checks of payload_link.py on the host: python3 -m pytest components/simulation/stack_sim/test_payload_link.py"""

import math
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from payload_link import PayloadEstimate, PayloadLink, WifiLink, quat_mul, rotvec_quat  # noqa: E402

DT = 1.0 / 250.0
IDENTITY = np.array([0.0, 0.0, 0.0, 1.0])


def config():
    with open(os.path.join(HERE, "../assets/config/payload_link.yaml")) as f:
        return yaml.safe_load(f)


def angle(q):
    return 2.0 * math.atan2(float(np.linalg.norm(q[:3])), abs(float(q[3])))


def test_quaternions():
    yaw90 = rotvec_quat([0.0, 0.0, math.pi / 2])
    np.testing.assert_allclose(yaw90, [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)], atol=1e-12)
    np.testing.assert_allclose(quat_mul(yaw90, yaw90), [0.0, 0.0, 1.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(quat_mul(rotvec_quat([0.0, 0.0, 0.0]), yaw90), yaw90, atol=1e-12)


def test_estimate_errors_have_the_configured_size_and_a_steady_offset():
    cfg = config()["estimate"]
    est = PayloadEstimate(cfg, DT, np.random.default_rng(1))
    true = np.array([0.3, -0.2, 1.0])
    errors, angles = [], []
    for _ in range(int(120.0 / DT)):
        p, q = est.step(true, IDENTITY)
        errors.append(p - true)
        angles.append(angle(q))
    errors = np.array(errors)
    np.testing.assert_allclose(errors.mean(axis=0), est.pos_bias, atol=0.006)  # the offset stays all run
    np.testing.assert_allclose(errors.std(axis=0), est.pos_noise, rtol=0.25)   # around it, the drawn noise
    assert cfg["pos_noise_std_xy"][0] <= est.pos_noise[0] <= cfg["pos_noise_std_xy"][1]
    assert abs(est.yaw_bias) <= 2.0 * cfg["yaw_bias_std"]
    assert max(angles) < math.radians(15.0)


def test_estimate_leaves_at_the_telemetry_rate():
    est = PayloadEstimate(config()["estimate"], DT, np.random.default_rng(2))
    sent = [est.step(np.zeros(3), IDENTITY)[0].copy() for _ in range(250)]
    changes = sum(1 for a, b in zip(sent, sent[1:]) if not np.array_equal(a, b))
    assert 48 <= changes <= 50  # 50 Hz out of 250 physics steps


def test_link_delays_each_drone_by_its_own_amount():
    cfg = dict(config()["link"], jitter_s=0.0, late_probability=0.0)
    link = WifiLink(cfg, 3, DT, np.random.default_rng(3))
    for k in range(200):
        link.push(k)
    for i in range(3):
        assert 0.005 <= link.delay[i] <= 0.030
        assert link.read(i) == 199 - round(link.delay[i] / DT)


def test_late_packets_give_an_older_sample_never_a_newer_one():
    link = WifiLink(dict(config()["link"], late_probability=1.0), 1, DT, np.random.default_rng(4))
    for k in range(200):
        link.push(k)
    reads = [link.read(0) for _ in range(500)]
    assert max(reads) <= 199 and min(reads) >= 199 - len(link.sent) + 1
    assert np.mean(reads) < 199 - round(link.delay[0] / DT) - 3  # on average 50 ms later


def test_same_seed_same_run():
    a, b = PayloadLink(config(), 3, DT, seed=7), PayloadLink(config(), 3, DT, seed=7)
    for _ in range(50):
        a.step(np.ones(3), IDENTITY)
        b.step(np.ones(3), IDENTITY)
    np.testing.assert_array_equal(a.read(1)[0], b.read(1)[0])
    assert "link delay" in a.summary()
