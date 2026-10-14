"""Constant velocity Kalman filter."""

import numpy as np
import pytest

from perimeter_defense.core.kalman import ConstantVelocityKF


def test_predict_moves_with_velocity():
    kf = ConstantVelocityKF(0.0, 0.0, 0.1, 0.1, 0.5, vx=1.3, vy=0.0)
    kf.predict(2.0)
    assert kf.position == pytest.approx((2.6, 0.0))


def test_predict_grows_uncertainty():
    kf = ConstantVelocityKF(0.0, 0.0, 0.2, 1.0, 1.5)
    before = np.trace(kf.P)
    kf.predict(1.0)
    assert np.trace(kf.P) > before


def test_update_reduces_uncertainty_and_moves_towards_measurement():
    kf = ConstantVelocityKF(0.0, 0.0, 1.0, 1.0, 1.0)
    before = kf.position_std()
    kf.update(np.array([1.0, 0.0]), 0.2)
    assert kf.position_std() < before
    assert 0.9 < kf.position[0] <= 1.0


def test_estimates_velocity_of_walking_person():
    rng = np.random.default_rng(0)
    kf = ConstantVelocityKF(0.0, 0.0, 0.2, 1.5, 1.0)
    dt = 0.1
    estimates = []
    for k in range(1, 80):
        kf.predict(dt)
        truth = np.array([1.3 * k * dt, 0.5 * k * dt])
        kf.update(truth + rng.normal(0.0, 0.2, 2), 0.2)
        if k > 40:
            estimates.append(kf.velocity)
    vx, vy = np.mean(estimates, axis=0)
    assert vx == pytest.approx(1.3, abs=0.15)
    assert vy == pytest.approx(0.5, abs=0.15)


def test_covariance_stays_symmetric_positive_definite():
    kf = ConstantVelocityKF(0.0, 0.0, 0.5, 1.0, 1.5)
    for k in range(100):
        kf.predict(0.1)
        if k % 3:
            kf.update(np.array([0.1 * k, 0.0]), 0.2)
    assert np.allclose(kf.P, kf.P.T)
    assert np.all(np.linalg.eigvalsh(kf.P) > 0)


def test_mahalanobis_larger_for_far_measurement():
    kf = ConstantVelocityKF(0.0, 0.0, 0.3, 1.0, 1.0)
    assert kf.mahalanobis2(np.array([0.2, 0.0]), 0.2) < kf.mahalanobis2(np.array([2.0, 0.0]), 0.2)


def test_speed_clamp():
    kf = ConstantVelocityKF(0.0, 0.0, 0.1, 0.1, 1.0, vx=6.0, vy=8.0)
    kf.clamp_speed(3.5)
    assert np.hypot(*kf.velocity) == pytest.approx(3.5)
