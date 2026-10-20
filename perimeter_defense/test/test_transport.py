"""Hand-over to a transport UGV (M4)."""

import math

import pytest

from perimeter_defense.core.transport import choose_transport, pickup_point


def test_pickup_point_on_the_destination_side():
    assert pickup_point((0.0, 0.0), (10.0, 0.0), 1.5) == pytest.approx((1.5, 0.0))
    assert pickup_point((2.0, 2.0), (2.0, 12.0), 1.5) == pytest.approx((2.0, 3.5))


def test_pickup_point_close_to_destination():
    # Closer to the destination than the standoff: stop at the destination.
    assert pickup_point((0.0, 0.0), (1.0, 0.0), 1.5) == pytest.approx((1.0, 0.0))
    assert pickup_point((3.0, 4.0), (3.0, 4.0), 1.5) == (3.0, 4.0)


def test_choose_fastest_reachable_transport():
    times = {'trn1': 12.0, 'trn2': 7.0, 'trn3': math.inf}
    assert choose_transport(['trn1', 'trn2', 'trn3'], times.get) == 'trn2'
    assert choose_transport(['trn3'], times.get) is None
    assert choose_transport([], times.get) is None
