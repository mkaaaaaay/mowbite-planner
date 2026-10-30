"""The ranking's rates (objective.py) against the design's touchstones."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from mowbite_planner.objective import (EARLIER_LEFT, ROUND_LEFT, Cost, allowed_left, better,  # noqa: E402
                                       seconds)

CM2 = 1e-4


def test_rates():
    assert EARLIER_LEFT == 1e-3 and ROUND_LEFT == 5e-3  # mkay 29.10.: 10 and 50 cm2
    assert allowed_left(spins=2, loops=1) == pytest.approx(70 * CM2)
    assert seconds(Cost(left_m2=CM2)) == pytest.approx(6.0)
    assert seconds(Cost(loops=1)) == seconds(Cost(left_m2=50 * CM2)) == pytest.approx(300.0)
    assert seconds(Cost(time_s=10.0, dock_s=10.0)) == pytest.approx(11.0)


@pytest.mark.parametrize('left, wins', [(40, True), (60, False)])
def test_a_loop_saved_is_worth_50_cm2(left, wins):
    saving = Cost(left_m2=left * CM2)
    looping = Cost(loops=1)
    assert better(saving, looping) is wins and better(looping, saving) is (not wins)


def test_a_loop_saved_loses_with_more_than_its_worth_in_time():
    # 30 cm2 (180 s) and 125 s more: more than the loop (300 s), 100 s more: less
    assert better(Cost(left_m2=30 * CM2, time_s=125.0), Cost(loops=1)) is False
    assert better(Cost(left_m2=30 * CM2, time_s=100.0), Cost(loops=1)) is True


@pytest.mark.parametrize('left, wins', [(8, True), (12, False), (20, False)])
def test_a_turn_on_the_spot_saved_is_worth_10_cm2(left, wins):
    assert better(Cost(left_m2=left * CM2), Cost(spins=1)) is wins


def test_within_5_cm2_less_along_the_edges_wins():
    assert better(Cost(left_m2=4 * CM2, edge_m2=0.0, time_s=50.0), Cost(left_m2=0.0, edge_m2=CM2)) is True
    assert better(Cost(left_m2=6 * CM2), Cost(left_m2=0.0, edge_m2=CM2, time_s=50.0)) is False
    assert better(Cost(time_s=1.0), Cost(time_s=1.0)) is None
