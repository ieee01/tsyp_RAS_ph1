import pytest

from living_map_writer.battery import Battery, BatterySpec


def test_drain_combines_powered_time_and_distance():
    battery = Battery(BatterySpec(idle_pct_per_s=0.1, drive_pct_per_m=1.0))
    battery.update(0.0, (0.0, 0.0))
    for step in range(1, 6):                   # 5 s powered + 5 m driven in 1 m steps
        battery.update(1.0, (float(step), 0.0))
    assert battery.level == pytest.approx(100.0 - 0.5 - 5.0)
    battery.update(1.0, (40.0, 0.0))           # localisation jump: no travel counted
    assert battery.level == pytest.approx(94.4)


def test_turns_back_with_enough_charge_to_get_home():
    battery = Battery(level_pct=40.0)
    home = (0.0, 0.0)
    assert not battery.must_return((5.0, 0.0), home)
    reserve_far = battery.reserve((20.0, 0.0), home)
    assert reserve_far > battery.reserve((5.0, 0.0), home)
    assert battery.must_return((20.0, 0.0), home) == (40.0 <= reserve_far)
    battery.level = reserve_far + 0.1
    assert not battery.must_return((20.0, 0.0), home)
    battery.level = reserve_far - 0.1
    assert battery.must_return((20.0, 0.0), home)


def test_depletion_floors_at_zero():
    battery = Battery(level_pct=0.5)
    battery.update(100.0, None)
    assert battery.level == 0.0 and battery.depleted
