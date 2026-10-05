import pytest

from living_map_beacons.aging import Freshness, freshness, policy_for, ttl_for


@pytest.mark.parametrize(
    "event_type,fresh_s,stale_s,ttl_s",
    [(0, 300, 900, 1800), (1, 300, 900, 3600),
     (2, 120, 600, 1800), (3, 900, 3600, 65535)],
)
def test_event_policy_boundaries(event_type, fresh_s, stale_s, ttl_s):
    assert freshness(fresh_s - 1, event_type=event_type) is Freshness.FRESH
    assert freshness(fresh_s, event_type=event_type) is Freshness.AGING
    assert freshness(stale_s, event_type=event_type) is Freshness.STALE
    assert freshness(ttl_s, event_type=event_type) is Freshness.EXPIRED


def test_packet_ttl_can_shorten_but_not_extend_policy():
    assert freshness(59, ttl_s=60, event_type=1) is not Freshness.EXPIRED
    assert freshness(60, ttl_s=60, event_type=1) is Freshness.EXPIRED
    assert freshness(1800, ttl_s=65000, event_type=2) is Freshness.EXPIRED


def test_unknown_event_uses_navigation_policy_and_negative_age_is_clamped():
    assert policy_for(99) == policy_for(0)
    assert ttl_for(99) == 1800
    assert freshness(-10, event_type=99) is Freshness.FRESH


def test_fire_memory_ages_faster_than_gas():
    from living_map_beacons.aging import Freshness, freshness, ttl_for

    assert ttl_for(4) == 900
    assert ttl_for(4) < ttl_for(2)
    assert freshness(90, event_type=4) == Freshness.AGING
    assert freshness(90, event_type=2) == Freshness.FRESH
    assert freshness(900, event_type=4) == Freshness.EXPIRED
