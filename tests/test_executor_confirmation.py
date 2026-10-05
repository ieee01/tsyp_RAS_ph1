from living_map_gateway.state import GatewayState


def test_executor_confirmation_is_visible_and_survives_memory_refresh():
    state = GatewayState()
    memory = {"beacon_id": 7, "written_at": 0, "ttl_sec": 1800, "event_type": 1}
    state.upsert_memory(1, 7, dict(memory))
    state.confirm_beacon(7, 123.0)
    confirmed = state.snapshot()["memories"][0]
    assert confirmed["executor_confirmed"] is True
    assert confirmed["confirmed_at"] == 123.0

    state.upsert_memory(1, 7, dict(memory))
    refreshed = state.snapshot()["memories"][0]
    assert refreshed["executor_confirmed"] is True
