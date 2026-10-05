from living_map_link_sim.model import LinkModel


def test_outage_store_and_forward_delivers_all_snapshots_after_restore():
    link = LinkModel(
        seed=1, latency_ms=600, jitter_ms=0, packet_loss=0,
        bandwidth_bytes_per_sec=1000, initially_up=False,
    )
    snapshots = ['{"memory":"B001"}', '{"memory":"B002"}']
    for payload in snapshots:
        assert link.submit("downlink", payload, now=0.0)
    assert link.ready(10.0) == []
    assert len(link.outage_queue) == 2

    link.set_up(True, now=10.0)
    deliveries = link.ready(20.0)
    assert [item.payload for item in deliveries] == snapshots
    assert all(item.direction == "downlink" for item in deliveries)


def test_mission_uses_reverse_uplink_with_latency_and_bandwidth():
    link = LinkModel(
        seed=2, latency_ms=600, jitter_ms=0, packet_loss=0,
        bandwidth_bytes_per_sec=100, initially_up=True,
    )
    assert link.submit("uplink", "x" * 100, now=5.0)
    assert link.ready(6.59) == []
    delivery = link.ready(6.60)
    assert len(delivery) == 1
    assert delivery[0].direction == "uplink"


def test_seeded_loss_is_reproducible():
    first = LinkModel(seed=42, packet_loss=0.5)
    second = LinkModel(seed=42, packet_loss=0.5)
    assert [first.submit("downlink", str(i), 0.0) for i in range(20)] == [
        second.submit("downlink", str(i), 0.0) for i in range(20)
    ]
