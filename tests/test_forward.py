from living_map_gateway.forward import LatestSnapshotForwarder


def test_outage_recovery_forwards_only_latest_snapshot():
    forwarder = LatestSnapshotForwarder(link_up=False)
    assert forwarder.offer("snapshot-1") is None
    assert forwarder.offer("snapshot-2") is None
    assert forwarder.offer("snapshot-3") is None
    assert forwarder.queued_snapshots == 1

    decision = forwarder.set_link(True)
    assert decision is not None
    assert decision.payload == "snapshot-3"
    assert decision.discarded_snapshots == 2
    assert forwarder.queued_snapshots == 0
    assert forwarder.set_link(True) is None
