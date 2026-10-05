from living_map_beacons.protocol import BeaconPacket
from living_map_beacons.store import BeaconStore


def test_store_round_trip_and_backup_recovery(tmp_path):
    path = tmp_path / "beacons.json"
    store = BeaconStore(str(path))
    first = {1: BeaconPacket(beacon_id=1, sequence=1, x_m=2.5)}
    second = {1: BeaconPacket(beacon_id=1, sequence=2, x_m=3.5)}

    store.save(first)
    store.save(second)
    assert store.load()[1].sequence == 2

    path.write_text("{broken json")
    recovered = store.load()
    assert recovered[1].sequence == 1
    assert store.recovered_from_backup is True
    assert any(tmp_path.glob("beacons.json.corrupt*"))


def test_store_quarantines_unrecoverable_primary(tmp_path):
    path = tmp_path / "beacons.json"
    path.write_text("not json")
    store = BeaconStore(str(path))
    assert store.load() == {}
    assert not path.exists()
    assert any(tmp_path.glob("beacons.json.corrupt*"))
