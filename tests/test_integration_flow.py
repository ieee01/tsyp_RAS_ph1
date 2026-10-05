import math,time
from living_map_beacons.protocol import BeaconPacket,encode,decode
from living_map_radio.model import RFModel
from living_map_gateway.dedup import DedupCache
from living_map_gateway.coordinate import map_to_wgs84
from living_map_executor.graph import MemoryGraph
def test_event_beacon_radio_gateway_executor_flow():
 p=BeaconPacket(beacon_id=1,sequence=1,timestamp_sec=int(time.time()),x_m=4,y_m=2,event_type=1,confidence=.96);raw=encode(p);rf=RFModel(42);ok,prob,_=rf.deliver(math.hypot(4,2));assert ok;received=decode(raw);cache=DedupCache();assert cache.accept((received.mission_id,received.beacon_id,received.sequence));lat,lon=map_to_wgs84(received.x_m,received.y_m,36.8065,10.1815,18);assert lat>36.8065 and lon>10.1815;g=MemoryGraph();g.add(received.beacon_id,received.x_m,received.y_m,event_type=received.event_type);assert 1 in g.nodes
