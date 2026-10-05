import pytest
from living_map_gateway.coordinate import *
def test_origin_and_axes():
 lat0,lon0=36.8,10.18
 assert map_to_wgs84(0,0,lat0,lon0,0)==pytest.approx((lat0,lon0))
 lat,lon=map_to_wgs84(10,0,lat0,lon0,0);assert lon>lon0 and lat==pytest.approx(lat0,abs=1e-9)
 lat2,lon2=map_to_wgs84(0,10,lat0,lon0,0);assert lat2>lat0
 lat3,lon3=map_to_wgs84(10,0,lat0,lon0,90);assert lat3>lat0 and lon3==pytest.approx(lon0,abs=1e-9)
def test_gateway_map_origin_maps_to_anchor():
 lat0,lon0=36.8065,10.1815
 assert map_to_wgs84(-12.5,0,lat0,lon0,18,-12.5,0)==pytest.approx((lat0,lon0))


def test_compass_east_converts_to_zero_enu_heading():
 compass_bearing = 90.0
 heading_deg = 90.0 - compass_bearing
 lat0,lon0=36.8,10.18
 latitude,longitude=map_to_wgs84(10,0,lat0,lon0,heading_deg)
 assert longitude > lon0
 assert latitude == pytest.approx(lat0,abs=1e-9)
