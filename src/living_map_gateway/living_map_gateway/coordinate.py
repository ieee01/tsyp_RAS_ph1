"""Coordinate conversion using the documented ENU yaw heading convention.

heading_deg is the counter-clockwise angle, in degrees, from geographic East to the map +X axis (ENU yaw convention). It is NOT a compass bearing. To convert a compass bearing B (clockwise from North): heading_deg = 90 - B.
"""

from __future__ import annotations
import math
R=6378137.0

def map_to_enu(x:float,y:float,heading_deg:float=0.,origin_x:float=0.,origin_y:float=0.)->tuple[float,float]:
    """Translate map coordinates to gateway-relative ENU, then rotate by anchor heading."""
    dx,dy=x-origin_x,y-origin_y; h=math.radians(heading_deg)
    return dx*math.cos(h)-dy*math.sin(h), dx*math.sin(h)+dy*math.cos(h)

def enu_to_wgs84(east:float,north:float,lat0:float,lon0:float)->tuple[float,float]:
    lat=lat0+math.degrees(north/R)
    lon=lon0+math.degrees(east/(R*math.cos(math.radians(lat0))))
    return lat,lon

def map_to_wgs84(x,y,lat0,lon0,heading_deg=0.,origin_x=0.,origin_y=0.):
    e,n=map_to_enu(x,y,heading_deg,origin_x,origin_y);return enu_to_wgs84(e,n,lat0,lon0)


COMPASS_POINTS = ("N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                  "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW")


def map_bearing_to_compass(bearing_rad: float, heading_deg: float = 0.0) -> tuple[float, str]:
    """Convert a map-frame bearing (CCW from map +X) to a compass bearing.

    The map +X axis points ``heading_deg`` CCW from East, so the ENU angle is
    ``heading_deg + bearing`` and the compass bearing (CW from North) is
    ``90 - ENU angle``.
    """
    compass = (90.0 - (heading_deg + math.degrees(bearing_rad))) % 360.0
    return compass, COMPASS_POINTS[int((compass + 11.25) // 22.5) % 16]
