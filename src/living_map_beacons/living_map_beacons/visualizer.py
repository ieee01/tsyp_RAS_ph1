import json
import math
import os

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from .protocol import FLAG_JUNCTION, FLAG_RELAY, FLAG_STANDOFF

ROLE_COLOURS = {FLAG_RELAY: (0.7, 0.55, 1.0), FLAG_JUNCTION: (1.0, 0.85, 0.2)}
EVENT_COLOURS = {1: (0.15, 1.0, 0.4), 2: (1.0, 0.25, 0.3), 3: (0.8, 0.6, 1.0), 4: (1.0, 0.55, 0.1)}


class Visualizer(Node):
    """RViz view of persistent beacons: role colours, chain links and guidance arrows."""

    def __init__(self):
        super().__init__('beacon_visualizer', namespace='/living_map')
        self.declare_parameter('store_path', 'runtime/beacons.json')
        self.pub = self.create_publisher(MarkerArray, 'beacon_markers', 10)
        self.create_timer(1.0, self.tick)

    @staticmethod
    def _colour(record):
        if record.get('event_type'):
            return EVENT_COLOURS.get(int(record['event_type']), (1.0, 1.0, 1.0))
        for flag, colour in ROLE_COLOURS.items():
            if int(record.get('flags', 0)) & flag:
                return colour
        return 0.05, 0.8, 1.0

    def tick(self):
        path = self.get_parameter('store_path').value
        if not os.path.exists(path):
            return
        try:
            with open(path) as handle:
                data = json.load(handle)
        except Exception:
            return
        markers = MarkerArray()
        for key, record in data.items():
            beacon_id = int(key)
            x, y = record['x_m'], record['y_m']
            r, g, b = self._colour(record)

            body = Marker()
            body.header.frame_id = 'map'
            body.ns = 'beacons'
            body.id = beacon_id
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position.x, body.pose.position.y, body.pose.position.z = x, y, 0.12
            body.pose.orientation.w = 1.0
            body.scale.x = body.scale.y = 0.18
            body.scale.z = 0.24
            body.color.r, body.color.g, body.color.b, body.color.a = r, g, b, 1.0
            markers.markers.append(body)

            text = Marker()
            text.header.frame_id = 'map'
            text.ns = 'beacon_labels'
            text.id = 1000 + beacon_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x, text.pose.position.y, text.pose.position.z = x, y, 0.45
            text.pose.orientation.w = 1.0
            text.scale.z = 0.22
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f'B{beacon_id:02d}'
            markers.markers.append(text)

            distance = float(record.get('range_m', 0.0))
            if distance > 0 and int(record.get('flags', 0)) & FLAG_STANDOFF:
                bearing = float(record.get('bearing_rad', 0.0))
                arrow = Marker()
                arrow.header.frame_id = 'map'
                arrow.ns = 'beacon_guidance'
                arrow.id = 2000 + beacon_id
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose.orientation.w = 1.0
                arrow.points = [Point(x=x, y=y, z=0.3),
                                Point(x=x + distance * math.cos(bearing), y=y + distance * math.sin(bearing), z=0.3)]
                arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.05, 0.14, 0.2
                arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = r, g, b, 0.9
                markers.markers.append(arrow)
        self.pub.publish(markers)


def main():
    rclpy.init()
    node = Visualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
