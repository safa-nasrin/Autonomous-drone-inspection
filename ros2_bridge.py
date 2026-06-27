import asyncio
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from mavsdk import System
import threading

HOME_LAT = 47.397971
HOME_LON = 8.5461633

def gps_to_local(lat, lon):
    x = (lon - HOME_LON) * 111320.0 * math.cos(math.radians(HOME_LAT))
    y = (lat - HOME_LAT) * 111320.0
    return x, y

class DroneROS2Bridge(Node):
    def __init__(self):
        super().__init__('drone_inspection_bridge')
        self.pose_pub   = self.create_publisher(PoseStamped, '/drone/pose', 10)
        self.path_pub   = self.create_publisher(Path, '/drone/path', 10)
        self.status_pub = self.create_publisher(String, '/drone/status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/drone/markers', 10)
        self.marker_timer = self.create_timer(2.0, self.publish_building_markers)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        self.get_logger().info('Drone ROS2 Bridge started!')

    def clear_path(self):
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

    def publish_pose(self, lat, lon, alt):
        x, y = gps_to_local(lat, lon)
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = alt
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses.append(msg)
        self.path_pub.publish(self.path_msg)

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'Status: {status}')

    def publish_building_markers(self):
        marker_array = MarkerArray()
        houses = [(30,30),(30,15),(15,30),(-30,-30),(-30,-15),(-15,-30),(30,-30),(-30,30),(15,-30),(-15,30)]
        sky    = [(0,0,8,40),(15,15,8,35),(-15,-15,8,35)]
        mid    = [(-10,15,8,20),(10,-15,8,20),(20,-20,8,15),(-20,20,8,15)]
        wh     = [(-25,0,10,20,10),(25,0,10,20,10)]
        mid_id = 0

        for x, y in houses:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'houses'; m.id = mid_id; mid_id += 1
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 5.0
            m.pose.orientation.w = 1.0
            m.scale.x = 8.0; m.scale.y = 8.0; m.scale.z = 10.0
            m.color.r = 0.76; m.color.g = 0.66; m.color.b = 0.51; m.color.a = 0.8
            marker_array.markers.append(m)

        for x, y, w, h in sky:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'skyscrapers'; m.id = mid_id; mid_id += 1
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = float(h/2)
            m.pose.orientation.w = 1.0
            m.scale.x = float(w); m.scale.y = float(w); m.scale.z = float(h)
            m.color.r = 0.4; m.color.g = 0.4; m.color.b = 0.4; m.color.a = 0.8
            marker_array.markers.append(m)

        for x, y, w, h in mid:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'midrise'; m.id = mid_id; mid_id += 1
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = float(h/2)
            m.pose.orientation.w = 1.0
            m.scale.x = float(w); m.scale.y = float(w); m.scale.z = float(h)
            m.color.r = 0.5; m.color.g = 0.5; m.color.b = 0.5; m.color.a = 0.8
            marker_array.markers.append(m)

        for x, y, dx, dy, dz in wh:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'warehouses'; m.id = mid_id; mid_id += 1
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = float(dz/2)
            m.pose.orientation.w = 1.0
            m.scale.x = float(dx); m.scale.y = float(dy); m.scale.z = float(dz)
            m.color.r = 0.6; m.color.g = 0.6; m.color.b = 0.6; m.color.a = 0.8
            marker_array.markers.append(m)

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(marker_array.markers)} building markers')

async def run_bridge(bridge):
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            bridge.publish_status("CONNECTED")
            break

    bridge.publish_building_markers()

    bridge.publish_status("WAITING_GPS")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            bridge.publish_status("GPS_LOCKED")
            break

    bridge.publish_building_markers()
    bridge.clear_path()
    bridge.publish_status("STREAMING_POSE")
    
    await drone.telemetry.set_rate_position(20.0)
    async for pos in drone.telemetry.position():
        bridge.publish_pose(
            pos.latitude_deg,
            pos.longitude_deg,
            pos.relative_altitude_m
        )
        

def ros2_spin(bridge):
    rclpy.spin(bridge)

def main():
    rclpy.init()
    bridge = DroneROS2Bridge()
    spin_thread = threading.Thread(target=ros2_spin, args=(bridge,), daemon=True)
    spin_thread.start()
    asyncio.run(run_bridge(bridge))
    rclpy.shutdown()

if __name__ == '__main__':
    main()
