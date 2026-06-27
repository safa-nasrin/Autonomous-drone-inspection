import asyncio
import math
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from rrt_planner import RRTStar, Obstacle

HOME_LAT = 47.397971
HOME_LON = 8.5461633

def local_to_gps(x, y, home_lat, home_lon):
    lat = home_lat + (y / 111320.0)
    lon = home_lon + (x / (111320.0 * math.cos(math.radians(home_lat))))
    return lat, lon

async def run():
    print("=== Running RRT* Path Planner ===")
    obstacles = [
        Obstacle(5,  0,  2.5, 1, 10, 5),
        Obstacle(-5, 0,  2.5, 1, 10, 5),
        Obstacle(0,  5,  2.5, 10, 1, 5),
        Obstacle(0, -5,  2.5, 10, 1, 5),
    ]
    start = (0, 0, 20)
    goal  = (8, 8, 20)
    bounds = (-20, 20, -20, 20, 10, 30)

    planner = RRTStar(start, goal, obstacles, bounds)
    path = planner.plan()

    if not path:
        print("No path found!")
        return

    print(f"Path found with {len(path)} waypoints")

    print("\n=== Connecting to Drone ===")
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position OK")
            break

    print("\n=== Building Mission ===")
    mission_items = []
    for i, (x, y, z) in enumerate(path):
        lat, lon = local_to_gps(x, y, HOME_LAT, HOME_LON)
        print(f"WP{i+1}: alt={z:.1f}m")
        mission_items.append(MissionItem(
            latitude_deg=lat,
            longitude_deg=lon,
            relative_altitude_m=z,
            speed_m_s=12,
            is_fly_through=True,
            gimbal_pitch_deg=0,
            gimbal_yaw_deg=0,
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=0,
            camera_photo_interval_s=0,
            acceptance_radius_m=3,
            yaw_deg=float("nan"),
            camera_photo_distance_m=0,
            vehicle_action=MissionItem.VehicleAction.NONE
        ))

    print("\n=== Arming and Flying ===")
    await drone.mission.upload_mission(MissionPlan(mission_items))
    await drone.action.arm()
    await drone.mission.start_mission()

    async for progress in drone.mission.mission_progress():
        print(f"Progress: {progress.current}/{progress.total}")
        if progress.current == progress.total:
            break

    print("Mission complete! Landing...")
    await drone.action.return_to_launch()
    await asyncio.sleep(10)
    await drone.action.land()
    print("Landed!")

if __name__ == "__main__":
    asyncio.run(run())
