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

def plan_path(start, goal, obstacles):
    bounds = (-30, 30, -30, 30, 5, 40)
    planner = RRTStar(start, goal, obstacles, bounds, max_iter=3000)
    return planner.plan()

def make_mission(path):
    items = []
    for x, y, z in path:
        lat, lon = local_to_gps(x, y, HOME_LAT, HOME_LON)
        items.append(MissionItem(
            latitude_deg=lat,
            longitude_deg=lon,
            relative_altitude_m=z,
            speed_m_s=10,
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
    return MissionPlan(items)

async def get_current_position(drone):
    async for pos in drone.telemetry.position():
        return pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m

def gps_to_local(lat, lon, home_lat, home_lon):
    x = (lon - home_lon) * 111320.0 * math.cos(math.radians(home_lat))
    y = (lat - home_lat) * 111320.0
    return x, y

async def run():
    # Static obstacles (walls world)
    static_obstacles = [
        Obstacle(5,  0,  2.5, 1, 10, 5),
        Obstacle(-5, 0,  2.5, 1, 10, 5),
        Obstacle(0,  5,  2.5, 10, 1, 5),
        Obstacle(0, -5,  2.5, 10, 1, 5),
    ]

    start = (0, 0, 20)
    goal  = (10, 10, 20)

    print("=== Phase 5: Dynamic Obstacle Avoidance ===")
    print("Planning initial path...")
    path = plan_path(start, goal, static_obstacles)
    if not path:
        print("Initial path not found!")
        return
    print(f"Initial path: {len(path)} waypoints")

    # Connect drone
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

    # Fly initial path
    await drone.mission.upload_mission(make_mission(path))
    await drone.action.arm()
    await drone.mission.start_mission()
    print("Mission started — flying initial path...")

    replanned = False

    async for progress in drone.mission.mission_progress():
        print(f"Progress: {progress.current}/{progress.total}")

        # Inject dynamic obstacle at waypoint 2
        if progress.current == 2 and not replanned:
            print("\n!!! DYNAMIC OBSTACLE DETECTED mid-flight !!!")
            print("Stopping mission and replanning...")

            await drone.mission.pause_mission()
            await asyncio.sleep(1)

            # Get current drone position
            lat, lon, alt = await get_current_position(drone)
            curr_x, curr_y = gps_to_local(lat, lon, HOME_LAT, HOME_LON)
            curr_pos = (curr_x, curr_y, alt)
            print(f"Current position: x={curr_x:.1f}, y={curr_y:.1f}, z={alt:.1f}")

            # Add dynamic obstacle between drone and goal
            dynamic_obstacle = Obstacle(6, 6, 20, 2, 2, 6)
            all_obstacles = static_obstacles + [dynamic_obstacle]

            print("Replanning path around new obstacle...")
            new_path = plan_path(curr_pos, goal, all_obstacles)

            if new_path:
                print(f"New path found: {len(new_path)} waypoints")
                await drone.mission.upload_mission(make_mission(new_path))
                await drone.mission.start_mission()
                replanned = True
                print("Resuming flight on new path!\n")
            else:
                print("Replan failed! Continuing original path.")
                await drone.mission.start_mission()

        if progress.current == progress.total:
            break

    print("\nMission complete! Landing...")
    await drone.action.return_to_launch()
    await asyncio.sleep(10)
    await drone.action.land()
    print("Landed!")

if __name__ == "__main__":
    asyncio.run(run())
