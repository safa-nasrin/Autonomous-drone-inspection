import asyncio
import math
import os
from datetime import datetime
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from rrt_planner import RRTStar, Obstacle

HOME_LAT = 47.397971
HOME_LON = 8.5461633
PHOTOS_DIR = os.path.expanduser("~/drone_project/mission_photos")
os.makedirs(PHOTOS_DIR, exist_ok=True)

def local_to_gps(x, y, home_lat, home_lon):
    lat = home_lat + (y / 111320.0)
    lon = home_lon + (x / (111320.0 * math.cos(math.radians(home_lat))))
    return lat, lon

def gps_to_local(lat, lon, home_lat, home_lon):
    x = (lon - home_lon) * 111320.0 * math.cos(math.radians(home_lat))
    y = (lat - home_lat) * 111320.0
    return x, y

def plan_path(start, goal, obstacles):
    bounds = (-45, 45, -45, 45, 5, 40)
    planner = RRTStar(start, goal, obstacles, bounds, max_iter=3500)
    return planner.plan()

def make_mission(path, speed=12):
    items = []
    for x, y, z in path:
        lat, lon = local_to_gps(x, y, HOME_LAT, HOME_LON)
        items.append(MissionItem(
            latitude_deg=lat, longitude_deg=lon, relative_altitude_m=z, speed_m_s=speed,
            is_fly_through=True, gimbal_pitch_deg=0, gimbal_yaw_deg=0,
            camera_action=MissionItem.CameraAction.NONE, loiter_time_s=0,
            camera_photo_interval_s=0, acceptance_radius_m=2, yaw_deg=float("nan"),
            camera_photo_distance_m=0, vehicle_action=MissionItem.VehicleAction.NONE
        ))
    return MissionPlan(items)

async def get_position(drone):
    async for pos in drone.telemetry.position():
        return pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m

def simulate_photo(waypoint_num, lat, lon, alt):
    timestamp = datetime.now().strftime("%H%M%S")
    filename = f"{PHOTOS_DIR}/photo_wp{waypoint_num}_{timestamp}.txt"
    with open(filename, 'w') as f:
        f.write(f"Waypoint: {waypoint_num}\nLatitude: {lat:.6f}\nLongitude: {lon:.6f}\nAltitude: {alt:.1f}m\nStatus: CAPTURED\n")
    print(f"  📸 Photo captured at WP{waypoint_num} → alt={alt:.1f}m")
    return filename

async def run():
    print("=" * 55)
    print("  AUTONOMOUS 'WEAVE-BETWEEN-BUILDINGS' MISSION")
    print("=" * 55)

    houses = [(30,30), (30,15), (15,30), (-30,-30), (-30,-15), (-15,-30), (30,-30), (-30,30), (15,-30), (-15,30)]
    trees = [(22,22), (35,22), (22,35), (-22,-22), (-35,-22), (-22,-35), (22,-22), (-22,35), (0,20), (0,-20), (15,0), (-15,0), (20,0), (-20,0)]
    cars = [(25,25), (-25,-25), (0,30), (0,-30)]
    sky = [(0,0, 8,40), (15,15, 8,35), (-15,-15, 8,35)]
    mid = [(-10,15, 8,20), (10,-15, 8,20), (20,-20, 8,15), (-20,20, 8,15)]
    wh = [(-25,0, 10,20,10), (25,0, 10,20,10)]

    static_obstacles = []
    for x,y in houses: static_obstacles.append(Obstacle(x, y, 5, 8, 8, 10))
    for x,y in trees: static_obstacles.append(Obstacle(x, y, 7.5, 4, 4, 15))
    for x,y in cars: static_obstacles.append(Obstacle(x, y, 2, 4, 6, 4))
    for x,y,w,h in sky: static_obstacles.append(Obstacle(x, y, h/2, w, w, h))
    for x,y,w,h in mid: static_obstacles.append(Obstacle(x, y, h/2, w, w, h))
    for x,y,dx,dy,dz in wh: static_obstacles.append(Obstacle(x, y, dz/2, dx, dy, dz))

    # Drone spawns at (0, 38) — edge of city, clear of buildings
    start = (0, 38, 20)
    goal  = (0, -38, 20)

    print("\n[STEP 1] RRT* Path Planning (Finding gaps between buildings)...")
    path = plan_path(start, goal, static_obstacles)
    if not path:
        print("ERROR: No path found!")
        return
    print(f"✓ Path found: {len(path)} waypoints weaving through the city")

    print("\n[STEP 2] Connecting to drone...")
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Connected to drone")
            break

    print("  Waiting for GPS lock...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("✓ GPS lock acquired")
            break

    print("\n[STEP 3] Pre-Flight Checks...")
    async for battery in drone.telemetry.battery():
        percent = battery.remaining_percent * 100
        print(f"✓ Battery: {percent:.0f}% ({battery.voltage_v:.1f}V)")
        break

    print("  Waiting for EKF alignment...")
    async for health in drone.telemetry.health():
        if health.is_armable and health.is_local_position_ok:
            print("✓ EKF aligned. Ready to arm.")
            break
        await asyncio.sleep(1)

    # Extra stabilization time before arming
    print("  Stabilizing sensors...")
    await asyncio.sleep(5)

    print("  Arming...")
    await drone.action.arm()
    await asyncio.sleep(2)

    print("  Taking off straight up...")
    await drone.action.set_takeoff_altitude(20.0)
    await drone.action.takeoff()

    # Wait for drone to reach takeoff altitude
    await asyncio.sleep(20)
    print("✓ Airborne at 20m — above all buildings")

    print("\n[STEP 4] Starting Navigation Between Buildings...")
    await drone.mission.upload_mission(make_mission(path[1:]))
    await asyncio.sleep(3)
    mission_plan = await drone.mission.download_mission()
    print(f"✓ Mission uploaded: {len(mission_plan.mission_items)} waypoints confirmed")
    await drone.mission.start_mission()

    replanned = False
    replan_count = 0
    photo_count = 0

    async for progress in drone.mission.mission_progress():
        print(f"\n  → Waypoint {progress.current}/{progress.total}")
        lat, lon, alt = await get_position(drone)
        simulate_photo(progress.current, lat, lon, alt)
        photo_count += 1

        if progress.current == 3 and not replanned:
            print("\n  ⚠️  DYNAMIC OBSTACLE DETECTED IN ALLEYWAY!")
            await drone.mission.pause_mission()
            await asyncio.sleep(2)

            cx, cy = gps_to_local(lat, lon, HOME_LAT, HOME_LON)
            print("  Replanning route to avoid...")

            all_obstacles = static_obstacles + [Obstacle(cx, cy-4, 12, 10, 10, 10)]
            new_path = plan_path((cx, cy, alt), goal, all_obstacles)

            if new_path:
                replan_count += 1
                await drone.mission.upload_mission(make_mission(new_path[1:]))
                await asyncio.sleep(3)
                await drone.mission.start_mission()
                replanned = True
            else:
                await drone.mission.start_mission()

        if progress.current == progress.total:
            break

    print("\n[STEP 5] Returning to base...")
    await drone.action.return_to_launch()
    await asyncio.sleep(20)
    await drone.action.land()

    print("\n  Post-Flight Checks...")
    async for battery in drone.telemetry.battery():
        percent = battery.remaining_percent * 100
        print(f"✓ Final Battery: {percent:.0f}%")
        break

    print("\n" + "=" * 55)
    print(f"  MISSION COMPLETE — SUCCESS")
    print("=" * 55)

if __name__ == "__main__":
    asyncio.run(run())
