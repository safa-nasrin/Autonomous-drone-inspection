import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position OK")
            break

    async for position in drone.telemetry.position():
        home_lat = position.latitude_deg
        home_lon = position.longitude_deg
        print(f"Home position: {home_lat}, {home_lon}")
        break

    def make_waypoint(lat, lon, alt):
        return MissionItem(
            latitude_deg=lat,
            longitude_deg=lon,
            relative_altitude_m=alt,
            speed_m_s=25,
            is_fly_through=True,
            gimbal_pitch_deg=0,
            gimbal_yaw_deg=0,
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=0,
            camera_photo_interval_s=0,
            acceptance_radius_m=1,
            yaw_deg=float("nan"),
            camera_photo_distance_m=0,
            vehicle_action=MissionItem.VehicleAction.NONE
        )

    mission_items = [
        make_waypoint(home_lat + 0.0001, home_lon,          30),
        make_waypoint(home_lat + 0.0001, home_lon + 0.0001, 30),
        make_waypoint(home_lat,          home_lon + 0.0001, 30),
    ]

    mission_plan = MissionPlan(mission_items)

    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)

    print("-- Arming")
    await drone.action.arm()

    print("-- Starting mission")
    await drone.mission.start_mission()

    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: {mission_progress.current}/{mission_progress.total}")
        if mission_progress.current == mission_progress.total:
            print("-- Mission complete, returning home")
            break

    await drone.action.return_to_launch()
    await asyncio.sleep(15)
    await drone.action.land()
    print("-- Landed!")

if __name__ == "__main__":
    asyncio.run(run())
