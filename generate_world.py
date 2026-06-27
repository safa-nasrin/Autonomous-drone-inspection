python3 << 'EOF'
import os

home = os.path.expanduser('~')
world_path = os.path.join(home, 'PX4-Autopilot/Tools/simulation/gz/worlds/mission_world.sdf')

# 1. Start the world and add required sensors (Barometer, etc. included!)
world_content = '''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="mission_world">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"></plugin>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"></plugin>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"></plugin>
    <plugin filename="gz-sim-altimeter-system" name="gz::sim::systems::Altimeter"></plugin>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"></plugin>
    
    <spherical_coordinates><surface_model>EARTH_WGS84</surface_model><latitude_deg>47.397971</latitude_deg><longitude_deg>8.5461633</longitude_deg><elevation>0</elevation></spherical_coordinates>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose><diffuse>1 1 1 1</diffuse></light>
    
    <model name="ground_plane"><static>true</static><link name="link"><collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision><visual name="visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual></link></model>
    <include>
      <uri>model://x500</uri>
      <name>drone_1</name>
      <pose>0 0 0.5 0 0 0</pose>
    </include>
    <model name="edge_n"><static>true</static><pose>0 48 10 0 0 0</pose><link name="l"><visual name="v"><geometry><box><size>100 4 20</size></box></geometry><material><ambient>0.2 0.2 0.2 1</ambient></material></visual><collision name="c"><geometry><box><size>100 4 20</size></box></geometry></collision></link></model>
    <model name="edge_s"><static>true</static><pose>0 -48 10 0 0 0</pose><link name="l"><visual name="v"><geometry><box><size>100 4 20</size></box></geometry><material><ambient>0.2 0.2 0.2 1</ambient></material></visual><collision name="c"><geometry><box><size>100 4 20</size></box></geometry></collision></link></model>
    <model name="edge_e"><static>true</static><pose>48 0 10 0 0 0</pose><link name="l"><visual name="v"><geometry><box><size>4 100 20</size></box></geometry><material><ambient>0.2 0.2 0.2 1</ambient></material></visual><collision name="c"><geometry><box><size>4 100 20</size></box></geometry></collision></link></model>
    <model name="edge_w"><static>true</static><pose>-48 0 10 0 0 0</pose><link name="l"><visual name="v"><geometry><box><size>4 100 20</size></box></geometry><material><ambient>0.2 0.2 0.2 1</ambient></material></visual><collision name="c"><geometry><box><size>4 100 20</size></box></geometry></collision></link></model>
'''

# 2. Coordinates mapped to fill the empty spaces
houses = [(30,30), (30,15), (15,30), (-30,-30), (-30,-15), (-15,-30), (30,-30), (-30,30), (15,-30), (-15,30)]
trees = [(22,22), (35,22), (22,35), (-22,-22), (-35,-22), (-22,-35), (22,-22), (-22,35), (0,20), (0,-20), (15,0), (-15,0), (20,0), (-20,0)]
cars = [(25,25), (-25,-25), (0,30), (0,-30)]
sky = [(0,0, 8,40), (15,15, 8,35), (-15,-15, 8,35)]
mid = [(-10,15, 8,20), (10,-15, 8,20), (20,-20, 8,15), (-20,20, 8,15)]
wh = [(-25,0, 10,20,10), (25,0, 10,20,10)]

# 3. Generate the XML layout dynamically
for i, (x, y) in enumerate(houses):
    world_content += f'<include><uri>{home}/.gz/models/house_2</uri><name>h_{i}</name><pose>{x} {y} 0 0 0 0</pose></include>\n'
    world_content += f'<model name="g_{i}"><static>true</static><pose>{x} {y} 0.05 0 0 0</pose><link name="l"><visual name="v"><geometry><box><size>12 12 0.1</size></box></geometry><material><ambient>0.2 0.6 0.2 1</ambient><diffuse>0.2 0.6 0.2 1</diffuse></material></visual></link></model>\n'

for i, (x, y) in enumerate(trees):
    uri = f"{home}/.gz/models/Oak tree" if i % 2 == 0 else f"{home}/.gz/models/Pine Tree"
    world_content += f'<include><uri>{uri}</uri><name>t_{i}</name><pose>{x} {y} 0 0 0 0</pose></include>\n'

for i, (x, y) in enumerate(cars):
    world_content += f'<include><uri>{home}/.gz/models/Hatchback copy</uri><name>c_{i}</name><pose>{x} {y} 0 0 0 1.57</pose></include>\n'

for i, (x, y, w, h) in enumerate(sky):
    world_content += f'<model name="sky_{i}"><static>true</static><pose>{x} {y} {h/2} 0 0 0</pose><link name="link"><collision name="c"><geometry><box><size>{w} {w} {h}</size></box></geometry></collision><visual name="v"><geometry><box><size>{w} {w} {h}</size></box></geometry><material><ambient>0.25 0.25 0.25 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material></visual></link></model>\n'

for i, (x, y, w, h) in enumerate(mid):
    world_content += f'<model name="mid_{i}"><static>true</static><pose>{x} {y} {h/2} 0 0 0</pose><link name="link"><collision name="c"><geometry><box><size>{w} {w} {h}</size></box></geometry></collision><visual name="v"><geometry><box><size>{w} {w} {h}</size></box></geometry><material><ambient>0.25 0.25 0.25 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material></visual></link></model>\n'

for i, (x, y, dx, dy, dz) in enumerate(wh):
    world_content += f'<model name="wh_{i}"><static>true</static><pose>{x} {y} {dz/2} 0 0 0</pose><link name="link"><collision name="c"><geometry><box><size>{dx} {dy} {dz}</size></box></geometry></collision><visual name="v"><geometry><box><size>{dx} {dy} {dz}</size></box></geometry><material><ambient>0.25 0.25 0.25 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material></visual></link></model>\n'

world_content += "  </world>\n</sdf>"

with open(world_path, 'w') as f:
    f.write(world_content)
print("✓ Wide City with Grass and Spaces Generated!")
EOF
