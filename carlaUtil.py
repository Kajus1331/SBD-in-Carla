import math
import carla
import random
import time
import logging
from typing import Optional, List, Tuple

# set up logging
logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

# INFO
# Lanes for motorway 3,4,5,6 and -3,-4,-5,-6
# Lanes for offramp 2 and -2
# Lanes for town 1

def setupCarla():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    world = client.load_world('Town04_Opt')
    blueprint_library = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()
    traffic_manager = client.get_trafficmanager()
    tm_port = traffic_manager.get_port()

    spectator = world.get_spectator()
    transform = spectator.get_transform()
    rotation = transform.rotation
    new_location = carla.Location(x=-65.0, y=205.0, z=90.0)
    new_transform = carla.Transform(new_location, rotation)
    spectator.set_transform(new_transform)

    traffic_manager.set_synchronous_mode(True)
    traffic_manager.global_percentage_speed_difference(0)
    return client, world, blueprint_library, spawn_points, traffic_manager, tm_port

def spawnDummy(blueprint_library, spawn_points, world, position):
    # vehicle blueprint
    # dummy_vehicle_bp = blueprint_library.filter('vehicle.dodge.charger_2020')[0]
    dummy_vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
    # spawning vehicles
    #dummy_spawn_point = spawn_points[118]
    # 173
    dummy_spawn_point = spawn_points[position]

    dummy_vehicle = world.try_spawn_actor(dummy_vehicle_bp, dummy_spawn_point)
    return dummy_vehicle

def cleanup(world):
    """Clean up the CARLA environment by destroying all actors."""
    if world is not None:
        # get all actors
        actor_list = world.get_actors()
        
        # filter for vehicles and sensors
        vehicles = actor_list.filter('vehicle.*')
        sensors = actor_list.filter('sensor.*')
        
        # destroy all vehicles and sensors
        for actor in vehicles:
            actor.destroy()
        for actor in sensors:
            actor.destroy()
            
        logging.info('Cleaned up all actors')
        
        # reset to asynchronous mode
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)

import math

# visualize road segments with alternating lane colors
def visualize_road_segments_with_lane_colors(world, duration=300.0):
    debug = world.debug
    carla_map = world.get_map()
    
    debug.draw_point(carla.Location(0, 0, 0), size=0.1, color=carla.Color(0, 0, 0, 0), life_time=0.01)
    
    topology = carla_map.get_topology()
    
    lane_segments = {}
    for segment in topology:
        start_waypoint, _ = segment
        lane_id = start_waypoint.lane_id
        if lane_id not in lane_segments:
            lane_segments[lane_id] = []
        lane_segments[lane_id].append(segment)

    colors = [carla.Color(255, 0, 0, 255), carla.Color(0, 0, 255, 255)]  # Red and Blue

    for lane_id, segments in lane_segments.items():
        color_index = 0  

        for segment in segments:
            start_waypoint, end_waypoint = segment
            start_location = start_waypoint.transform.location
            end_location = end_waypoint.transform.location

            road_id = start_waypoint.road_id

            segment_length = math.sqrt(
                (end_location.x - start_location.x) ** 2 +
                (end_location.y - start_location.y) ** 2 +
                (end_location.z - start_location.z) ** 2
            )
            
            debug.draw_line(
                start_location,
                end_location,
                thickness=0.3,
                color=colors[color_index],
                life_time=duration
            )
            
            color_index = (color_index + 1) % 2

            mid_x = (start_location.x + end_location.x) / 2
            mid_y = (start_location.y + end_location.y) / 2
            mid_z = (start_location.z + end_location.z) / 2 + 1.0 
            
            road_lane_text = f"Road: {road_id}, Lane: {lane_id}, Length: {segment_length:.1f} m"
            debug.draw_string(
                carla.Location(mid_x, mid_y, mid_z),
                road_lane_text,
                color=carla.Color(255, 165, 0, 255),
                life_time=duration
            )

    logging.info(f'Visualized road segments with alternating colors for {len(lane_segments)} lanes.')

# visualize spawn points with markers and additional information
def visualize_spawn_points(world, spawn_points, duration=1000.0):
    debug = world.debug
    map = world.get_map()
    
    debug.draw_point(carla.Location(0,0,0), size=0.1, color=carla.Color(0,0,0,0), life_time=0.01)
    
    for i, spawn_point in enumerate(spawn_points):
        location = spawn_point.location
    
        line_length = 1.0  
        z_offset = 2.5    

        waypoint = map.get_waypoint(location, project_to_road=True)
        lane_id = waypoint.lane_id
        road_id = waypoint.road_id
        
        debug.draw_line(
            carla.Location(x=location.x - line_length/2, y=location.y - line_length/2, z=location.z + z_offset),
            carla.Location(x=location.x + line_length/2, y=location.y + line_length/2, z=location.z + z_offset),
            thickness=0.2,
            color=carla.Color(255, 0, 0, 255),
            life_time=duration
        )
        debug.draw_line(
            carla.Location(x=location.x - line_length/2, y=location.y + line_length/2, z=location.z + z_offset),
            carla.Location(x=location.x + line_length/2, y=location.y - line_length/2, z=location.z + z_offset),
            thickness=0.2,
            color=carla.Color(255, 0, 0, 255),
            life_time=duration
        )
        
        debug.draw_string(
            carla.Location(x=location.x, y=location.y, z=location.z + z_offset + 1),
            str(i),
            color=carla.Color(0, 255, 0, 255),
            life_time=duration
        )
        
        coord_text = f'({location.x:.1f}, {location.y:.1f}, {location.z:.1f})'
        debug.draw_string(
            carla.Location(x=location.x, y=location.y, z=location.z + z_offset + 0.5),
            coord_text,
            color=carla.Color(0, 255, 255, 255),
            life_time=duration
        )

        lane_road_text = f'Lane: {lane_id}, Road: {road_id}'
        debug.draw_string(
            carla.Location(x=location.x, y=location.y, z=location.z + z_offset + 1.5),
            lane_road_text,
            color=carla.Color(255, 165, 0, 255),
            life_time=duration
        )
        
        arrow_length = 2.0
        end_point = carla.Location(
            x=location.x + arrow_length * math.cos(math.radians(spawn_point.rotation.yaw)),
            y=location.y + arrow_length * math.sin(math.radians(spawn_point.rotation.yaw)),
            z=location.z + z_offset
        )
        debug.draw_arrow(
            carla.Location(x=location.x, y=location.y, z=location.z + z_offset),
            end_point,
            thickness=0.2,
            arrow_size=0.2,
            color=carla.Color(255, 255, 0, 255),
            life_time=duration
        )
    
    logging.info(f'Visualized {len(spawn_points)} spawn points')

def green_traffic_lights(world):
    list_actor = world.get_actors()
    for actor_ in list_actor:
        if isinstance(actor_, carla.TrafficLight):
            actor_.set_state(carla.TrafficLightState.Green) 
            actor_.set_green_time(1000.0)