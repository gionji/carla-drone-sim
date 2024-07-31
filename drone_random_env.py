import json
import os
import time

import cv2
import pygame
import carla
import numpy as np
import math

from tqdm import tqdm

def load_map_from_xodr(client, xodr_path = "vasteras_01.xodr"):
    od_file = open(xodr_path)
    data = od_file.read()

    # Load the opendrive map
    vertex_distance = 2.0  # in meters
    max_road_length = 50.0 # in meters
    wall_height = 1.0      # in meters
    extra_width = 0.6      # in meters
    world = client.generate_opendrive_world(
        data, carla.OpendriveGenerationParameters(
            vertex_distance=vertex_distance,
            max_road_length=max_road_length,
            wall_height=wall_height,
            additional_width=extra_width,
            smooth_junctions=True,
            enable_mesh_visibility=True)
        )
    
    return world


import random
import carla
import math

def spawn_props_along_track(world, blueprint_library, distance_from_lane=5.0, prop_spacing=20.0):
    """
    Spawns random static props along the sides of the track based on the given XODR file.

    Parameters:
    - world: carla.World, the CARLA simulation world.
    - blueprint_library: carla.BlueprintLibrary, library to access props.
    - distance_from_lane: float, distance from the track's centerline to place the props.
    - prop_spacing: float, spacing between consecutive props.
    """
    
    # Get all waypoints along the centerline of the road
    map = world.get_map()
    waypoints = map.generate_waypoints(distance=prop_spacing)
    
    # Get static prop blueprints
    prop_blueprints = [bp for bp in blueprint_library if 'static.prop' in bp.id]

    # Function to calculate left and right offsets
    def get_offset_location(waypoint, distance):
        left_vector = carla.Vector3D(-math.sin(math.radians(waypoint.transform.rotation.yaw)),
                                     math.cos(math.radians(waypoint.transform.rotation.yaw)), 0)
        right_vector = carla.Vector3D(math.sin(math.radians(waypoint.transform.rotation.yaw)),
                                      -math.cos(math.radians(waypoint.transform.rotation.yaw)), 0)
        
        left_location = waypoint.transform.location + left_vector * distance
        right_location = waypoint.transform.location + right_vector * distance
        
        return left_location, right_location

    # Iterate over the waypoints and spawn props
    for waypoint in waypoints:
        # Calculate spawn positions
        left_location, right_location = get_offset_location(waypoint, distance_from_lane)

        # Choose random props for left and right sides
        left_prop_bp = random.choice(prop_blueprints)
        right_prop_bp = random.choice(prop_blueprints)

        # Create transforms for props
        left_transform = carla.Transform(left_location, waypoint.transform.rotation)
        right_transform = carla.Transform(right_location, waypoint.transform.rotation)

        # Attempt to spawn props
        left_prop = world.try_spawn_actor(left_prop_bp, left_transform)
        right_prop = world.try_spawn_actor(right_prop_bp, right_transform)
        
        # Print success message if spawn was successful
        if left_prop is not None:
            print(f"Spawned {left_prop.type_id} at {left_location}")
        if right_prop is not None:
            print(f"Spawned {right_prop.type_id} at {right_location}")


import random
import carla
import math

def spawn_props_near_waypoints(
    world,
    blueprint_library,
    num_objects_per_waypoint=3,
    distance_from_lane=5.0,
    random_range=3.0,
    prop_spacing=20.0
):
    """
    Spawns random static props near the waypoints along the track.

    Parameters:
    - world: carla.World, the CARLA simulation world.
    - blueprint_library: carla.BlueprintLibrary, library to access props.
    - num_objects_per_waypoint: int, number of objects to spawn per waypoint.
    - distance_from_lane: float, distance from the track's centerline to start spawning props.
    - random_range: float, random range for x and y plane positioning.
    - prop_spacing: float, spacing between waypoints for spawning props.

    Returns:
    - A list of spawned actors for later cleanup.
    """
    
    # Get all waypoints along the centerline of the road
    map = world.get_map()
    waypoints = map.generate_waypoints(distance=prop_spacing)
    
    # Get static prop blueprints
    prop_blueprints = [bp for bp in blueprint_library if 'static.prop' in bp.id]

    # List to keep track of spawned props
    spawned_props = []

    # Function to calculate offsets
    def get_offset_location(waypoint, distance, random_range):
        # Calculate left and right offset vectors
        left_vector = carla.Vector3D(
            -math.sin(math.radians(waypoint.transform.rotation.yaw)),
            math.cos(math.radians(waypoint.transform.rotation.yaw)),
            0
        )
        right_vector = carla.Vector3D(
            math.sin(math.radians(waypoint.transform.rotation.yaw)),
            -math.cos(math.radians(waypoint.transform.rotation.yaw)),
            0
        )

        # Calculate left and right base locations
        left_location = waypoint.transform.location + left_vector * distance
        right_location = waypoint.transform.location + right_vector * distance

        # Add random offset within the defined range
        random_offset_left = carla.Vector3D(
            random.uniform(-random_range, random_range),
            random.uniform(-random_range, random_range),
            0
        )
        random_offset_right = carla.Vector3D(
            random.uniform(-random_range, random_range),
            random.uniform(-random_range, random_range),
            0
        )
        
        left_location += random_offset_left
        right_location += random_offset_right
        
        return left_location, right_location

    # Iterate over the waypoints and spawn props
    for waypoint in waypoints:
        # Only spawn props on driving lanes
        if waypoint.lane_type == carla.LaneType.Driving:
            # Check if there's no left lane (leftmost lane)
            left_lane = waypoint.get_left_lane()
            if left_lane is None or left_lane.lane_type != carla.LaneType.Driving:
                # Spawn props to the left
                for _ in range(num_objects_per_waypoint):
                    left_location, _ = get_offset_location(waypoint, distance_from_lane, random_range)
                    left_prop_bp = random.choice(prop_blueprints)
                    left_transform = carla.Transform(left_location, waypoint.transform.rotation)
                    left_prop = world.try_spawn_actor(left_prop_bp, left_transform)
                    if left_prop is not None:
                        spawned_props.append(left_prop)
                        print(f"Spawned {left_prop.type_id} at {left_location}")
            
            # Check if there's no right lane (rightmost lane)
            right_lane = waypoint.get_right_lane()
            if right_lane is None or right_lane.lane_type != carla.LaneType.Driving:
                # Spawn props to the right
                for _ in range(num_objects_per_waypoint):
                    _, right_location = get_offset_location(waypoint, distance_from_lane, random_range)
                    right_prop_bp = random.choice(prop_blueprints)
                    right_transform = carla.Transform(right_location, waypoint.transform.rotation)
                    right_prop = world.try_spawn_actor(right_prop_bp, right_transform)
                    if right_prop is not None:
                        spawned_props.append(right_prop)
                        print(f"Spawned {right_prop.type_id} at {right_location}")
    
    return spawned_props

def destroy_spawned_props(spawned_props):
    """
    Destroys all spawned props.

    Parameters:
    - spawned_props: list of carla.Actor, the spawned props to destroy.
    """
    for prop in spawned_props:
        prop.destroy()
        print(f"Destroyed {prop.type_id}")



# Initialize Pygame
pygame.init()

# Set up display
display_width = 1850
display_height = 1000
display = pygame.display.set_mode((display_width, display_height))
racetrack = 'test_track_01.xodr'
pygame.display.set_caption("CARLA Drone Sim")

clock = pygame.time.Clock()

# Connect to CARLA server
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
map_index = 3
available_maps = ["Town01", "Town02", "Town03", "Town04", "Town05", "Town06", "Town07", "Town10HD"]
map_name = available_maps[map_index]

if racetrack is None:
    client.load_world(map_name)

else: 
    world = load_map_from_xodr(client, xodr_path = racetrack)
    map = world.get_map()

# get the bluprint library
blueprint_library = world.get_blueprint_library()

# Example of using the function
#spawn_props_along_track(world, blueprint_library, distance_from_lane=5.0, prop_spacing=20.0)

# Assuming the CARLA world and blueprint library have been set up previously
spawned_props = spawn_props_near_waypoints(
    world,
    blueprint_library,
    num_objects_per_waypoint=3,
    distance_from_lane=5.0,
    random_range=3.0,
    prop_spacing=20.0
)


# Filter and print all static props
#print("Static Props:")
#for prop in [bp.id for bp in blueprint_library if 'static.prop.drone' in bp.id]:
#    print(prop)

# Spawn the trashbin
drone_bp = blueprint_library.find('static.prop.drone_fiveinch')
drone_initial_transform = carla.Transform(carla.Location(x=0, y=0, z=10))
drone = world.try_spawn_actor(drone_bp, drone_initial_transform)

# Create a camera sensor
drone_camera_bp_rgb = blueprint_library.find('sensor.camera.rgb')
drone_camera_bp_rgb.set_attribute('image_size_x', str(display_width))
drone_camera_bp_rgb.set_attribute('image_size_y', str(display_height))
drone_camera_bp_rgb.set_attribute('fov', '110')

# Attach the camera to the trashbin
close_camera_position = carla.Location(x=-3, y=0, z=1)
far_camera_position = carla.Location(x=-10, y=0, z=2)
drone_camera_relative_transform = carla.Transform( far_camera_position )
drone_camera = world.try_spawn_actor(drone_camera_bp_rgb, drone_camera_relative_transform, attach_to=drone)


# Trashbin control variables
drone_speed = 30
drone_rotation_speed = 30


def move_drone(trashbin, keys, delta_time):
    transform = trashbin.get_transform()
    location = transform.location
    rotation = transform.rotation

    # Calculate forward and right vectors
    yaw = math.radians(rotation.yaw)
    forward_vector = carla.Vector3D(math.cos(yaw), math.sin(yaw), 0)
    right_vector = carla.Vector3D(-math.sin(yaw), math.cos(yaw), 0)

    if keys[pygame.K_w]:
        location += forward_vector * drone_speed * delta_time
    if keys[pygame.K_s]:
        location -= forward_vector * drone_speed * delta_time
    if keys[pygame.K_a]:
        location -= right_vector * drone_speed * delta_time
    if keys[pygame.K_d]:
        location += right_vector * drone_speed * delta_time
    if keys[pygame.K_q]:
        location.z -= drone_speed * delta_time
    if keys[pygame.K_e]:
        location.z += drone_speed * delta_time

    if keys[pygame.K_y]:
        rotation.pitch += drone_rotation_speed * delta_time
    if keys[pygame.K_h]:
        rotation.pitch -= drone_rotation_speed * delta_time
    if keys[pygame.K_g]:
        rotation.yaw -= drone_rotation_speed * delta_time
    if keys[pygame.K_j]:
        rotation.yaw += drone_rotation_speed * delta_time

    trashbin.set_transform(carla.Transform(location, rotation))


saved_transform = None


# Callback to capture camera images
image = None
def process_image(data):
    global image
    array = np.frombuffer(data.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (data.height, data.width, 4))
    array = array[:, :, :3]
    image = array

# Attach the callback to the camera
drone_camera.listen(process_image)



try:
    # Main loop
    
    running = True
    while running:
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()

        if keys[pygame.K_r]:
            drone.set_transform(drone_initial_transform)

        else:
            move_drone(drone, keys, clock.get_time() / 1000.0)

        if image is not None:
            surface = pygame.surfarray.make_surface(image.swapaxes(0, 1))
            display.blit(surface, (0, 0))

        tf = drone.get_transform()
        location = tf.location
        rotation = tf.rotation

        font = pygame.font.Font(None, 36)
        pose_label = font.render(f"x: {location.x:.1f}, y: {location.y:.1f}, z:{location.z:.1f}, roll:{rotation.roll:.0f}, pitch:{rotation.pitch:.0f}, Yaw:{rotation.yaw:.0f}", True, (255, 255, 255))
        display.blit(pose_label, (800, display_height - 50))

        pygame.display.flip()
        clock.tick(30)

finally:
    # Clean up
    drone_camera.stop()
    drone_camera.destroy()

    # Later, when you want to clean up the spawned objects
    destroy_spawned_props(spawned_props)

    drone.destroy()
    pygame.quit()
