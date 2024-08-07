import json
import os
import random
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

def get_map_boundaries(world):
    waypoints = world.get_map().generate_waypoints(2.0)
    min_x, max_x = float('inf'), float('-inf')
    min_y, max_y = float('inf'), float('-inf')
    
    for waypoint in waypoints:
        location = waypoint.transform.location
        min_x = min(min_x, location.x)
        max_x = max(max_x, location.x)
        min_y = min(min_y, location.y)
        max_y = max(max_y, location.y)

    return min_x, max_x, min_y, max_y


def scale_to_window(x, y, min_x, max_x, min_y, max_y, window_width, window_height):
    # Scale the x and y coordinates to fit within the Pygame window
    scale_x = (x - min_x) / (max_x - min_x) * window_width
    scale_y = (y - min_y) / (max_y - min_y) * window_height
    return int(scale_x), int(scale_y)


def draw_map(display, vehicle, drone, min_x, max_x, min_y, max_y, map_width, map_height, window_width, window_height):
    # Draw a rectangle representing the map
    map_rect = pygame.Rect((window_width - map_width - 20, 20), (map_width, map_height))
    pygame.draw.rect(display, (50, 50, 50), map_rect, 2)

    # Get positions of the vehicle and drone
    vehicle_location = vehicle.get_transform().location
    drone_location = drone.get_transform().location

    # Scale positions to the map
    vehicle_pos = scale_to_window(vehicle_location.x, vehicle_location.y, min_x, max_x, min_y, max_y, map_width, map_height)
    drone_pos = scale_to_window(drone_location.x, drone_location.y, min_x, max_x, min_y, max_y, map_width, map_height)

    # Draw vehicle and drone positions on the map
    pygame.draw.circle(display, (0, 255, 0), (map_rect.left + vehicle_pos[0], map_rect.top + vehicle_pos[1]), 5)
    pygame.draw.circle(display, (255, 0, 0), (map_rect.left + drone_pos[0], map_rect.top + drone_pos[1]), 5)


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

# Spawn a vehicle
vehicle_bp = blueprint_library.filter('vehicle.*')[0]  # Choose a vehicle blueprint
spawn_points = world.get_map().get_spawn_points()

vehicle_spawn_transform = random.choice(spawn_points)  # Use the first spawn point for simplicity

vehicle = world.try_spawn_actor(vehicle_bp, vehicle_spawn_transform)
if vehicle is None:
    raise RuntimeError("Failed to spawn vehicle")

# Calculate drone's initial position 10 meters above the vehicle
vehicle_location = vehicle.get_transform().location
print(vehicle_location)

drone_initial_transform = carla.Transform(
    carla.Location(
        x=vehicle_location.x, 
        y=vehicle_location.y, 
        z=vehicle_location.z + 10  # 10 meters above the vehicle
    )
)
vehicle.set_autopilot(False)

# Filter and print all static props
print("Static Props:")
for prop in [bp.id for bp in blueprint_library if 'static.prop.drone' in bp.id]:
    print(prop)

# Spawn the trashbin
drone_bp = blueprint_library.find('static.prop.drone_fiveinch')
#drone_initial_transform = carla.Transform(carla.Location(x=0, y=0, z=100))
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


def move_drone_to_vehicle():
    vehicle_transform = vehicle.get_transform()
    drone_trasnform = carla.Transform(
        carla.Location(
            x=vehicle_transform.location.x, 
            y=vehicle_transform.location.y, 
            z=vehicle_transform.location.z + 10  # 10 meters above the vehicle
        ),
        carla.Rotation(
            yaw=vehicle_transform.rotation.yaw, 
            pitch=vehicle_transform.rotation.pitch -40 , 
            roll=vehicle_transform.rotation.roll
        )
    )
    drone.set_transform( drone_trasnform )


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

drone_autopilot = False
vehicle_autopilot = False

# Determine map boundaries once
min_x, max_x, min_y, max_y = get_map_boundaries(world)

# Define map rectangle size on Pygame window
map_width = 200
map_height = 200

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
            move_drone_to_vehicle()
            time.sleep(0.1)
        
        elif keys[pygame.K_x]:
            drone_autopilot = not drone_autopilot  
            time.sleep(0.1) 
        
        elif keys[pygame.K_z]:
            vehicle_autopilot = not vehicle_autopilot
            vehicle.set_autopilot( vehicle_autopilot )  
            time.sleep(0.1)         
        
        else:
            if not drone_autopilot:
                move_drone(drone, keys, clock.get_time() / 1000.0)
            
            else:
                move_drone_to_vehicle()

        # Vehicle control
        if not vehicle_autopilot:
            throttle = 0.0
            brake = 0.0
            steer = 0.0

            if keys[pygame.K_UP]:
                throttle = 1.0
            if keys[pygame.K_DOWN]:
                brake = 1.0
            if keys[pygame.K_LEFT]:
                steer = -1.0
            if keys[pygame.K_RIGHT]:
                steer = 1.0

            control = carla.VehicleControl(throttle=throttle, brake=brake, steer=steer)
            vehicle.apply_control(control)


        if image is not None:
            surface = pygame.surfarray.make_surface(image.swapaxes(0, 1))
            display.blit(surface, (0, 0))

        tf = drone.get_transform()
        location = tf.location
        rotation = tf.rotation

        font = pygame.font.Font(None, 36)
        pose_label = font.render(f"x: {location.x:.1f}, y: {location.y:.1f}, z:{location.z:.1f}, roll:{rotation.roll:.0f}, pitch:{rotation.pitch:.0f}, Yaw:{rotation.yaw:.0f}", True, (255, 255, 255))
        display.blit(pose_label, (800, display_height - 50))

        # Draw map with vehicle and drone positions
        draw_map(display, vehicle, drone, min_x, max_x, min_y, max_y, map_width, map_height, display_width, display_height)

        pygame.display.flip()
        clock.tick(30)

finally:
    # Clean up
    drone_camera.stop()
    drone_camera.destroy()
    drone.destroy()
    pygame.quit()
