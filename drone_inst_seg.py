import json
import os
import time

import cv2
import pygame
import carla
import numpy as np
import math

from tqdm import tqdm

# Initialize Pygame
pygame.init()

# Set up display
display_width = 1850
display_height = 1000
display = pygame.display.set_mode((display_width, display_height))
pygame.display.set_caption("CARLA Drone Sim")

clock = pygame.time.Clock()

# Connect to CARLA server
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
map_index = 3
available_maps = ["Town01", "Town02", "Town03", "Town04", "Town05", "Town06", "Town07", "Town10HD"]
map_name = available_maps[map_index]
client.load_world(map_name)

# Get the blueprint library
blueprint_library = world.get_blueprint_library()

# Filter and print all static props
print("Static Props:")
for prop in [bp.id for bp in blueprint_library if 'static.prop.drone' in bp.id]:
    print(prop)

# Spawn the drone
drone_bp = blueprint_library.find('static.prop.drone_fiveinch')
drone_initial_transform = carla.Transform(carla.Location(x=0, y=0, z=100))
drone = world.try_spawn_actor(drone_bp, drone_initial_transform)

# Create a semantic segmentation camera sensor
# Updated Blueprint for semantic segmentation camera
drone_camera_bp_seg = blueprint_library.find('sensor.camera.semantic_segmentation')
drone_camera_bp_seg.set_attribute('image_size_x', str(display_width))
drone_camera_bp_seg.set_attribute('image_size_y', str(display_height))
drone_camera_bp_seg.set_attribute('fov', '110')

# Attach the camera to the drone
close_camera_position = carla.Location(x=-3, y=0, z=1)
far_camera_position = carla.Location(x=-10, y=0, z=2)
drone_camera_relative_transform = carla.Transform(far_camera_position)
drone_camera = world.try_spawn_actor(drone_camera_bp_seg, drone_camera_relative_transform, attach_to=drone)

# Drone control variables
drone_speed = 30
drone_rotation_speed = 30

def move_drone(drone, keys, delta_time):
    transform = drone.get_transform()
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

    drone.set_transform(carla.Transform(location, rotation))


# Semantic Segmentation Label Colors
# These colors correspond to different semantic classes in CARLA
LABEL_COLORS = np.array([
    (0, 0, 0),        # 0: None
    (70, 70, 70),     # 1: Buildings
    (190, 153, 153),  # 2: Fences
    (250, 170, 160),  # 3: Other
    (220, 20, 60),    # 4: Pedestrians
    (153, 153, 153),  # 5: Poles
    (157, 234, 50),   # 6: RoadLines
    (128, 64, 128),   # 7: Roads
    (244, 35, 232),   # 8: Sidewalks
    (107, 142, 35),   # 9: Vegetation
    (0, 0, 142),      # 10: Vehicles
    (102, 102, 156),  # 11: Walls
    (220, 220, 0),    # 12: TrafficSigns
    (70, 130, 180),   # 13: Sky
    (81, 0, 81),      # 14: Ground
    (150, 100, 100),  # 15: Bridge
    (230, 150, 140),  # 16: RailTrack
    (180, 165, 180),  # 17: GuardRail
    (250, 170, 30),   # 18: TrafficLight
    (110, 190, 160),  # 19: Static
    (170, 120, 50),   # 20: Dynamic
    (45, 60, 150),    # 21: Water
    (145, 170, 100),  # 22: Terrain
    (70, 70, 70),     # 1: Buildings
    (190, 153, 153),  # 2: Fences
    (250, 170, 160),  # 3: Other
    (220, 20, 60),    # 4: Pedestrians
    (153, 153, 153),  # 5: Poles
    (157, 234, 50),   # 6: RoadLines
    (128, 64, 128),   # 7: Roads
    (244, 35, 232),   # 8: Sidewalks
    (107, 142, 35),   # 9: Vegetation
    (0, 0, 142),      # 10: Vehicles
    (102, 102, 156),  # 11: Walls
    (220, 220, 0),    # 12: TrafficSigns
    (70, 130, 180),   # 13: Sky
    (81, 0, 81),      # 14: Ground
    (150, 100, 100),  # 15: Bridge
    (230, 150, 140),  # 16: RailTrack
    (180, 165, 180),  # 17: GuardRail
    (250, 170, 30),   # 18: TrafficLight
    (110, 190, 160),  # 19: Static
    (170, 120, 50),   # 20: Dynamic
    (45, 60, 150),    # 21: Water
    (145, 170, 100),  # 22: Terrain
])

# Callback to capture and process semantic segmentation images
image = None
def process_image(data):
    global image
    # Convert raw data to numpy array
    array = np.frombuffer(data.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (data.height, data.width, 4))
    # Only the red channel contains the class information
    segmentation_array = array[:, :, 2]

    # Map class IDs to colors
    color_segmentation = LABEL_COLORS[segmentation_array]
    
    # Store processed image
    image = color_segmentation

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
        pose_label = font.render(
            f"x: {location.x:.1f}, y: {location.y:.1f}, z: {location.z:.1f}, "
            f"roll: {rotation.roll:.0f}, pitch: {rotation.pitch:.0f}, Yaw: {rotation.yaw:.0f}",
            True, (255, 255, 255)
        )
        display.blit(pose_label, (800, display_height - 50))

        pygame.display.flip()
        clock.tick(30)

finally:
    # Clean up
    drone_camera.stop()
    drone_camera.destroy()
    drone.destroy()
    pygame.quit()
