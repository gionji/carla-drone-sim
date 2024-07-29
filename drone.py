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

# get the bluprint library
blueprint_library = world.get_blueprint_library()

# Filter and print all static props
print("Static Props:")
for prop in [bp.id for bp in blueprint_library if 'static.prop.drone' in bp.id]:
    print(prop)

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

# Create and configure the LIDAR sensor
lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('upper_fov', '15')
lidar_bp.set_attribute('lower_fov', '-30')
lidar_bp.set_attribute('rotation_frequency', '10')
lidar_bp.set_attribute('points_per_second', '50000')
lidar_bp.set_attribute('range', '100')

# Attach the LIDAR to the drone
lidar_position = carla.Location(x=0, y=0, z=0.2)
lidar_transform = carla.Transform(lidar_position)
lidar = world.try_spawn_actor(lidar_bp, lidar_transform, attach_to=drone)

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
#drone_camera.listen(process_image)



# LIDAR data processing callback
def process_lidar_data(data):
    points = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)
    # Project points to 2D
    if points.size > 0:
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        intensity = points[:, 3]
        
        # Normalize intensity for visualization
        intensity = np.clip(intensity, 0, 1) * 255
        intensity = intensity.astype(np.uint8)

        # Create a blank image and draw points
        image = np.zeros((display_height, display_width, 3), dtype=np.uint8)
        for (px, py, pz, inten) in zip(x, y, z, intensity):
            if px > 0 and px < display_width and py > 0 and py < display_height:
                color = (inten, inten, inten)
                image[int(py), int(px)] = color
        
        # Display the image
        surface = pygame.surfarray.make_surface(image.swapaxes(0, 1))
        display.blit(surface, (0, 0))

# Attach the callback to the LIDAR sensor
lidar.listen(process_lidar_data)




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

    lidar.stop()
    lidar.destroy()

    drone.destroy()
    pygame.quit()
