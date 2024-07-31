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



def draw_bounding_box(display, actor, camera_transform, camera_fov, display_width, display_height, color=(255, 0, 0)):
    # Get the actor's bounding box
    bounding_box = actor.bounding_box

    # Transform the bounding box vertices to world coordinates
    world_vertices = bounding_box.get_world_vertices(actor.get_transform())

    # Camera intrinsic parameters
    fov = camera_fov
    focal = display_width / (2.0 * math.tan(float(fov) * math.pi / 360.0))
    intrinsic = np.array([[focal, 0, display_width / 2.0],
                          [0, focal, display_height / 2.0],
                          [0, 0, 1]])

    # Project the 3D bounding box vertices to 2D screen coordinates
    screen_points = []
    for vertex in world_vertices:
        vertex_cam = camera_transform.get_inverse_matrix() @ np.array([vertex.x, vertex.y, vertex.z, 1])
        vertex_cam = vertex_cam[:3] / vertex_cam[3]

        # Project vertex to 2D
        if vertex_cam[2] > 0:  # Ensure the vertex is in front of the camera
            vertex_img = intrinsic @ vertex_cam
            vertex_img /= vertex_img[2]

            # Add to the list of screen points
            screen_points.append((int(vertex_img[0]), int(vertex_img[1])))

    # Draw the edges of the bounding box
    if screen_points:
        for i in range(4):
            # Draw bottom face
            pygame.draw.line(display, color, screen_points[i], screen_points[(i + 1) % 4], 2)

            # Draw top face
            pygame.draw.line(display, color, screen_points[i + 4], screen_points[(i + 1) % 4 + 4], 2)

            # Connect bottom and top faces
   


def is_prop_in_view(camera, prop, display_width, display_height):
    # Get the camera and prop transforms
    camera_transform = camera.get_transform()
    prop_transform = prop.get_transform()

    # Transform the prop's bounding box vertices into the camera's space
    bbox = prop.bounding_box
    bbox_world_vertices = bbox.get_world_vertices(prop_transform)
    
    # Camera intrinsic parameters
    fov = camera.attributes['fov']
    focal = display_width / (2.0 * math.tan(float(fov) * math.pi / 360.0))
    intrinsic = np.array([[focal, 0, display_width / 2.0],
                          [0, focal, display_height / 2.0],
                          [0, 0, 1]])

    # Check each vertex
    for vertex in bbox_world_vertices:
        vertex_cam = camera_transform.get_inverse_matrix() @ np.array([vertex.x, vertex.y, vertex.z, 1])
        vertex_cam = vertex_cam[:3] / vertex_cam[3]

        # Check if the vertex is in front of the camera
        if vertex_cam[2] <= 0:
            continue

        # Project vertex to 2D
        vertex_img = intrinsic @ vertex_cam
        vertex_img /= vertex_img[2]

        # Check if the vertex is within the screen bounds
        if 0 <= vertex_img[0] < display_width and 0 <= vertex_img[1] < display_height:
            return True

    return False



def spawn_static_props(world, blueprint_library, num_props=10):
    # Choose a blueprint for the static prop (you can change this to any static prop you want)
    prop_blueprint = blueprint_library.filter('static.prop.*')

    # Retrieve map boundaries for random spawn locations
    map_boundaries = get_map_boundaries(world)
    min_x, max_x, min_y, max_y = map_boundaries

    # List to store spawned props
    spawned_props = []

    for _ in range(num_props):
        # Randomly generate a spawn location within the map boundaries
        spawn_location = carla.Location(
            x=random.uniform(min_x, max_x),
            y=random.uniform(min_y, max_y),
            z=0  # Props are usually on the ground level
        )
        spawn_transform = carla.Transform(spawn_location)

        # Randomly choose a blueprint for each prop
        blueprint = random.choice(prop_blueprint)

        # Try to spawn the prop
        prop = world.try_spawn_actor(blueprint, spawn_transform)

        print('Spawned asset at: ', spawn_location )

        if prop:
            spawned_props.append(prop)

    return spawned_props



def get_map_boundaries(world):
    # Function to determine the map boundaries based on the waypoints
    map = world.get_map()
    waypoints = map.generate_waypoints(2.0)  # 2.0 meters apart waypoints

    min_x = min(waypoint.transform.location.x for waypoint in waypoints)
    max_x = max(waypoint.transform.location.x for waypoint in waypoints)
    min_y = min(waypoint.transform.location.y for waypoint in waypoints)
    max_y = max(waypoint.transform.location.y for waypoint in waypoints)

    return min_x, max_x, min_y, max_y


def build_projection_matrix(w, h, fov, is_behind_camera=False):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)

    if is_behind_camera:
        K[0, 0] = K[1, 1] = -focal
    else:
        K[0, 0] = K[1, 1] = focal

    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K


def get_image_point(loc, K, w2c):
    # Calculate 2D projection of 3D coordinate

    # Format the input coordinate (loc is a carla.Position object)
    point = np.array([loc.x, loc.y, loc.z, 1])
    # transform to camera coordinates
    point_camera = np.dot(w2c, point)

    # New we must change from UE4's coordinate system to an "standard"
    # (x, y ,z) -> (y, -z, x)
    # and we remove the fourth componebonent also
    point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

    # now project 3D->2D using the camera matrix
    point_img = np.dot(K, point_camera)
    # normalize
    point_img[0] /= point_img[2]
    point_img[1] /= point_img[2]

    return point_img[0:2]


# Initialize Pygame
pygame.init()

# Set up display
display_width = 1850
display_height = 1000
camera_fov = 110
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

# Spawn the trashbin
drone_bp = blueprint_library.find('static.prop.drone_fiveinch')
#drone_initial_transform = carla.Transform(carla.Location(x=0, y=0, z=100))
drone = world.try_spawn_actor(drone_bp, drone_initial_transform)

# Create a camera sensor
drone_camera_bp_rgb = blueprint_library.find('sensor.camera.rgb')
drone_camera_bp_rgb.set_attribute('image_size_x', str(display_width))
drone_camera_bp_rgb.set_attribute('image_size_y', str(display_height))
drone_camera_bp_rgb.set_attribute('fov', str(camera_fov))

# Attach the camera to the trashbin
close_camera_position = carla.Location(x=-3, y=0, z=1)
far_camera_position = carla.Location(x=-10, y=0, z=2)
drone_camera_relative_transform = carla.Transform( far_camera_position )
drone_camera = world.try_spawn_actor(drone_camera_bp_rgb, drone_camera_relative_transform, attach_to=drone)


# Get the world to camera matrix
world_2_camera = np.array(drone_camera.get_transform().get_inverse_matrix())

# Get the attributes from the camera
image_w = drone_camera_bp_rgb.get_attribute("image_size_x").as_int()
image_h = drone_camera_bp_rgb.get_attribute("image_size_y").as_int()
fov = drone_camera_bp_rgb.get_attribute("fov").as_float()

# Calculate the camera projection matrix to project from 3D -> 2D
K = build_projection_matrix(image_w, image_h, fov)
K_b = build_projection_matrix(image_w, image_h, fov, is_behind_camera=True)

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

num_props = 10
spawned_props = spawn_static_props(world, blueprint_library, num_props)

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

        # Process and display camera image
        if image is not None:

            # Get the camera matrix 
            world_2_camera = np.array(drone_camera.get_transform().get_inverse_matrix())

            surface = pygame.surfarray.make_surface(image.swapaxes(0, 1))
            display.blit(surface, (0, 0))

            # Draw bounding boxes for visible props
            for prop in spawned_props:
                ;

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

    for prop in spawned_props:
        prop.destroy()

    pygame.quit()
