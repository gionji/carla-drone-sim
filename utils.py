

def get_bboxed_img(img, camera, world_2_camera):
    bb = npc.bounding_box
    dist = npc.get_transform().location.distance(vehicle.get_transform().location)

    # Filter for the vehicles within 50m
    if dist < 50:

    # Calculate the dot product between the forward vector
    # of the vehicle and the vector between the vehicle
    # and the other vehicle. We threshold this dot product
    # to limit to drawing bounding boxes IN FRONT OF THE CAMERA
        forward_vec = vehicle.get_transform().get_forward_vector()
        ray = npc.get_transform().location - vehicle.get_transform().location

        if forward_vec.dot(ray) > 0:
            p1 = get_image_point(bb.location, K, world_2_camera)
            verts = [v for v in bb.get_world_vertices(npc.get_transform())]
            x_max = -10000
            x_min = 10000
            y_max = -10000
            y_min = 10000

            for vert in verts:
                p = get_image_point(vert, K, world_2_camera)
                # Find the rightmost vertex
                if p[0] > x_max:
                    x_max = p[0]
                # Find the leftmost vertex
                if p[0] < x_min:
                    x_min = p[0]
                # Find the highest vertex
                if p[1] > y_max:
                    y_max = p[1]
                # Find the lowest  vertex
                if p[1] < y_min:
                    y_min = p[1]

            cv2.line(img, (int(x_min),int(y_min)), (int(x_max),int(y_min)), (0,0,255, 255), 1)
            cv2.line(img, (int(x_min),int(y_max)), (int(x_max),int(y_max)), (0,0,255, 255), 1)
            cv2.line(img, (int(x_min),int(y_min)), (int(x_min),int(y_max)), (0,0,255, 255), 1)
            cv2.line(img, (int(x_max),int(y_min)), (int(x_max),int(y_max)), (0,0,255, 255), 1)

