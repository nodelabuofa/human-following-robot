#!/usr/bin/env python3

import cv2
import numpy as np
from dataclasses import dataclass

### CONFIG: used by both finder.py and controller.py
@dataclass # so I don't have to instantiate line by line in finder.py
class CameraConfig:
    # intrinsic ZED mini camera parameters 
    # found using bash command rostopic echo /zedm/zed_node/left/camera_info
    f: float = 754.0        # x/y focal length IN PIXELS
    rho: float = 0.000002   # physical pixel sensor size (in meters)
    cx: int = 473           # 'principal point' X (not exactly midpoint, not sure how this is calculated)
    cy: int = 267           # 'principal point' Y
    height: int = 540       # image resolution height (smaller number!)
    width: int = 960        # image resolution weight

@dataclass
class ControllerConfig:
    proportional_gain: float = 0.5
    integral_gain: float = 0.3
    steering_gain: float = 1.0
    throttle_gain: float = -1.0
    max_angular_velocity: float = 6.0
    vehicle_length: float = 0.1778  # wheel center to center
    wheelbase: float = 0.1667       # rear tires, tread center to center
    integral_history_size: int = 3 # frames to keep in history

# shared ArUco 'reference'/ground truth
DESIRED_CORNERS = {
    0: {'u': -150.0, 'v': 200.0},  # top left
    1: {'u': 150.0,  'v': 200.0},  # top right
    2: {'u': 130.0,  'v': -85.0},  # bottom right
    3: {'u': -130.0, 'v': -85.0}   # bottom left
}



### VISUAL ERROR CONTROLLER

def compute_interaction_matrix(error_param, depth_param, camera_config_param: CameraConfig):
    
    L = np.zeros((2, 6), dtype=np.float32)

    f = camera_config_param.f
    rho = camera_config_param.rho
    Z = depth_param
    u_error = error_param[0]
    v_error = error_param[1]

    # First row
    L[0, 0] = -f / Z
    L[0, 1] = 0
    L[0, 2] = u_error / Z
    L[0, 3] = (u_error * v_error) / f
    L[0, 4] = -f * rho  - ((u_error ** 2) / f)
    L[0, 5] = v_error

    # Second row
    L[1, 0] = 0
    L[1, 1] = -f / Z
    L[1, 2] = v_error / Z
    L[1, 3] = f * rho + ((v_error ** 2) / f)
    L[1, 4] = -(u_error * v_error) / f
    L[1, 5] = -u_error

    interaction_matrix = L
    
    return interaction_matrix

def compute_unpack_errors(
            i_param, aruco_corners_array_param, desired_corners_param, 
            camera_config_param: CameraConfig):
    
    # if corner isn't detected, return zero error and skip it
    if len(aruco_corners_array_param) < (i_param + 1) * 3:
        return np.zeros(2, dtype=np.float32), 0.0, True

    u_0 = aruco_corners_array_param[i_param*3 + 0]
    v_0 = aruco_corners_array_param[i_param*3 + 1]
    Z = aruco_corners_array_param[i_param*3 + 2]

    # flipping so origin at bottom right of visual feed
    v_flipped = camera_config_param.height - v_0

    # convert from origin at bottom left to origin at center
    u = u_0 - camera_config_param.cx
    v = v_flipped - camera_config_param.cy

    u_desired = desired_corners_param[i_param]['u']
    v_desired = desired_corners_param[i_param]['v']

    u_error = u - u_desired
    v_error = v - v_desired

    error = np.array([u_error, v_error], dtype=np.float32) # error vector for this corner
    
    if Z == 0 or np.isnan(Z) or np.isinf(Z) or Z <= 0.1:
        skip = True
    else: 
        skip = False

    return error, Z, skip

def differential_wheel_speed(v_twist_param, updated_twist_param, controller_config_param):
    throttle_gain = controller_config_param.throttle_gain
    steering_gain = controller_config_param.steering_gain
    vehicle_length = controller_config_param.vehicle_length
    wheelbase = controller_config_param.wheelbase

    if abs(v_twist_param[2]) < 1e-2: # prevents division by zero and huge steering values
        updated_twist_param.angular.y = 0.0
    else:
        # clamp angular velocity
        if v_twist_param[4] > 6: # max angular velocity of kinematic bicycle model v/L, max lin velocity is 1 m/s, wheel to wheel length 0.1778, so max omega approx 6
            v_twist_param[4] = 6
        if v_twist_param[4] < -6:
            v_twist_param[4] = -6

        # convert angular velocity to steering angle
        updated_twist_param.angular.y = float(np.arctan(v_twist_param[4] * 0.1778 / v_twist_param[2])) * steering_gain

    # linear.x is left wheel, linear.y is right wheel
    # left is positive radian max, right is negative radian max
    # 0.1778 is vehicle length (wheel center to center), 0.1667 wheelbase (rear tires, tread center to center)
    if updated_twist_param.angular.y == 0:
        # z axis portrudes perpendicular from lens, so z from computed twist is desired linear velocity of end effector/camera
        updated_twist_param.linear.x = float(v_twist_param[2]) * throttle_gain
        updated_twist_param.linear.y = float(v_twist_param[2]) * throttle_gain
    else:
        turning_radius = vehicle_length / np.tan(updated_twist_param.angular.y) # 0.
        ackerman_differential_ratio = (abs(turning_radius) - (wheelbase/2)) / (abs(turning_radius) + (wheelbase/2))
        
        if turning_radius > 0: # turning left
            updated_twist_param.linear.y = float(v_twist_param[2] * throttle_gain) # outer (right) wheel has max velocity
            updated_twist_param.linear.x = float(float(v_twist_param[2]) * ackerman_differential_ratio * throttle_gain)
        else: # turning right
            updated_twist_param.linear.x = float(v_twist_param[2] * throttle_gain) # outer (left) wheel has max velocity
            updated_twist_param.linear.y = float(float(v_twist_param[2]) * ackerman_differential_ratio * throttle_gain)

    return updated_twist_param


### ARUCO FINDER
def get_aruco_parameters():    # instantiate weird aruco param object outside
    params = cv2.aruco.DetectorParameters_create()
    
    # 'the comb' setting
    params.adaptiveThreshWinSizeMin = 3  # most fine toothed comb
    params.adaptiveThreshWinSizeMax = 100 # most wide toothed comb
    params.adaptiveThreshWinSizeStep = 10 # step size of combs in between
    params.adaptiveThreshConstant = 2     # i forget
    
    # filtering size
    params.minMarkerPerimeterRate = 0.01 # not sure exactly how calculated but this setting works
    params.maxMarkerPerimeterRate = 4.0
    
    # refining image
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX 
    params.cornerRefinementWinSize = 15
    params.cornerRefinementMaxIterations = 50  # scans 50 times
    params.cornerRefinementMinAccuracy = 0.01  
    
    return params

def centered_to_corner_origin(desired_corners_params, config: CameraConfig):
    # converts from centered, y-up coordinate frame for visual servoing to top-left origin, y-down coordinate frame for plotting

    absolute_desired_corners = []
    for i in range(4): #0, 1, 2, 3
        u = desired_corners_params[i]['u']
        v = desired_corners_params[i]['v']

        u_absolute = int(u + config.cx)
        v_absolute = int(config.height - (v + config.cy))
        absolute_desired_corners.append([u_absolute, v_absolute])

    absolute_desired_corners_array = np.array(absolute_desired_corners, np.int32)
    absolute_desired_corners_array = absolute_desired_corners_array.reshape((-1, 1, 2))

    return absolute_desired_corners_array

def package_corners(corners_param, depth_image_param):
    flat_corners_list = [] # so doesn't return None, funky things happen
    for index, corners in enumerate(corners_param): # 'corners_param' weird shape, (1, 4, 2), need to unpack 
        corner_coordinates = corners.reshape((4, 2))
        corner_depths_array = [] # will look like [(x1, y1, d1), ... , (x4, y4, d4)]

        for corner in corner_coordinates:
            x, y = int(corner[0]), int(corner[1])
            d = float(depth_image_param[y, x]) # (y,x) because (row, column)
            corner_depths_array.append((x,y,d))

        # flatten (x,y) coordinates to single list [x1, y1, x2, y2, x3, y3, x4, y4]
        flat_corners = np.array(corner_depths_array).flatten()
        flat_corners_list = flat_corners.tolist()

    return flat_corners_list

def annotate_image(color_image_param, corners_param, ids_param, absolute_desired_corners_param):

    # make green rectanglish around where ArUco code should be
    cv2.polylines(color_image_param, [absolute_desired_corners_param], isClosed=True, color=(0, 255, 0), thickness=2)
    
    # thinner rectanglish around where ArUco code is
    if ids_param is not None and len(ids_param) > 0:
        cv2.aruco.drawDetectedMarkers(color_image_param, corners_param, ids_param)
    
    return color_image_param

        
   