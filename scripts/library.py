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
    width: int = 960        # image resolution weigth

# shared ArUco 'reference'/ground truth
DESIRED_CORNERS = {
    0: {'u': -150.0, 'v': 200.0},  # top left
    1: {'u': 150.0,  'v': 200.0},  # top right
    2: {'u': 130.0,  'v': -85.0},  # bottom right
    3: {'u': -130.0, 'v': -85.0}   # bottom left
}



### ArUco Finder Functions
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

        
   