"""
ArUco Marker Detection Node
Detects ArUco markers and publishes corner positions to 'aruco-corners-topic'
"""
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ArucoCornersPub:
    def __init__(self): # constructor
        rospy.init_node('aruco_corners_pub')

        # ~ means publish 'locally' under this node, eg. aruco_corners_pub/aruco_dict

        # zed mini subscribers
        self.RGB_sub = rospy.Subscriber('/zedm/zed_node/left/image_rect_gray', Image, self.RGB_callback) 
  
        rospy.loginfo("Subscribed to ZED Mini RGB feed")

    def RGB_callback(self, msg):
        rospy.loginfo("Received an image message")
        rospy.loginfo(f"Image frame id: {msg.header.frame_id}, height: {msg.height}, width: {msg.width}")
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = ArucoCornersPub()
        node.run()
    except rospy.ROSInterruptException:
        pass


        







# import rospy
# import cv2
# import numpy as np
# from std_msgs.msg import Float32MultiArray
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import sys

# class ArucoCornersPub:
#     def __init__(self): # constructor
#         rospy.init_node('aruco_corners_pub', anonymous=True) # initialize node named aruco-corners-pub
        
#         # ~ means local thing to publish, ie. aruco-corners-pub/aruco_
#         self.aruco_dict_type = rospy.get_param('~aruco-dict', 'DICT_5X5_50') # specifies we're using 'library' of 5x5 bit (black/white) arucos, with IDs between 0-49 (first 50)
#         self.RGB_topic = rospy.get_param('~RGB-topic', '/zedm/zed_node/left/image_rect_gray') # subscribes to ZED Mini's visual RGB feed
#         self.depth_topic = rospy.get_param('~depth-topic', '/zedm/zed_node/depth/depth_registered') # subscribes to topic with correponsing 'latitudinal' depth along axis portruding from lens
#         self.target_marker_id = rospy.get_param('~target-marker-id', 0)  # which aruco marker to track
        
#         # Initialize OpenCV ArUco detector
#         self.aruco_dict = cv2.aruco.Dictionary_get(getattr(cv2.aruco, self.aruco_dict_type)) # sources first 50 of 5x5 aruco dictionary
#         self.aruco_params = cv2.aruco.DetectorParameters_create()
        
#         # CV Bridge for ROS-OpenCV conversion
#         self.bridge = CvBridge()
        
#         # Publisher for ArUco corner positions
#         self.corner_pub = rospy.Publisher('aruco_corners_topic', Float32MultiArray, queue_size=10)
        
#         # Subscriber to camera feed
#         self.image_sub = rospy.Subscriber(self.RGB_topic, Image, self.image_callback)
        
#         # For visualization (optional)
#         self.debug = rospy.get_param('~debug', True)
#         if self.debug:
#             self.debug_pub = rospy.Publisher('aruco_debug_image', Image, queue_size=1)
        
#         rospy.loginfo("ArUco Detector Node Initialized")
#         rospy.loginfo(f"Tracking marker ID: {self.target_marker_id}")
        
#     def image_callback(self, msg):
#         """
#         Callback for camera image messages
#         Detects ArUco markers and publishes corner positions
#         """
#         try:
#             # Convert ROS Image message to OpenCV format
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
#             # Convert to grayscale for detection
#             gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
#             # Detect ArUco markers
#             corners, ids, rejected = cv2.aruco.detectMarkers(
#                 gray, 
#                 self.aruco_dict, 
#                 parameters=self.aruco_params
#             )
            
#             # If markers are detected
#             if ids is not None and len(ids) > 0:
#                 # Find the target marker
#                 for i, marker_id in enumerate(ids.flatten()):
#                     if marker_id == self.target_marker_id:
#                         # Extract corners for this marker
#                         # corners[i] shape: (1, 4, 2) - 4 corners with (x, y) coordinates
#                         marker_corners = corners[i].reshape(4, 2)
                        
#                         # Flatten corners to 1D array: [x1, y1, x2, y2, x3, y3, x4, y4]
#                         corners_flat = marker_corners.flatten()
                        
#                         # Publish corner positions
#                         corner_msg = Float32MultiArray()
#                         corner_msg.data = corners_flat.tolist()
#                         self.corner_pub.publish(corner_msg)
                        
#                         # Debug visualization
#                         if self.debug:
#                             debug_image = cv_image.copy()
#                             cv2.aruco.drawDetectedMarkers(debug_image, [corners[i]], ids[i])
                            
#                             # Draw corner points
#                             for j, corner in enumerate(marker_corners):
#                                 cv2.circle(debug_image, tuple(corner.astype(int)), 5, (0, 255, 0), -1)
#                                 cv2.putText(debug_image, f"C{j}", 
#                                           tuple(corner.astype(int) + 10), 
#                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                            
#                             # Publish debug image
#                             debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
#                             self.debug_pub.publish(debug_msg)
                        
#                         rospy.loginfo_throttle(1.0, f"Marker {self.target_marker_id} detected at corners: {marker_corners[0]}")
#                         break
#                 else:
#                     rospy.logwarn_throttle(2.0, f"Target marker {self.target_marker_id} not found in frame")
#             else:
#                 rospy.logwarn_throttle(2.0, "No ArUco markers detected")
                
#         except Exception as e:
#             rospy.logerr(f"Error in image callback: {str(e)}")
    
#     def run(self):
#         """Keep the node running"""
#         rospy.spin()

# if __name__ == '__main__':
#     try:
#         node = ArucoCornersPub()
#         node.run()
#     except rospy.ROSInterruptException:
#         pass
