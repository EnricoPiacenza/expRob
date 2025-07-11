#!/usr/bin/env python3

import rclpy
import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry

class MarkerSubscriber(Node):
    """
    Class to subscribe to the ArucoMarkers and the /camera/image_raw topic,
    publish the angular velocity to the /cmd_vel topic 
    and the custom image to the processed_image topic
    """

    def __init__(self):
        super().__init__('RobotNode')

        # Publisher for the angular velocity
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for the custom image topic
        self.image_publisher_ = self.create_publisher(Image, 'processed_image', 10)

        # Status variable to control the flow of the program
        self.status = 1 # 1 - Looking for markers, 2 - taking photos, 3 - done
        self.active = True
        
        # num unique markers
        self.num_unique_markers = 5
        self.unique_marker_ids = []
        
        # Initialize CvBridge
        self.bridge = CvBridge()
 
        # Subscriber for the /camera/image_raw topic
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )
        self.camera_subscription
        
        # Subscribe to the ArucoMarkers topic to receive the marker IDs
        self.subscription = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',  
            self.marker_callback,
            10  
        )
        self.subscription

        # Timer to control the flow of the program
        self.timer = self.create_timer(0.1, self.run)


    def marker_callback(self, msg):
 
        if msg.marker_ids:
            for marker_id in msg.marker_ids:
                if self.status == 1:
                    if marker_id not in self.unique_marker_ids:
                        self.unique_marker_ids.append(marker_id)  
                        self.get_logger().info(f'Marker ID detected: {marker_id}')
                    if len(self.unique_marker_ids) >= self.num_unique_markers:
                        self.status = 2
                        self.unique_marker_ids.sort()
                        self.get_logger().info(f'Found {len(self.unique_marker_ids)} unique markers, switching to status 2')
    
    
    def camera_callback(self, msg):
        """
        Callback function for the /camera/image_raw topic,
        the function will convert the image message to a cv
        """
        try:
            bridge = CvBridge()
            img = self.bridge.imgmsg_to_cv2(msg,
                                             desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            
        if self.status == 2:
            if len(img.shape) == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
            parameters = aruco.DetectorParameters_create()
            marker_corners, marker_ids, rejected_candidates = aruco.detectMarkers(img, aruco_dict, parameters=parameters)
                
            if marker_ids is not None and len(marker_ids) > 0:
 
                for i, corners in enumerate(marker_corners):
        
                    center_x = int(np.mean(corners[0][:, 0]))
                    center_y = int(np.mean(corners[0][:, 1]))
        
                    if marker_ids[i][0] == self.unique_marker_ids[0]:
                        marker = self.unique_marker_ids.pop(0)
                        cv2.circle(img, (center_x, center_y), 50, (0, 0, 255), 5)
                        cv2.imwrite(f'img{marker}.png', img)
                        cv2.imshow('Detected Markers', img)
                        cv2.waitKey(1)
                        self.get_logger().info(f'Marker {marker} detected and saved.')
                            
                    if not self.unique_marker_ids:
                        self.status = 3
                        self.get_logger().info('All unique markers found, switching to status 3')
            else:
                self.get_logger().info("No markers detected.")
        
        return msg
    
    def run(self):
        if self.active:
            if self.status == 1:
                msg = Twist()
                msg.angular.z = 0.5
                msg.linear.x = 0.0
                self.publisher_.publish(msg)
            elif self.status == 2:
                msg = Twist()
                msg.angular.z = 0.5
                msg.linear.x = 0.0
                self.publisher_.publish(msg)
            elif self.status == 3:
                msg = Twist()
                msg.angular.z = 0.0
                msg.linear.x = 0.0
                self.publisher_.publish(msg)
                self.active = False
                self.get_logger().info('Status 3 reached, stopping the robot')
    

def main(args=None):
    rclpy.init(args=args)
    """
    Main function to initialize the MarkerSubscriber and publish the angular velocity
    """
    marker_subscriber = MarkerSubscriber()
    rclpy.spin(marker_subscriber)
    marker_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()