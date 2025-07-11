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
import time
from std_srvs.srv import SetBool
from std_msgs.msg import Int32


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


        self.id_publisher = self.create_publisher(Int32, 'aruco_marker_id', 10)

        self.client = self.create_client(SetBool, 'response_patrol')
        self.service = self.create_service(SetBool, 'patrol_action', self.service_callback)

        # Status variable to control the flow of the program
        self.status = 1 # 1 - Looking for markers, 2 - Patrolling
        self.active = False

        # Initialize CvBridge
        self.bridge = CvBridge()

        self.marker_ids = []

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


    def service_callback(self, request, response):
        self.active = request.data
        self.status = 1
        return response
    
    def marker_callback(self, msg):

        if msg.marker_ids and self.active:
            for marker_id in msg.marker_ids:
                if marker_id > 0:
                    self.get_logger().info(f'Marker ID detected: {marker_id}')
                    self.marker_ids.append(marker_id)
                    self.status = 2
        elif self.active:
            self.get_logger().info('No markers detected')
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
                msg.angular.z = 0.0
                msg.linear.x = 0.0
                self.publisher_.publish(msg)
                self.active = False

                if len(self.marker_ids) == 4:
                    id_msg = Int32()
                    min_value = min(self.marker_ids)
                    min_index = self.marker_ids.index(min_value)
                    id_msg.data = min_index
                    self.id_publisher.publish(id_msg)
                
                self.get_logger().info('Number of Uniques markers: ' + str(len(self.marker_ids)))
                request = SetBool.Request()
                request.data = True
                future = self.client.call_async(request)



        return None

    

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
