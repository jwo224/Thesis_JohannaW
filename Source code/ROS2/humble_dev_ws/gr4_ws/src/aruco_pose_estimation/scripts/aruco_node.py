#!/usr/bin/env python3
"""
ROS2 wrapper code taken from:
https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco/tree/main

This node locates ArUco markers in images and publishes their IDs and poses.

Subscriptions:
   /camera/image_raw      sensor_msgs.msg.Image
   /camera/camera_info    sensor_msgs.msg.CameraInfo

Published topics:
   /aruco_poses           geometry_msgs.msg.PoseArray
   /aruco_markers         aruco_interfaces.msg.ArucoMarkers
   /aruco_image           sensor_msgs.msg.Image

This version supports both:
   - newer OpenCV ArUco API with cv2.aruco.ArucoDetector
   - older OpenCV ArUco API with cv2.aruco.detectMarkers
"""

import cv2
import message_filters
import numpy as np
import rclpy
import rclpy.node
from aruco_interfaces.msg import ArucoMarkers
from aruco_pose_estimation.pose_estimation import pose_estimation
from aruco_pose_estimation.utils import ARUCO_DICT
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        self.initialize_parameters()

        try:
            dictionary_id = getattr(cv2.aruco, self.dictionary_id_name)

            if dictionary_id not in ARUCO_DICT.values():
                raise AttributeError

        except AttributeError:
            self.get_logger().error(
                f"Bad aruco_dictionary_id: {self.dictionary_id_name}"
            )
            options = "\n".join([s for s in ARUCO_DICT])
            self.get_logger().error(f"Valid options:\n{options}")
            raise

        self.bridge = CvBridge()

        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary = self.create_aruco_dictionary(dictionary_id)
        self.aruco_parameters = self.create_aruco_parameters()
        self.aruco_detector = self.create_aruco_detector(
            self.aruco_dictionary,
            self.aruco_parameters,
        )

        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            self.info_topic,
            self.info_callback,
            self.qos,
        )

        if self.use_depth_input:
            self.image_sub = message_filters.Subscriber(
                self,
                Image,
                self.image_topic,
                qos_profile=self.qos,
            )

            self.depth_image_sub = message_filters.Subscriber(
                self,
                Image,
                self.depth_image_topic,
                qos_profile=self.qos,
            )

            self.synchronizer = message_filters.ApproximateTimeSynchronizer(
                [self.image_sub, self.depth_image_sub],
                queue_size=10,
                slop=0.05,
            )
            self.synchronizer.registerCallback(self.rgb_depth_sync_callback)

        else:
            self.image_sub = self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                self.qos,
            )

        self.poses_pub = self.create_publisher(
            PoseArray,
            self.markers_visualization_topic,
            self.qos,
        )

        self.markers_pub = self.create_publisher(
            ArucoMarkers,
            self.detected_markers_topic,
            self.qos,
        )

        self.image_pub = self.create_publisher(
            Image,
            self.output_image_topic,
            self.qos,
        )

        self.get_logger().info("Aruco node started.")

    def create_aruco_dictionary(self, dictionary_id):
        if hasattr(cv2.aruco, "getPredefinedDictionary"):
            return cv2.aruco.getPredefinedDictionary(dictionary_id)

        return cv2.aruco.Dictionary_get(dictionary_id)

    def create_aruco_parameters(self):
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            return cv2.aruco.DetectorParameters_create()

        return cv2.aruco.DetectorParameters()

    def create_aruco_detector(self, aruco_dictionary, aruco_parameters):
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.get_logger().info("Using new OpenCV ArucoDetector API.")
            return cv2.aruco.ArucoDetector(
                aruco_dictionary,
                aruco_parameters,
            )

        self.get_logger().info("Using old OpenCV detectMarkers API.")
        return aruco_dictionary, aruco_parameters

    def info_callback(self, info_msg):
        self.info_msg = info_msg

        self.intrinsic_mat = np.reshape(
            np.array(self.info_msg.k, dtype=np.float64),
            (3, 3),
        )

        self.distortion = np.array(self.info_msg.d, dtype=np.float64)

        self.get_logger().info("Camera info received.")
        self.get_logger().info(f"Intrinsic matrix: {self.intrinsic_mat}")
        self.get_logger().info(f"Distortion coefficients: {self.distortion}")
        self.get_logger().info(
            f"Camera frame: {self.info_msg.width}x{self.info_msg.height}"
        )

        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg: Image):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                img_msg,
                desired_encoding="rgb8",
            )
        except Exception as e:
            self.get_logger().warn(f"Could not convert image: {e}")
            return

        markers = ArucoMarkers()
        pose_array = PoseArray()

        self.fill_headers(
            markers=markers,
            pose_array=pose_array,
            stamp=img_msg.header.stamp,
        )

        try:
            frame, pose_array, markers = pose_estimation(
                rgb_frame=cv_image,
                depth_frame=None,
                aruco_detector=self.aruco_detector,
                marker_size=self.marker_size,
                matrix_coefficients=self.intrinsic_mat,
                distortion_coefficients=self.distortion,
                pose_array=pose_array,
                markers=markers,
                draw_axes=self.draw_axes,
            )
        except Exception as e:
            self.get_logger().warn(f"Pose estimation failed: {e}")
            return

        if len(markers.marker_ids) > 0:
            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)

        try:
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
            out_msg.header.stamp = img_msg.header.stamp
            out_msg.header.frame_id = self.output_frame_id()
            self.image_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f"Could not publish output image: {e}")

    def rgb_depth_sync_callback(self, rgb_msg: Image, depth_msg: Image):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        try:
            cv_depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="16UC1",
            )
            cv_image = self.bridge.imgmsg_to_cv2(
                rgb_msg,
                desired_encoding="rgb8",
            )
        except Exception as e:
            self.get_logger().warn(f"Could not convert synchronized images: {e}")
            return

        markers = ArucoMarkers()
        pose_array = PoseArray()

        self.fill_headers(
            markers=markers,
            pose_array=pose_array,
            stamp=rgb_msg.header.stamp,
        )

        try:
            frame, pose_array, markers = pose_estimation(
                rgb_frame=cv_image,
                depth_frame=cv_depth_image,
                aruco_detector=self.aruco_detector,
                marker_size=self.marker_size,
                matrix_coefficients=self.intrinsic_mat,
                distortion_coefficients=self.distortion,
                pose_array=pose_array,
                markers=markers,
                draw_axes=self.draw_axes,
            )
        except Exception as e:
            self.get_logger().warn(f"Pose estimation failed: {e}")
            return

        if len(markers.marker_ids) > 0:
            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)

        try:
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
            out_msg.header.stamp = rgb_msg.header.stamp
            out_msg.header.frame_id = self.output_frame_id()
            self.image_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f"Could not publish output image: {e}")

    def fill_headers(self, markers: ArucoMarkers, pose_array: PoseArray, stamp):
        frame_id = self.output_frame_id()

        markers.header.frame_id = frame_id
        markers.header.stamp = stamp

        pose_array.header.frame_id = frame_id
        pose_array.header.stamp = stamp

    def output_frame_id(self):
        if self.camera_frame == "":
            return self.info_msg.header.frame_id

        return self.camera_frame

    def initialize_parameters(self):
        self.declare_parameter(
            name="marker_size",
            value=0.0625,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_5X5_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name="use_depth_input",
            value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Use depth camera input for pose estimation.",
            ),
        )

        self.declare_parameter(
            name="image_topic",
            value="/camera/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="depth_image_topic",
            value="/camera/depth/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Depth camera topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/camera/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )

        self.declare_parameter(
            name="detected_markers_topic",
            value="/aruco_markers",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish detected marker IDs and poses.",
            ),
        )

        self.declare_parameter(
            name="markers_visualization_topic",
            value="/aruco_poses",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish marker poses.",
            ),
        )

        self.declare_parameter(
            name="output_image_topic",
            value="/aruco_image",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish annotated image.",
            ),
        )
        self.declare_parameter(
            name="draw_axes",
            value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Draw 3D axes on annotated ArUco images.",
            ),
        )

        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size: {self.marker_size}")

        self.dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id")
            .get_parameter_value()
            .string_value
        )
        self.get_logger().info(f"Marker type: {self.dictionary_id_name}")

        self.use_depth_input = (
            self.get_parameter("use_depth_input").get_parameter_value().bool_value
        )
        self.get_logger().info(f"Use depth input: {self.use_depth_input}")

        self.image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Input image topic: {self.image_topic}")

        self.depth_image_topic = (
            self.get_parameter("depth_image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Input depth image topic: {self.depth_image_topic}")

        self.info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image camera info topic: {self.info_topic}")

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )
        self.get_logger().info(f"Camera frame: {self.camera_frame}")

        self.detected_markers_topic = (
            self.get_parameter("detected_markers_topic")
            .get_parameter_value()
            .string_value
        )
        self.get_logger().info(
            f"Detected markers topic: {self.detected_markers_topic}"
        )

        self.markers_visualization_topic = (
            self.get_parameter("markers_visualization_topic")
            .get_parameter_value()
            .string_value
        )
        self.get_logger().info(
            f"Markers visualization topic: {self.markers_visualization_topic}"
        )

        self.output_image_topic = (
            self.get_parameter("output_image_topic")
            .get_parameter_value()
            .string_value
        )
        self.get_logger().info(f"Output image topic: {self.output_image_topic}")

        self.draw_axes = (
            self.get_parameter("draw_axes").get_parameter_value().bool_value
        )
        self.get_logger().info(f"Draw axes on output image: {self.draw_axes}")


def main(args=None):
    rclpy.init(args=args)

    node = ArucoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
