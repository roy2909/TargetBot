#!/usr/bin/env python3
"""
Offers a service to detect pins and publish them as markers.

PUBLISHERS:
    visualization_marker_array (MarkerArray/visualization_msgs.msg) - publishes marker arrays

SERVICES:
    coordinates (Empty/std_srvs.srv) - detect pins
"""
from ultralytics import YOLO
import os
import cv2
import copy
import pyrealsense2 as rs2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from std_srvs.srv import Empty

import tf2_geometry_msgs
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from cv_bridge import CvBridge, CvBridgeError

if not hasattr(rs2, "intrinsics"):
    import pyrealsense2.pyrealsense2 as rs2


bridge = CvBridge()


class Camera_subscriber(Node):
    """
    Camera subscriber node that achieves cv recognition.

    Args:
    ----
        Node (rclpy.node.Node): Super class of the writer node.

    Attrbutes
    ---------
        bridge (CvBridge): OpenCV bridge for converting image messages.
        _depth_image_topic (str): Topic for depth images.
        _depth_info_topic (str): Topic for depth camera information.
        sub_depth (Subscription): Subscription for depth images.
        sub_info (Subscription): Subscription for depth camera information.
        sub1 (Subscription): Subscription for color images.
        intrinsics (None or CameraInfo): Camera intrinsic parameters.
        pix (None or ndarray): Pixel coordinates of detected pins.
        pix_grade (None or ndarray): Pixel coordinates of detected pins after grading.
        model (YOLO): YOLO model for detecting pins.
        centroid (Service): Service for obtaining detected pin coordinates.
        _latest_depth_img (None or Image): Latest depth image.
        _latest_color_img (None or Image): Latest color image.
        point_r (PointStamped): Detected red pin coordinates.
        point_y (PointStamped): Detected yellow pin coordinates.
        point_g (PointStamped): Detected green pin coordinates.
        point_b (PointStamped): Detected blue pin coordinates.
        marker_array (MarkerArray): Marker array for rviz visualization.
        red_count (int): Count of detected red pins.
        blue_count (int): Count of detected blue pins.
        green_count (int): Count of detected green pins.
        yellow_count (int): Count of detected yellow pins.
        pub2 (Publisher): Publisher for visualization marker array.

    """

    def __init__(self):
        super().__init__("camera_subscriber")
        self.bridge = CvBridge()
        self._depth_image_topic = (
            "camera/aligned_depth_to_color/image_raw"
        )
        self._depth_info_topic = "/camera/depth/camera_info"
        self.sub_depth = self.create_subscription(
            msg_Image, self._depth_image_topic, self.imageDepthCallback, 1
        )
        self.sub_info = self.create_subscription(
            CameraInfo, self._depth_info_topic, self.imageDepthInfoCallback, 1
        )

        self.sub1 = self.create_subscription(
            msg_Image, "/camera/color/image_raw", self.get_latest_frame, 1
        )
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None
        path = os.path.dirname(__file__)
        self.model = YOLO(path + '/best.pt')
        self.centroid = self.create_service(Empty, "coordinates", self.detect_pins)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._latest_depth_img = None
        self._latest_color_img = None
        self._latest_color_img_ts = None
        self.inference_ts = None
        self.point_r = PointStamped()
        self.point_y = PointStamped()
        self.point_g = PointStamped()
        self.point_b = PointStamped()
        self.pub = self.create_publisher(msg_Image, "pixel_img", 10)
        self.marker_array = MarkerArray()
        self.red_count = 0
        self.blue_count = 0
        self.green_count = 0
        self.yellow_count = 0
        self.t = None

        self.scaling = 0.615

        markerQoS = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        )
        self.pub2 = self.create_publisher(
            MarkerArray, "visualization_marker_array", markerQoS
        )

    def detect_pins(self, request, response):
        """
        Detect pins in the camera feed using a YOLO model.

        Args:
        ----
            request: Service request.
            response: Service response.

        Returns
        -------
            Service response.

        """
        red_pins = []  # List to store centroids of red pins
        yellow_pins = []
        green_pins = []
        blue_pins = []
        # not_pins=[]
        cent = []
        self.inference_ts = copy.deepcopy(self._latest_color_img_ts)

        results = self.model(self._latest_color_img)
        # Use the YOLO model to get detection results
        try:
            self.t = self.tf_buffer.lookup_transform(
                "panda_link0", "panda_hand", rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform camera_link to panda_link0: {ex}"
            )
        # print(str(self.t))

        try:
            t = self.tf_buffer.lookup_transform(
                "panda_hand", "camera_link", rclpy.time.Time()
            )
            self.get_logger().info(f"Transform is found: {t}")
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform camera_link to panda_link0: {ex}"
            )
        # print(str(t))

        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = (
                    box.xyxy[0].to("cpu").detach().numpy().copy()
                )  # Get box coordinates in (top, left, bottom, right) format
                centroid = (
                    (b[0] + b[2]) // 2,
                    (b[1] + b[3]) // 2,
                )  # Calculate centroid from box coordinates
                cent.append(centroid)
                class_name = self.model.names[int(box.cls)]  # Get class name
                if class_name != "not_pins":  # Exclude class "not_pins"
                    if class_name == "red_pins" and box.conf > 0.5:
                        x, y, z = self.depth_world(
                            centroid[1], centroid[0]
                        )  # Get x, y, z from depth_world function
                        self.get_logger().info(f"{x},{y},{z}")
                        red_pins.append((x, y, z))
                        for i in red_pins:
                            self.point_r.header.stamp = (
                                self.get_clock().now().to_msg()
                            )  # Set the timestamp
                            self.point_r.header.frame_id = f"red_pins_{self.red_count}"
                            self.point_r.point.x = i[2]  # Set x, y, z coordinates
                            self.point_r.point.y = -i[0] * self.scaling  # i[1]
                            self.point_r.point.z = -i[1]
                            self.create_marker(
                                self.point_r.point.x,
                                self.point_r.point.y,
                                self.point_r.point.z,
                                self.red_count,
                                "red",
                            )
                            self.red_count += 1

                    elif class_name == "yellow_pins" and box.conf > 0.5:
                        x1, y1, z1 = self.depth_world(
                            centroid[1], centroid[0]
                        )  # Get x, y, z from depth_world function
                        yellow_pins.append((x1, y1, z1))
                        for j in yellow_pins:
                            self.point_y.header.stamp = (
                                self.get_clock().now().to_msg()
                            )  # Set the timestamp
                            self.point_y.header.frame_id = (
                                f"yellow_pins_{self.yellow_count}"
                            )
                            self.point_y.point.x = j[2]  # Set x, y, z coordinates
                            self.point_y.point.y = -j[0] * self.scaling  # j[1]
                            self.point_y.point.z = -j[1]
                            self.create_marker(
                                self.point_y.point.x,
                                self.point_y.point.y,
                                self.point_y.point.z,
                                self.yellow_count,
                                "yellow",
                            )
                            self.yellow_count += 1

                    elif class_name == "green_pins" and box.conf > 0.5:
                        x2, y2, z2 = self.depth_world(
                            centroid[1], centroid[0]
                        )  # Get x, y, z from depth_world function
                        green_pins.append((x2, y2, z2))
                        for k in green_pins:
                            self.point_g.header.stamp = (
                                self.get_clock().now().to_msg()
                            )  # Set the timestamp
                            self.point_g.header.frame_id = (
                                f"green_pins_{self.green_count}"
                            )
                            self.point_g.point.x = k[2]  # Set x, y, z coordinates
                            self.point_g.point.y = -k[0] * self.scaling  # k[1]
                            self.point_g.point.z = -k[1]
                            self.create_marker(
                                self.point_g.point.x,
                                self.point_g.point.y,
                                self.point_g.point.z,
                                self.green_count,
                                "green",
                            )
                            self.green_count += 1

                    elif class_name == "blue_pins" and box.conf > 0.5:
                        x3, y3, z3 = self.depth_world(
                            centroid[1], centroid[0]
                        )  # Get x, y, z from depth_world function
                        blue_pins.append((x3, y3, z3))

                        for b in blue_pins:
                            self.point_b.header.stamp = (
                                self.get_clock().now().to_msg()
                            )  # Set the timestamp
                            self.point_b.header.frame_id = (
                                f"blue_pins_{self.blue_count}"
                            )
                            self.point_b.point.x = b[2]  # Set x, y, z coordinates
                            self.point_b.point.y = -b[0] * self.scaling
                            self.point_b.point.z = -b[1]
                            self.get_logger().info(
                                "BLUE PIN DETECTED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                            )
                            self.get_logger().info(
                                f"blue pin x: {self.point_b.point.x}"
                            )
                            self.get_logger().info(
                                f"blue pin y: {self.point_b.point.y}"
                            )
                            self.get_logger().info(
                                f"blue pin z: {self.point_b.point.z}"
                            )

                            self.create_marker(
                                self.point_b.point.x,
                                self.point_b.point.y,
                                self.point_b.point.z,
                                self.blue_count,
                                "blue",
                            )
                            self.blue_count += 1

            for x in cent:
                img = cv2.circle(
                    self._latest_color_img,
                    (int(x[0]), int(x[1])),
                    radius=5,
                    color=(0, 0, 255),
                    thickness=-1,
                )
            if len(cent) > 0:
                img_msg = self.bridge.cv2_to_imgmsg(img)
                self.pub.publish(img_msg)

            self.pub2.publish(self.marker_array)
            self.marker_array = MarkerArray()
        return response

    def depth_world(self, x, y):
        """
        Convert pixel coordinates to real-world coordinates using depth information.

        Args:
        ----
            x (int): X-coordinate.
            y (int): Y-coordinate.

        Returns
        -------
            Tuple[float, float, float]: Real-world coordinates (x, y, z).

        """
        if (
            self.intrinsics
            and self._latest_depth_img is not None
            and self._latest_color_img is not None
        ):
            self.get_logger().info("processing request")

            depth_x = int(x)
            depth_y = int(y)
            depth = self._latest_depth_img[depth_x, depth_y]

            result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [y, x], depth)
            print(self.intrinsics)
            x_new, y_new, z_new = result[0], result[1], result[2]

            return x_new, y_new, z_new

    def create_marker(self, x, y, z, count, ns):
        """
        Create a visualization marker based on coordinates.

        Args:
        ----
            x (float): X-coordinate in meters.
            y (float): Y-coordinate in meters.
            z (float): Z-coordinate in meters.
            count (int): Marker ID.
            ns (str): Marker namespace.

        Returns
        -------
            None

        """
        marker = Marker()
        marker.header.frame_id = "panda_link0"
        marker.header.stamp = self.inference_ts

        k = PointStamped()
        k.point.x = z / 1000
        k.point.y = -y / 1000
        k.point.z = x / 1000

        tp = tf2_geometry_msgs.do_transform_point(k, self.t)
        marker.id = count
        marker.ns = ns
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.pose.position.x = tp.point.x
        marker.pose.position.y = (tp.point.y) + 0.05
        marker.pose.position.z = tp.point.z
        print(
            f"namespace:{ns}, \
            x:{marker.pose.position.x}, \
            y:{marker.pose.position.y}, \
            z:{marker.pose.position.z}"
        )
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        if ns == "red":
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        elif ns == "green":
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        elif ns == "blue":
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
        elif ns == "yellow":
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        # Append the marker to the MarkerArray
        self.marker_array.markers.append(marker)

    def get_latest_frame(self, data):
        """
        Call for the latest color image.

        Args:
        ----
            data (Image): Color image message.

        Returns
        -------
            None

        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self._latest_color_img = cv_image
            self._latest_color_img_ts = data.header.stamp
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return

    def imageDepthInfoCallback(self, cameraInfo):
        """
        Obtain depth camera information.

        Args:
        ----
            cameraInfo (CameraInfo): Camera information message.

        Returns
        -------
            None

        """
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == "plumb_bob":
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == "equidistant":
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.d]
        except CvBridgeError as e:
            print(e)
            return

    def imageDepthCallback(self, data):
        """
        Obtain latest depth image.

        Args:
        ----
            data (Image): Depth image message.

        Returns
        -------
            None

        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self._latest_depth_img = cv_image
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return


def main(args=None):
    """
    Run the Camera_subscriber node.

    Args:
    ----
        args: Command-line arguments.

    Returns
    -------
        None

    """
    rclpy.init(args=args)
    node = Camera_subscriber()
    rclpy.spin(node)
    rclpy.shutdown()
