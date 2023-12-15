"""
A Control node that could control the robot with various clients.

Clients
-------
    input:user input [UserInput]
    coordinates: pin coordinates [Empty]
    target_scan: target scan request [TargetScanRequest]
    gun_scan: gun scan request [Empty]
    grab: grab request [Grab]
    place: place back gun request [Grab]
    cali: calibration [Empty]
    aim: gun aiming [Target]
    fire: gun firing [Fire]

Returns
-------
    None

"""
import time
import rclpy
import copy
from rclpy.node import Node
from std_srvs.srv import Empty
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion, Pose
from trajectory_interfaces.srv import Grab, Target, TargetScanRequest, UserInput
from trigger_interfaces.srv import Fire


class ControlNode(Node):
    """
    A Control node that could control the robot to achieve different gun shooting.

    Args:
    ----
        Node (ros node): superclass of the ros node

    """

    def __init__(self):
        super().__init__("control")
        # Broadcast a camera frame
        self.camera_broadcaster = StaticTransformBroadcaster(self)
        self._cam_hand_tf = TransformStamped()
        self._cam_hand_tf.header.stamp = self.get_clock().now().to_msg()
        self._cam_hand_tf.header.frame_id = "panda_hand"
        self._cam_hand_tf.child_frame_id = "camera_link"
        self._cam_hand_tf.transform.translation.x = 0.05
        self._cam_hand_tf.transform.translation.y = -0.02
        self._cam_hand_tf.transform.translation.z = 0.065
        self._cam_hand_tf.transform.rotation = Quaternion(
            x=0.707, y=0.0, z=0.707, w=0.0
        )

        self.camera_broadcaster.sendTransform(self._cam_hand_tf)  # publish transform

        # Callback group
        self._cbgrp = ReentrantCallbackGroup()
        self.loop_cbgrp = MutuallyExclusiveCallbackGroup()
        self.tf_cbgrp = MutuallyExclusiveCallbackGroup()
        self.marker_cbgrp = MutuallyExclusiveCallbackGroup()

        # clients and publishers
        self._input_client = self.create_client(
            UserInput, "input", callback_group=self._cbgrp
        )
        self._vision_client = self.create_client(
            Empty, "coordinates", callback_group=self._cbgrp
        )
        self._targets_client = self.create_client(
            TargetScanRequest, "target_scan", callback_group=self._cbgrp
        )
        self._gun_client = self.create_client(
            Empty, "gun_scan", callback_group=self._cbgrp
        )
        self._grab_client = self.create_client(Grab, "grab", callback_group=self._cbgrp)
        self._place_client = self.create_client(
            Grab, "place", callback_group=self._cbgrp
        )
        self._calibration_client = self.create_client(
            Empty, "cali", callback_group=self._cbgrp
        )

        self._aim_client = self.create_client(Target, "aim", callback_group=self._cbgrp)

        self._shoot_client = self.create_client(
            Fire, "fire", callback_group=self._cbgrp
        )

        # # wait for services to become available
        while not self._input_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("input service not available, waiting again...")

        while not self._vision_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("yolo service not available, waiting again...")

        while not self._gun_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("gun scan service not available, waiting again...")

        while not self._grab_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("grab service not available, waiting again...")

        while not self._place_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("place service not available, waiting again...")

        while not self._calibration_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("calibrate service not available, waiting again...")

        while not self._aim_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("aim service not available, waiting again...")

        # main loop timer
        self._loop_timer = self.create_timer(
            0.01, self.loop_cb, callback_group=self.loop_cbgrp
        )
        self._tf_timer = self.create_timer(
            1 / 30, self.tf_cb, callback_group=self.tf_cbgrp
        )
        markerQoS = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        )

        self.marker_pub = self.create_subscription(
            MarkerArray,
            "visualization_marker_array",
            self.marker_cb,
            markerQoS,
            callback_group=self.marker_cbgrp,
        )

        # variables
        self._markers = []  # store the MarkerArray
        self.t1 = Pose()
        self.t1.position.x = None
        self.t1.position.y = None
        self.t1.position.z = None
        self.t1.orientation.x = None
        self.t1.orientation.y = None
        self.t1.orientation.z = None
        self.t1.orientation.w = None

        self.t2 = Pose()
        self.t2.position.x = None
        self.t2.position.y = None
        self.t2.position.z = None
        self.t2.orientation.x = None
        self.t2.orientation.y = None
        self.t2.orientation.z = None
        self.t2.orientation.w = None

        self.marker_count = 0

        # TF listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # run once
        self._run = False

    async def loop_cb(self):
        """Control main loop."""
        # calibrate gripper

        self.get_logger().error("Waiting for Input.")

        input_answer = await self._input_client.call_async(UserInput.Request())
        colour_target = input_answer.answer
        self.get_logger().error(f"{colour_target}")
        # RUN ONCE!
        # # scan targets
        await self._calibration_client.call_async(Empty.Request())

        if not self._run:
            await self.scan_targets()

        # scan guns
        self._gun_scan_future = await self._gun_client.call_async(Empty.Request())

        # wait for user input

        # grab gun
        while self.t1.position.x is None:
            pass

        self.get_logger().info(
            f"Gun 1 coordinates: ({self.t1.position.x},{self.t1.position.y},{self.t1.position.z})"
        )

        self.get_logger().info("grabbing gun!")
        # this pause helped stabilize tf tree issues we were having
        for k in range(25):
            time.sleep(0.1)

        if not self._run:
            self.first_t = copy.deepcopy(self.t1)
            self.last_t = copy.deepcopy(self.t2)

        self._run = True
        if self.marker_count < 2:
            self.get_logger().info(f"{self.t1}")
            self._grab_future = await self._grab_client.call_async(
                Grab.Request(pose=self.first_t)
            )
            self.gun1 = True
        else:
            self._grab_future = await self._grab_client.call_async(
                Grab.Request(pose=self.last_t)
            )
            self.gun1 = False

        # # aim
        self.get_logger().info(f"{self._markers}")

        for m in self._markers:
            self.get_logger().info(f"\n{m}")

            if colour_target in m.ns:
                if self.marker_count == 2 and self.gun1:
                    self.place_future = await self._place_client.call_async(
                        Grab.Request(pose=self.first_t)
                    )
                    self.gun1 = False
                    self._grab_future = await self._grab_client.call_async(
                        Grab.Request(pose=self.last_t)
                    )
                target_pose = m.pose.position
                await self._aim_client.call_async(Target.Request(target=target_pose))
                # shoot service
                req = Fire.Request()
                if self.gun1:
                    req.gun_id = 0
                else:
                    req.gun_id = 1

                await self._shoot_client.call_async(req)
                self.marker_count += 1
                self.get_logger().info(f"{self.marker_count}")

        if self.marker_count <= 2 and self.gun1:
            self.get_logger().info(f"{self.first_t}")
            self.place_future = await self._place_client.call_async(
                Grab.Request(pose=self.first_t)
            )
        else:
            self.get_logger().info(f"{self.last_t}")
            self.place_future = await self._place_client.call_async(
                Grab.Request(pose=self.last_t)
            )
        await self._calibration_client.call_async(Empty.Request())
        return

    def tf_cb(self):
        """Listen to tf data to track April Tags."""
        # TF listener
        try:
            # get the latest transform between left and right
            tag_1 = self.buffer.lookup_transform(
                "panda_link0", "tag36h11:42", rclpy.time.Time()
            )
            self.t1.position.x = tag_1.transform.translation.x
            self.t1.position.y = tag_1.transform.translation.y
            self.t1.position.z = tag_1.transform.translation.z
            self.t1.orientation.x = tag_1.transform.rotation.x
            self.t1.orientation.y = tag_1.transform.rotation.y
            self.t1.orientation.w = tag_1.transform.rotation.w
            self.t1.orientation.z = tag_1.transform.rotation.z

            tag_2 = self.buffer.lookup_transform(
                "panda_link0", "tag36h11:95", rclpy.time.Time()
            )
            self.t2.position.x = tag_2.transform.translation.x
            self.t2.position.y = tag_2.transform.translation.y
            self.t2.position.z = tag_2.transform.translation.z
            self.t2.orientation.x = tag_2.transform.rotation.x
            self.t2.orientation.y = tag_2.transform.rotation.y
            self.t2.orientation.w = tag_2.transform.rotation.w
            self.t2.orientation.z = tag_2.transform.rotation.z
        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().debug(f"Lookup exception: {e}")
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().debug(f"Connectivity exception: {e}")
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().debug(f"Extrapolation exception: {e}")

    def marker_cb(self, msg):
        """
        Store the pin positions in markerArrays.

        Args:
        ----
            msg (MarkerArray): Array storing Markers

        """
        for given_m in msg.markers:
            duplicate = False
            x = given_m.pose.position.x
            y = given_m.pose.position.y
            z = given_m.pose.position.z
            for stored_m in self._markers:
                xp = stored_m.pose.position.x
                yp = stored_m.pose.position.y
                zp = stored_m.pose.position.z
                if abs(x - xp) < 0.01 and abs(y - yp) < 0.01 and abs(z - zp) < 0.01:
                    duplicate = True
            if not duplicate:
                self._markers.append(given_m)

    async def scan_targets(self):
        """Move the robot to scanning positions and requests for detections."""
        # move robot to scan position
        self.get_logger().info("position 1")
        response = await self._targets_client.call_async(TargetScanRequest.Request())
        self.get_logger().info("position 1 reached")

        # scan pins
        self.get_logger().info("requesting camera scan...")
        await self._vision_client.call_async(Empty.Request())

        # return
        self.get_logger().info("camera scan complete")

        count = 2
        while response.more_scans:
            # move robot to scan position
            self.get_logger().info(f"position {count}")
            response = await self._targets_client.call_async(
                TargetScanRequest.Request()
            )
            count += 1
            # scan pins
            self.get_logger().info("requesting camera scan...")
            await self._vision_client.call_async(Empty.Request())
            self.get_logger().info("camera scan complete")
        return


def main():
    rclpy.init()
    node = ControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


if __name__ == "__main__":
    main()
