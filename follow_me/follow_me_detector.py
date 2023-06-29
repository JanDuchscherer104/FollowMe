#!/usr/bin/env python3
import numpy as np

# import tf2_geometry_msgs
# import tf2_ros
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point  # , PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2

from .human_pose_estimator import HumanPoseEstimator

# from tf2_ros import ConnectivityException, ExtrapolationException, LookupException


POINT_CLOUD_TOPIC = "/zed/point_cloud/cloud_registered"
IMG_TOPIC = "/zed/left/image_rect_color"
LOCATION_TOPIC = "/human_location"
IMG_OUT_TOPIC = "/proc_bgr_img"


class PoseEstimationNode(Node):
    def __init__(self, publish_img: bool = False):
        super().__init__("pose_estimation_node")
        self.estimator = HumanPoseEstimator(publish_img=publish_img)
        self.publish_img = publish_img

        self.cv_bridge = CvBridge()

        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # initialize the publishers & subscribers
        self.rgb_sub = Subscriber(self, Image, IMG_TOPIC)
        self.point_cloud_sub = Subscriber(self, PointCloud2, POINT_CLOUD_TOPIC)

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.point_cloud_sub], queue_size=10, slop=0.5
        )
        self.sync.registerCallback(self.zed_callback)
        self.point_publisher = self.create_publisher(Point, LOCATION_TOPIC, 10)
        if self.publish_img:
            self.img_publisher = self.create_publisher(Image, IMG_OUT_TOPIC, 10)

        self.get_logger().info(f"Started node {self.get_name()}.")

    def zed_callback(self, rgb_msg: Image, point_cloud_msg: PointCloud2):
        """
        Subscribes to the ZED topics, processes the images and publishes the human location.
        """

        # Convert ROS image message to OpenCV format and process the bgr frames
        rgb_frame = self.cv_bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        result = self.estimator.process(rgb_frame, point_cloud_msg)
        if result is not None:
            if self.publish_img:
                image, point = result
                self.publish_image(image)
            else:
                _, point = result
            self.point_publisher.publish(point)
            # try:
            #     # Create a stamped version of the point
            #     point_stamped = PointStamped()
            #     point_stamped.point = point
            #     point_stamped.header.frame_id = 'zed_camera_center'
            #     point_stamped.header.stamp = self.get_clock().now().to_msg()

            #     # Transform the point
            #     transform = self.tf_buffer.lookup_transform('base_link', 'zed_camera_center', rclpy.time.Time())
            #     transformed_point_stamped = tf2_geometry_msgs.do_transform_point(point_stamped, transform)

            #     # Publish the transformed point
            #     self.point_publisher.publish(transformed_point_stamped.point)
            # except (LookupException, ConnectivityException, ExtrapolationException) as e:
            #     self.get_logger().error("TF2 exception occurred.")
            #     self.get_logger().error(e)

    def publish_image(self, image: np.ndarray):
        ros_image = self.cv_bridge.cv2_to_imgmsg(image, "bgr8")
        self.img_publisher.publish(ros_image)


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
# TODO add tf to base_link, publish PointStamped
