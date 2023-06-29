#!/usr/bin/env python3
import struct
from typing import Optional, Tuple

import cv2
import mediapipe as mp
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2


class HumanPoseEstimator:
    def __init__(self, publish_img: bool = False):
        self.publish_img = publish_img

        # initialize the mediapipe pose model
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        self.pose_detector = self.mp_pose.Pose(
            min_detection_confidence=0.8, min_tracking_confidence=0.8
        )

        print(f"INFO: Initialized HumanPoseEstimator.")

    def process(self, rgb_frame, point_cloud) -> Tuple[Optional[np.ndarray], Point]:
        """Processes the given frame and returns the 3D coordinates of the detected person."""
        image = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2RGB)

        # mark as read-only to improve performance and make detection
        image.flags.writeable = False
        results = self.pose_detector.process(image)
        if results.pose_landmarks is None:
            return None

        # extract landmarks and subsequently the coordinates w.r.t. the camera frame
        landmarks = results.pose_landmarks.landmark
        landmark_arr = np.array([(lm.x, lm.y, lm.visibility) for lm in landmarks])
        xyz = self.get_coordinates(point_cloud, landmark_arr)

        if self.publish_img:
            image = self.proc_image(image, results, xyz)
        else:
            image = None
        x, y, z = xyz
        return image, Point(x=x, y=y, z=z)

    def get_coordinates(
        self,
        point_cloud: PointCloud2,
        landmarks: np.ndarray,
        min_confidence: float = 0.8,
    ) -> Tuple[float, float, float]:
        """Extracts the 3D coordinates of the landmarks from the point cloud."""
        # extract the point cloud fields
        width, height = point_cloud.width, point_cloud.height
        point_step = point_cloud.point_step
        row_step = point_cloud.row_step
        data = point_cloud.data

        # filter out landmarks with low confidence and convert to pixel coordinates
        good_landmarks = landmarks[landmarks[:, 2] > min_confidence]
        np.multiply(
            good_landmarks[:, :2],
            (width, height),
            out=good_landmarks[:, :2],
        )
        uv = np.clip(
            good_landmarks[:, :2].astype(np.int32),
            [0, 0],
            [width - 1, height - 1],
        )

        # calculate the offests for the given uv coordinates and extract the points
        offsets = np.multiply(uv[:, 1], row_step) + np.multiply(uv[:, 0], point_step)
        points = np.array(
            [list(struct.unpack_from("fff", data, offset)) for offset in offsets]
        )
        xyz = np.nanmedian(points, axis=0, out=points[:1]).tolist()[0]

        return xyz

    def proc_image(
        self, image: np.ndarray, results, xyz: Tuple[float, float, float]
    ) -> np.ndarray:
        """Draws landmarks and xyz coordinates on the image."""
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.putText(
            image,
            "x: {}, y: {}, z: {}".format(*xyz),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        # render detections
        self.mp_drawing.draw_landmarks(
            image,
            results.pose_landmarks,
            self.mp_pose.POSE_CONNECTIONS,
            self.mp_drawing.DrawingSpec(
                color=(245, 117, 66), thickness=2, circle_radius=2
            ),
            self.mp_drawing.DrawingSpec(
                color=(245, 66, 230), thickness=2, circle_radius=2
            ),
        )

        return image
