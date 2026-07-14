#!/usr/bin/env python3

# Code taken and readapted from:
# https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python/tree/main

import math
import numpy as np
import cv2
import tf_transformations

from rclpy.impl import rcutils_logger

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from aruco_interfaces.msg import ArucoMarkers

from aruco_pose_estimation.utils import aruco_display


def quaternion_from_rotation_matrix_3x3(R: np.array) -> np.array:
    """
    Convert a 3x3 rotation matrix to quaternion [x, y, z, w].

    This avoids tf_transformations.quaternion_from_matrix(), which can crash
    when the rotation matrix is numerically imperfect.
    """
    R = np.asarray(R, dtype=np.float64)

    if R.shape != (3, 3):
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

    if not np.all(np.isfinite(R)):
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

    try:
        U, _, Vt = np.linalg.svd(R)
        R = U @ Vt

        if np.linalg.det(R) < 0:
            U[:, -1] *= -1
            R = U @ Vt

    except Exception:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]

    trace = m00 + m11 + m22

    try:
        if trace > 0.0:
            s = 0.5 / math.sqrt(max(trace + 1.0, 1e-12))
            w = 0.25 / s
            x = (m21 - m12) * s
            y = (m02 - m20) * s
            z = (m10 - m01) * s

        elif m00 > m11 and m00 > m22:
            s = 2.0 * math.sqrt(max(1.0 + m00 - m11 - m22, 1e-12))
            w = (m21 - m12) / s
            x = 0.25 * s
            y = (m01 + m10) / s
            z = (m02 + m20) / s

        elif m11 > m22:
            s = 2.0 * math.sqrt(max(1.0 + m11 - m00 - m22, 1e-12))
            w = (m02 - m20) / s
            x = (m01 + m10) / s
            y = 0.25 * s
            z = (m12 + m21) / s

        else:
            s = 2.0 * math.sqrt(max(1.0 + m22 - m00 - m11, 1e-12))
            w = (m10 - m01) / s
            x = (m02 + m20) / s
            y = (m12 + m21) / s
            z = 0.25 * s

    except Exception:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

    quaternion = np.array([x, y, z, w], dtype=np.float64)
    norm_quat = np.linalg.norm(quaternion)

    if norm_quat < 1e-12 or not np.isfinite(norm_quat):
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

    return quaternion / norm_quat


def detect_markers_compatible(rgb_frame: np.array, aruco_detector):
    """
    Supports both OpenCV ArUco APIs:

    New API:
        detector = cv2.aruco.ArucoDetector(...)
        detector.detectMarkers(...)

    Old API:
        cv2.aruco.detectMarkers(...)
    """
    if hasattr(aruco_detector, "detectMarkers"):
        return aruco_detector.detectMarkers(rgb_frame)

    aruco_dictionary, aruco_parameters = aruco_detector

    return cv2.aruco.detectMarkers(
        rgb_frame,
        aruco_dictionary,
        parameters=aruco_parameters,
    )


def pose_estimation(
    rgb_frame: np.array,
    depth_frame: np.array,
    aruco_detector,
    marker_size: float,
    matrix_coefficients: np.array,
    distortion_coefficients: np.array,
    pose_array: PoseArray,
    markers: ArucoMarkers,
    draw_axes: bool = True,
) -> list:
    """
    Estimate ArUco marker poses from an RGB image.

    rgb_frame:
        Frame from the RGB camera stream.

    depth_frame:
        Optional depth frame. Can be None.

    aruco_detector:
        Either a new OpenCV cv2.aruco.ArucoDetector object,
        or a tuple (aruco_dictionary, aruco_parameters) for old OpenCV.

    marker_size:
        Physical marker size in meters.

    matrix_coefficients:
        Camera intrinsic matrix.

    distortion_coefficients:
        Camera distortion coefficients.

    pose_array:
        PoseArray message to be filled.

    markers:
        ArucoMarkers message to be filled.

    returns:
        frame_processed, pose_array, markers
    """
    logger = rcutils_logger.RcutilsLogger(name="aruco_node")

    frame_processed = rgb_frame.copy()

    try:
        corners, marker_ids, rejected = detect_markers_compatible(
            rgb_frame,
            aruco_detector,
        )
    except Exception as e:
        logger.warn(f"ArUco marker detection failed: {e}")
        return frame_processed, pose_array, markers

    if marker_ids is None or len(corners) == 0:
        return frame_processed, pose_array, markers

    logger.debug(f"Detected {len(corners)} markers.")

    try:
        frame_processed = aruco_display(
            corners=corners,
            ids=marker_ids,
            image=frame_processed,
        )
    except Exception as e:
        logger.warn(f"Could not draw detected marker boxes: {e}")

    for i, marker_id in enumerate(marker_ids):
        marker_id_int = int(marker_id[0])

        try:
            tvec, rvec, quat = my_estimatePoseSingleMarkers(
                corners=corners[i],
                marker_size=marker_size,
                camera_matrix=matrix_coefficients,
                distortion=distortion_coefficients,
            )

        except Exception as e:
            logger.warn(
                f"Skipping marker {marker_id_int} because pose estimation failed: {e}"
            )
            continue

        if draw_axes:
            try:
                frame_processed = cv2.drawFrameAxes(
                    image=frame_processed,
                    cameraMatrix=matrix_coefficients,
                    distCoeffs=distortion_coefficients,
                    rvec=rvec,
                    tvec=tvec,
                    length=0.05,
                    thickness=3,
                )

            except Exception as e:
                logger.warn(f"Could not draw axes for marker {marker_id_int}: {e}")

        centroid = None

        if depth_frame is not None:
            try:
                centroid = depth_to_pointcloud_centroid(
                    depth_image=depth_frame,
                    intrinsic_matrix=matrix_coefficients,
                    corners=corners[i],
                )

                logger.info(f"depthcloud centroid = {centroid}")
                logger.info(f"tvec = {tvec[0]} {tvec[1]} {tvec[2]}")

            except Exception as e:
                logger.warn(f"Depth centroid failed for marker {marker_id_int}: {e}")
                centroid = None

        pose = Pose()

        if depth_frame is not None and centroid is not None:
            pose.position.x = float(centroid[0])
            pose.position.y = float(centroid[1])
            pose.position.z = float(centroid[2])

        else:
            pose.position.x = float(tvec[0])
            pose.position.y = float(tvec[1])
            pose.position.z = float(tvec[2])

        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])

        pose_array.poses.append(pose)
        markers.poses.append(pose)
        markers.marker_ids.append(marker_id_int)

    return frame_processed, pose_array, markers


def my_estimatePoseSingleMarkers(
    corners,
    marker_size,
    camera_matrix,
    distortion,
) -> tuple[np.array, np.array, np.array]:
    """
    Estimate rvec and tvec for one detected ArUco marker.

    corners:
        Detected corners for one marker.

    marker_size:
        Physical marker size in meters.

    camera_matrix:
        Camera intrinsic matrix.

    distortion:
        Camera distortion coefficients.

    returns:
        tvec, rvec, quaternion [x, y, z, w]
    """
    marker_points = np.array(
        [
            [-marker_size / 2.0, marker_size / 2.0, 0.0],
            [marker_size / 2.0, marker_size / 2.0, 0.0],
            [marker_size / 2.0, -marker_size / 2.0, 0.0],
            [-marker_size / 2.0, -marker_size / 2.0, 0.0],
        ],
        dtype=np.float32,
    )

    image_points = np.asarray(corners, dtype=np.float32)

    if image_points.shape == (1, 4, 2):
        image_points = image_points.reshape((4, 2))

    retval, rvec, tvec = cv2.solvePnP(
        objectPoints=marker_points,
        imagePoints=image_points,
        cameraMatrix=camera_matrix,
        distCoeffs=distortion,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )

    if not retval:
        raise RuntimeError("cv2.solvePnP failed")

    rvec = rvec.reshape(3, 1)
    tvec = tvec.reshape(3, 1)

    rotation_3x3, _ = cv2.Rodrigues(rvec)
    quaternion = quaternion_from_rotation_matrix_3x3(rotation_3x3)

    return tvec, rvec, quaternion


def depth_to_pointcloud_centroid(
    depth_image: np.array,
    intrinsic_matrix: np.array,
    corners: np.array,
) -> np.array:
    """
    Convert marker depth pixels into a 3D centroid.

    depth_image:
        2D depth image.

    intrinsic_matrix:
        Camera intrinsic matrix.

    corners:
        Detected marker corners with shape (1, 4, 2).

    returns:
        np.array([x, y, z])
    """
    height, width = depth_image.shape

    corners_indices = np.array(
        [(int(x), int(y)) for x, y in corners[0]],
        dtype=np.int32,
    )

    for x, y in corners_indices:
        if x < 0 or x >= width or y < 0 or y >= height:
            raise ValueError("One or more corners are outside the image bounds.")

    x_min = int(min(corners_indices[:, 0]))
    x_max = int(max(corners_indices[:, 0]))
    y_min = int(min(corners_indices[:, 1]))
    y_max = int(max(corners_indices[:, 1]))

    points = []

    for x in range(x_min, x_max):
        for y in range(y_min, y_max):
            if is_pixel_in_polygon(pixel=(x, y), corners=corners_indices):
                depth_value = depth_image[y, x]

                if depth_value > 0 and np.isfinite(depth_value):
                    points.append([x, y, depth_value])

    if len(points) == 0:
        raise ValueError("No valid depth pixels found inside marker polygon.")

    points = np.array(points, dtype=np.float64)

    pointcloud = []

    fx = intrinsic_matrix[0, 0]
    fy = intrinsic_matrix[1, 1]
    cx = intrinsic_matrix[0, 2]
    cy = intrinsic_matrix[1, 2]

    for x, y, d in points:
        z = d / 1000.0
        x_3d = (x - cx) * z / fx
        y_3d = (y - cy) * z / fy
        pointcloud.append([x_3d, y_3d, z])

    centroid = np.mean(np.array(pointcloud, dtype=np.float64), axis=0)

    return centroid


def is_pixel_in_polygon(pixel: tuple, corners: np.array) -> bool:
    """
    Return True if pixel is inside polygon defined by corners.
    Uses ray casting.
    """
    num_intersections = 0

    for i in range(len(corners)):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % len(corners)]

        if (y1 <= pixel[1] < y2) or (y2 <= pixel[1] < y1):
            x_intersection = (x2 - x1) * (pixel[1] - y1) / (y2 - y1) + x1

            if x_intersection > pixel[0]:
                num_intersections += 1

    return num_intersections % 2 == 1
