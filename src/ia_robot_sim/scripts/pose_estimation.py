# Copyright (c) 2025, Intuitive Autonomy. All rights reserved.
#
# Intuitive Autonomy and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation,
# and any modifications thereto. Any use, reproduction, disclosure, or
# distribution of this software and related documentation without an express
# license agreement from Intuitive Autonomy is strictly prohibited.
#
# Author: Greaten
# Licensed under the MIT License.

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from argparse import ArgumentParser
from calculate_handle_position import *
import matplotlib.image as mpimg
import cv2
from typing import List, Tuple, Dict

#################################################################
from scipy.spatial.transform import Rotation as R

from scipy.interpolate import NearestNDInterpolator
######################################################################

from visualization import *
from scipy.optimize import minimize
################################
import mediapipe as mp

map_2d_to_3d = {0: 9, 15: 8, 2: 14, 3: 15, 4: 16, 5: 11, 6: 12,
                7: 13, 8: 0, 9: 1, 10: 2, 11: 3, 12: 4, 13: 5, 14: 6}
map_3d_to_2d = {9: 0, 8: 15, 11: 2, 12: 3, 13: 4, 14: 5, 15: 6,
                16: 7, 0: 8, 4: 9, 5: 10, 6: 11, 1: 12, 2: 13, 3: 14}

# K = np.array([605.9083251953125, 0, 315.4358825683594,
#               0, 605.8673706054688, 253.82937622070312,
#               0, 0, 1]).reshape(3, 3)

K = np.array([
    [640.0,   0.0, 640.0],
    [  0.0, 493.4, 360.0],
    [  0.0,   0.0,   1.0]
])

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720

def depth_lift(keypoint: np.ndarray, depth: np.ndarray, K: np.ndarray) -> np.ndarray:
    """AI is creating summary for depth_lift

    Args:
        keypoint (np.ndarray): [description]
        depth (np.ndarray): [description]
        K (np.ndarray): [description]

    Returns:
        np.ndarray: [description]
    """
    u, v = keypoint
    u = np.clip(u, 0, IMAGE_HEIGHT-1)
    v = np.clip(v, 0, IMAGE_HEIGHT-1)
    z = depth[int(v), int(u)]
    # print("u", u, "v", v, "depth", z)
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    y = -y
    y, z = z, y
    return np.array([x, y, z])


def normalize_arm(test_scene: Dict):
    l = np.linalg.norm(test_scene['measurements'][2]) + \
        np.linalg.norm(test_scene['measurements'][3])
    l *= 0.5
    test_scene["measurements"][5] = test_scene["measurements"][5] * \
        l/np.linalg.norm(test_scene["measurements"][5])
    test_scene["measurements"][6] = test_scene["measurements"][6] * \
        l/np.linalg.norm(test_scene["measurements"][6])

def get_3d_keypoints(keypoints2d: np.ndarray, depth: np.ndarray, K: np.ndarray) -> List[List[float]]:
    """Computes 3D keypoints from 2D keypoints using depth information and camera intrinsic matrix.

    Args:
        keypoints2d (np.ndarray): 2D keypoints array of shape (N, 2), where N is the number of keypoints.
        depth (np.ndarray): Depth map of shape (H, W).
        K (np.ndarray): Camera intrinsic matrix of shape (3, 3).

    Returns:
        list[list[float]]: A list of 3D keypoints, each represented as [x, y, z].
    """
    points: list[list[float]] = [[] for _ in range(17)]

    for i, j in map_2d_to_3d.items():
        u, v = keypoints2d[i]

        u = np.clip(u, 0, IMAGE_WIDTH-1)
        v = np.clip(v, 0, IMAGE_HEIGHT-1)

        z = depth[int(v), int(u)]

        x = (u - K[0, 2]) * z / K[0, 0]
        y = (v - K[1, 2]) * z / K[1, 1]
        points[j] = [x, y, z]

    points[10] = points[9]
    points[7] = [
        0.5 * (points[0][0] + points[8][0]),
        0.5 * (points[0][1] + points[8][1]),
        0.5 * (points[0][2] + points[8][2])
    ]

    return points


def depth_inpainting(depth: np.ndarray) -> np.ndarray:
    """AI is creating summary for depth_inpainting

    Args:
        depth (np.ndarray): [description]

    Returns:
        np.ndarray: [description]
    """
    mask = np.where(depth > 0)
    if mask[0].shape[0] != 0:
        interp = NearestNDInterpolator(np.transpose(mask), depth[mask])
        depth = interp(*np.indices(depth.shape))
    return depth


def get_medium_depth(keypoints2d: np.ndarray, depth: np.ndarray) -> float:
    """AI is creating summary for get_medium_depth

    Args:
        keypoints2d (np.ndarray): [description]
        depth (np.ndarray): [description]

    Returns:
        float: [description]
    """
    depth_values = []
    for i in range(len(keypoints2d)):
        u, v = keypoints2d[i]
        u = np.clip(u, 0, IMAGE_WIDTH-1)
        v = np.clip(v, 0, IMAGE_HEIGHT-1)
        depth_values.append(depth[int(v), int(u)])
    return np.median(depth_values)


def get_average_depth(keypoints2d: np.ndarray, depth: np.ndarray) -> float:
    """AI is creating summary for get_average_depth
    Args:
        keypoints2d (np.ndarray): [description]
        depth (np.ndarray): [description]
    Returns:
        float: [description]
    """
    depth_values = []
    for i in range(len(keypoints2d)):
        u, v = keypoints2d[i]
        u = np.clip(u, 0, IMAGE_WIDTH-1)
        v = np.clip(v, 0, IMAGE_HEIGHT-1)
        depth_values.append(depth[int(v), int(u)])
    return np.mean(depth_values)


def compute_error(keypoints3d: np.ndarray, keypoints2d_coco: np.ndarray, K: np.ndarray) -> float:
    """Compute the reprojection error between 3D keypoints and their 2D projections.

    Args:
        keypoints3d (np.ndarray): Array of shape (N, 3) containing 3D keypoints (x, y, z)
        keypoints2d (np.ndarray): Array of shape (N, 2) containing 2D keypoints (u, v)
        K (np.ndarray): 3x3 camera intrinsic matrix

    Returns:
        float: Average reprojection error
    """
    # Extract x, y, z coordinates
    keypoints2d = np.zeros((17, 2))
    for i, j in map_2d_to_3d.items():
        keypoints2d[j] = keypoints2d_coco[i]
    keypoints2d[10] = keypoints2d[9]
    keypoints2d[7] = 0.5 * (keypoints2d[0] + keypoints2d[8])
    pixel_points = project_points_to_pixel(keypoints3d, K)

    return np.mean(np.sqrt(np.sum((pixel_points - keypoints2d)**2, axis=1)))


def solve_alignment_problem(keypoints3d: np.ndarray, keypoints2d: np.ndarray, depth: np.ndarray, K: np.ndarray) -> np.ndarray:
    """Align 3D keypoints to match 2D keypoints by finding optimal scale and translation.

    Args:
        keypoints3d (np.ndarray): 3D keypoints array of shape (n, 3)
        keypoints2d (np.ndarray): 2D keypoints array of shape (n, 2)
        depth (np.ndarray): Depth map array
        K (np.ndarray): Camera intrinsic matrix (3x3)

    Returns:
        np.ndarray: Aligned 3D keypoints array of shape (n, 3)
    """
    dm = get_medium_depth(keypoints2d, depth)
    keypoints2d_coco = convert_coco_to_mpii(keypoints2d)
    keypoints2d = np.zeros((17, 2))
    for i, j in map_2d_to_3d.items():
        keypoints2d[j] = keypoints2d_coco[i]
    keypoints2d[10] = keypoints2d[9]
    keypoints2d[7] = 0.5*(keypoints2d[0]+keypoints2d[8])

    def func(params):
        s, dx, dy = params
        # Scale relative to first point
        keypoints3d_scaled = keypoints3d[0] + \
            s * (keypoints3d - keypoints3d[0])
        # Apply translation
        keypoints3d_translated = keypoints3d_scaled + np.array([dx, dm, dy])
        # Project to 2D
        pixel_points = project_points_to_pixel(keypoints3d_translated, K)
        # Calculate mean squared error
        return np.mean(np.sqrt(np.sum((pixel_points - keypoints2d)**2, axis=1)))

    # Initial guess for s, dx, dy
    initial_params = [1.0, 0.0, 0.0]

    # Optimize parameters
    result = minimize(func, initial_params, method='Nelder-Mead')

    # Get optimal parameters
    s_opt, dx_opt, dy_opt = result.x

    # Apply optimal transformation to keypoints
    keypoints3d_aligned = keypoints3d[0] + \
        s_opt * (keypoints3d - keypoints3d[0])
    keypoints3d_aligned = keypoints3d_aligned + np.array([dx_opt, dm, dy_opt])

    return keypoints3d_aligned


def alignment_dumb(keypoints: np.ndarray, keypoints2d: np.ndarray, depth_image: np.ndarray, K: np.ndarray) -> np.ndarray:
    """AI is creating summary for alignment_dumb

    Args:
        keypoints (np.ndarray): [description]
        keypoints2d (np.ndarray): [description]
        depth_image (np.ndarray): [description]
        K (np.ndarray): [description]

    Returns:
        np.ndarray: [description]
    """
    transform = compute_transform(keypoints2d, depth_image, keypoints, K)
    keypoints = apply_scale(keypoints, transform["scale"])
    keypoints = keypoints + transform["translation"]
    return keypoints


def convert_coco_to_mpii(coco_keypoints: np.ndarray) -> np.ndarray:
    """
    Convert keypoints from COCO format (17 keypoints) to MPII format (16 keypoints).

    COCO keypoints:
    0: nose, 1: left eye, 2: right eye, 3: left ear, 4: right ear,
    5: left shoulder, 6: right shoulder, 7: left elbow, 8: right elbow,
    9: left wrist, 10: right wrist, 11: left hip, 12: right hip,
    13: left knee, 14: right knee, 15: left ankle, 16: right ankle

    MPII keypoints:
    0: head_top, 1: neck, 2: right_shoulder, 3: right_elbow, 4: right_wrist,
    5: left_shoulder, 6: left_elbow, 7: left_wrist, 8: hip_center, 
    9: right_hip, 10: right_knee, 11: right_ankle,
    12: left_hip, 13: left_knee, 14: left_ankle, 15: thorax

    Args:
        coco_keypoints: numpy array of shape (17, 2) or (17, 3) for 2D or 3D keypoints

    Returns:
        mpii_keypoints: numpy array of shape (16, 2) or (16, 3)
    """
    # Create empty array for MPII keypoints
    dim = coco_keypoints.shape[1]  # Get dimension (2D or 3D)
    mpii_keypoints = np.zeros((16, dim))

    # Direct mappings from COCO to MPII
    # Right side
    mpii_keypoints[2] = coco_keypoints[6]  # right shoulder
    mpii_keypoints[3] = coco_keypoints[8]  # right elbow
    mpii_keypoints[4] = coco_keypoints[10]  # right wrist
    mpii_keypoints[9] = coco_keypoints[12]  # right hip
    mpii_keypoints[10] = coco_keypoints[14]  # right knee
    mpii_keypoints[11] = coco_keypoints[16]  # right ankle

    # Left side
    mpii_keypoints[5] = coco_keypoints[5]  # left shoulder
    mpii_keypoints[6] = coco_keypoints[7]  # left elbow
    mpii_keypoints[7] = coco_keypoints[9]  # left wrist
    mpii_keypoints[12] = coco_keypoints[11]  # left hip
    mpii_keypoints[13] = coco_keypoints[13]  # left knee
    mpii_keypoints[14] = coco_keypoints[15]  # left ankle

    # Derived keypoints
    # Neck - average of shoulders
    mpii_keypoints[1] = (coco_keypoints[5] + coco_keypoints[6]) / 2

    # Thorax - similar to neck but can be adjusted
    mpii_keypoints[15] = mpii_keypoints[1]

    # Hip center - average of hips
    mpii_keypoints[8] = (coco_keypoints[11] + coco_keypoints[12]) / 2

    # Head top - extrapolate from nose and neck
    if False:  # If nose is available
        neck_to_nose = coco_keypoints[0] - mpii_keypoints[1]
        mpii_keypoints[0] = coco_keypoints[0] + neck_to_nose
    else:
        # Fallback: use eyes to estimate head position
        # avg of left and right eye
        eyes_center = (coco_keypoints[1] + coco_keypoints[2]) / 2
        mpii_keypoints[0] = eyes_center
        # if not np.all(eyes_center == 0):
        #     eyes_to_nose_factor = 0.5  # Adjustable factor
        #     neck_to_eyes = eyes_center - mpii_keypoints[1]
        #     mpii_keypoints[0] = eyes_center + \
        #         neck_to_eyes * eyes_to_nose_factor

    return mpii_keypoints


def fit_plane_to_points(points: np.ndarray, weights: np.ndarray = None) -> np.ndarray:
    """
    Find the plane that minimizes the weighted sum of squared distances to a set of 3D points.

    Parameters:
    -----------
    points : np.ndarray
        Array of shape (n, 3) containing the 3D points.
    weights : np.ndarray, optional
        Array of shape (n,) containing the weights for each point.
        If None, all points are weighted equally.

    Returns:
    --------
    tuple
        (a, b, c, d) coefficients of the plane equation ax + by + cz + d = 0
        The normal vector (a, b, c) is normalized to have unit length.
    """
    if weights is None:
        weights = np.ones(len(points))

    # Normalize weights
    weights = weights / np.sum(weights)

    # Calculate the weighted centroid
    centroid = np.sum(points * weights[:, np.newaxis], axis=0)

    # Center the points around the weighted centroid
    centered_points = points - centroid

    # Calculate the weighted covariance matrix
    cov_matrix = np.zeros((3, 3))
    for i in range(len(points)):
        p = centered_points[i, :].reshape(3, 1)
        cov_matrix += weights[i] * (p @ p.T)

    # Find the eigenvector corresponding to the smallest eigenvalue
    eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
    normal = eigenvectors[:, 0]  # Smallest eigenvalue's eigenvector

    # Ensure normal vector has unit length
    normal = normal / np.linalg.norm(normal)

    # Calculate d in the plane equation ax + by + cz + d = 0
    d = -np.dot(normal, centroid)

    return np.array([normal[0], normal[1], normal[2], d])


def distance_point_to_plane(point: np.ndarray, plane_params: List[float]) -> float:
    """
    Calculate the signed distance from a point to a plane.

    Parameters:
    -----------
    point : np.ndarray
        Array of shape (3,) containing the 3D point.
    plane_params : tuple
        (a, b, c, d) coefficients of the plane equation ax + by + cz + d = 0

    Returns:
    --------
    float
        Signed distance from the point to the plane.
    """
    a, b, c, d = plane_params
    numerator = a * point[0] + b * point[1] + c * point[2] + d
    denominator = np.sqrt(a**2 + b**2 + c**2)
    return numerator / denominator


def mean_squared_error(points: np.ndarray, weights: np.ndarray, plane_params: List[float]) -> float:
    """
    Calculate the weighted mean squared error between points and a plane.

    Parameters:
    -----------
    points : np.ndarray
        Array of shape (n, 3) containing the 3D points.
    weights : np.ndarray
        Array of shape (n,) containing the weights for each point.
    plane_params : tuple
        (a, b, c, d) coefficients of the plane equation ax + by + cz + d = 0

    Returns:
    --------
    float
        Weighted mean squared error.
    """
    distances = np.array(
        [distance_point_to_plane(p, plane_params) for p in points])
    return np.sum(weights * distances**2) / np.sum(weights)


def demonstrate_weighted_plane_fitting():
    # Generate some example points
    np.random.seed(42)
    n_points = 100

    # Create a random plane
    true_normal = np.array([1, 2, 3])
    true_normal = true_normal / np.linalg.norm(true_normal)
    true_d = -5

    # Generate random points near the plane
    base_points = np.random.randn(n_points, 3)

    # Project points onto the plane
    dots = np.dot(base_points, true_normal)
    projections = base_points - np.outer(dots, true_normal)

    # Add some noise
    noise = np.random.normal(0, 0.2, (n_points, 3))
    noisy_points = projections - true_d * \
        true_normal / np.sum(true_normal**2) + noise

    # Assign different weights to the points
    # Points closer to the true plane get higher weights in this example
    # In a real scenario, weights would come from your specific application
    distances = np.abs(np.dot(noisy_points, true_normal) +
                       true_d) / np.linalg.norm(true_normal)
    weights = 1 / (1 + distances)

    # Fit the plane
    coronal_plane = fit_plane_to_points(noisy_points, weights)

    # Calculate error
    error = mean_squared_error(noisy_points, weights, coronal_plane)

    print(
        f"True plane: {true_normal[0]:.4f}x + {true_normal[1]:.4f}y + {true_normal[2]:.4f}z + {true_d:.4f} = 0")
    print(
        f"Estimated plane: {coronal_plane[0]:.4f}x + {coronal_plane[1]:.4f}y + {coronal_plane[2]:.4f}z + {coronal_plane[3]:.4f} = 0")
    print(f"Weighted mean squared error: {error:.6f}")

    return noisy_points, weights, coronal_plane


def check_validity(keypoints3d: np.ndarray) -> bool:
    """AI is creating summary for check_validity

    Args:
        keypoints3d (np.ndarray): [description]

    Returns:
        bool: [description]
    """
    threshold = 0.1
    distance_shoulder = np.linalg.norm(
        keypoints3d[11] - keypoints3d[14])  # left shoulder to right shoulder
    distance_hip = np.linalg.norm(
        keypoints3d[1] - keypoints3d[4])  # left hip to right hip
    print(f"distance_shoulder: {distance_shoulder}")
    print(f"distance_hip: {distance_hip}")
    if distance_hip < threshold or distance_shoulder < threshold:
        return False
    return True


def compute_center_of_mass(points: np.ndarray, weights: np.ndarray) -> np.ndarray:
    """
    Compute the center of mass of a set of 3D points with weights.

    Parameters:
    -----------
    points : np.ndarray
        Array of shape (n, 3) containing the 3D points.
    weights : np.ndarray
        Array of shape (n,) containing the weights for each point.

    Returns:
    --------
    np.ndarray
        Array of shape (3,) containing the center of mass.
    """
    total_weight = np.sum(weights)
    center_of_mass = np.sum(
        points * weights[:, np.newaxis], axis=0) / total_weight
    return center_of_mass


def get_human_bounding_box(keypoints3d: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """_summary_

    Args:
        keypoints3d (np.ndarray): _description_

    Returns:
        Tuple[np.ndarray, np.ndarray]: _description_
    """
    # Calculate the minimum and maximum coordinates along each axis
    min_coords = np.min(keypoints3d, axis=0)
    max_coords = np.max(keypoints3d, axis=0)

    # Create the bounding box as two points: (min_x, min_y, min_z) and (max_x, max_y, max_z)
    bounding_box_min = min_coords
    bounding_box_max = max_coords

    return bounding_box_min, bounding_box_max


def find_perpendicular_plane_through_points(coronal_plane: np.ndarray, point1: np.ndarray, point2: np.ndarray):
    """
    Find a plane that passes through two points and is perpendicular to a given plane.

    Parameters:
    -----------
    coronal_plane : tuple
        (a, b, c, d) coefficients of the estimated plane equation ax + by + cz + d = 0
    point1 : np.ndarray
        First point that the new plane should pass through (shape: (3,))
    point2 : np.ndarray
        Second point that the new plane should pass through (shape: (3,))

    Returns:
    --------
    tuple
        (a, b, c, d) coefficients of the perpendicular plane equation ax + by + cz + d = 0
    """
    # Extract the normal vector of the estimated plane
    a1, b1, c1, _ = coronal_plane
    estimated_normal = np.array([a1, b1, c1])

    # Step 1: Find a vector that lies in the perpendicular plane
    # This is simply the vector connecting the two points
    connecting_vector = point2 - point1

    # Step 2: The normal vector of the perpendicular plane must be:
    # 1) Perpendicular to the connecting vector (to ensure the plane contains both points)
    # 2) Perpendicular to the normal of the estimated plane

    # We find this by taking the cross product of the estimated plane's normal
    # and the connecting vector
    perpendicular_normal = np.cross(estimated_normal, connecting_vector)

    # If the two vectors are parallel, this cross product will be zero or very small
    if np.linalg.norm(perpendicular_normal) < 1e-10:
        raise ValueError(
            "The connecting vector is parallel to the estimated plane's normal. No unique perpendicular plane exists.")

    # Normalize the normal vector
    perpendicular_normal = perpendicular_normal / \
        np.linalg.norm(perpendicular_normal)

    # Step 3: Calculate the d coefficient for the perpendicular plane equation
    # Using the point-normal form of the plane equation: ax + by + cz + d = 0
    # Therefore: d = -(ax + by + cz) where (x,y,z) is a point on the plane
    d = -np.dot(perpendicular_normal, point1)

    return (perpendicular_normal[0], perpendicular_normal[1], perpendicular_normal[2], d)


def project_points_to_plane(points: np.ndarray, plane: np.ndarray, fixed_norm=None) -> np.ndarray:
    """AI is creating summary for project_points_to_plane

    Args:
        points (np.ndarray): [description]
        plane (np.ndarray): [description]

    Returns:
        np.ndarray: [description]
    """
    a, b, c, d = plane

    # Define the plane normal
    normal = np.array([a, b, c])
    normal = normal / np.linalg.norm(normal)  # Normalize the normal vector
    print(f"Plane normal: {normal}")
    # Find a point on the plane (assuming z=0 gives (x, y, 0))
    if c != 0:
        point_on_plane = np.array([0, 0, -d / c])
    elif b != 0:
        point_on_plane = np.array([0, -d / b, 0])
    else:
        point_on_plane = np.array([-d / a, 0, 0])
    print(f"Point on plane: {point_on_plane}")
    # Construct a coordinate system for the plane
    # The new y-axis is the projection of the original z-axis onto the plane
    new_y = np.array([0, 0, 1]) - np.dot(np.array([0, 0, 1]), normal) * normal
    print(f"New y-axis: {new_y}")
    new_y /= np.linalg.norm(new_y)

    # Choose new_x as perpendicular to both new_y and normal
    new_x = np.cross(new_y, normal)
    new_x /= np.linalg.norm(new_x)
    print(f"New x-axis: {new_x}")
    if fixed_norm is not None:
        new_x = fixed_norm

    # Projection matrix to transform 3D points to 2D plane coordinates
    transformation_matrix = np.vstack([new_x, new_y])  # 2x3 matrix

    # Project points onto the plane along the normal direction
    projected_points = []
    for p in points:
        p = np.array(p)
        # Find projection on the plane
        dist_to_plane = np.dot((p - point_on_plane), normal)
        projected_p = p - dist_to_plane * normal

        # Convert to 2D coordinates
        p_2d = transformation_matrix @ (projected_p - point_on_plane)
        projected_points.append(p_2d)

    return np.array(projected_points)


def recover_3d_from_2d(plane: np.ndarray, p_2d: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """AI is creating summary for recover_3d_from_2d

    Args:
        plane (np.ndarray): [description]
        p_2d (np.ndarray): [description]

    Returns:
        Tuple[np.ndarray, np.ndarray]: [description]
    """
    a, b, c, d = plane
    # Define the plane normal
    normal = np.array([a, b, c])
    normal = normal / np.linalg.norm(normal)  # Normalize the normal vector

    # Find a point on the plane
    if c != 0:
        point_on_plane = np.array([0, 0, -d / c])
    elif b != 0:
        point_on_plane = np.array([0, -d / b, 0])
    else:
        point_on_plane = np.array([-d / a, 0, 0])

    # points in the sagittal plane coordinates
    p_3d = np.array([p_2d[0], p_2d[1], 0])

    # Construct a coordinate system for the plane
    # The new y-axis is the projection of the original z-axis onto the plane
    new_y = np.array([0, 0, 1]) - np.dot(np.array([0, 0, 1]), normal) * normal
    new_y /= np.linalg.norm(new_y)

    # Choose new_x as perpendicular to both new_y and normal
    new_x = np.cross(new_y, normal)
    new_x /= np.linalg.norm(new_x)

    # Projection matrix to transform 3D points to 2D plane coordinates
    transformation_matrix = np.vstack([new_x, new_y, normal])  # 3x3 matrix

    transformation_matrix = np.linalg.inv(transformation_matrix)

    p_3d = transformation_matrix @ p_3d + point_on_plane

    orientation = normal
    return p_3d, orientation


def vector_to_quaternion(vector: np.ndarray, reference: np.ndarray = np.array([1, 0, 0])) -> np.ndarray:
    """Returns a quaternion that rotates the reference vector to the given vector direction.

    Args:
        vector (np.ndarray): Target direction vector (3D).
        reference (np.ndarray): Reference direction vector (3D), default is [1, 0, 0].

    Returns:
        np.ndarray: Quaternion [x, y, z, w] that rotates reference to vector.
    """
    vector = np.array(vector, dtype=np.float64)
    reference = np.array(reference, dtype=np.float64)
    
    norm_v = np.linalg.norm(vector)
    norm_r = np.linalg.norm(reference)

    if norm_v == 0 or norm_r == 0:
        return np.array([0, 0, 0, 1])  # Identity quaternion

    v1 = reference / norm_r
    v2 = vector / norm_v

    if np.allclose(v1, v2):
        return np.array([0, 0, 0, 1])  # No rotation needed
    if np.allclose(v1, -v2):
        # 180 degree rotation around any orthogonal axis
        ortho = np.array([1, 0, 0]) if not np.allclose(v1, [1, 0, 0]) else np.array([0, 1, 0])
        axis = np.cross(v1, ortho)
        axis = axis / np.linalg.norm(axis)
        return R.from_rotvec(np.pi * axis).as_quat()

    axis = np.cross(v1, v2)
    angle = np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))
    quat = R.from_rotvec(axis / np.linalg.norm(axis) * angle).as_quat()
    return quat

def compute_vcom(scene: Dict, link_masses: Dict):
    """AI is creating summary for compute_vcom

    Args:
        scene (dict): [description]
        link_masses (dict): [description]
        vis (bool, optional): [description]. Defaults to False.

    Returns:
        [type]: [description]
    """
    center_of_mass_from_origin = first_5_center_of_mass(scene, link_masses)
    weighted_average_x = 0
    weighted_average_y = 0
    points = scene['measurements']
    sum_l = 0
    for i in range(1, 6):
        li = np.sqrt(points[i-1][0]**2+points[i-1][1]**2)
        weighted_average_x += 0
        weighted_average_y += ((li/2)+sum_l)*link_masses[i-1]
        sum_l += li

    vec = np.array([weighted_average_x, weighted_average_y]) - \
        np.array(center_of_mass_from_origin)
    print(f"computed v_com: {vec}")
    vec[0], vec[1] = vec[1],  -vec[0]
    return vec/np.linalg.norm(vec)

# Define key event function


def get_human_orientation(scene: Dict):
    """
    Compute the human face orientation given 2D keypoints in the sagittal plane.

    Args:
        points (np.ndarray): A (N, 2) array representing 2D keypoints (x, y).

    Returns:
        np.ndarray: A unit vector indicating the face orientation.
    """
    # Define indices for neck and head (adjust based on keypoint format)
    flip = False
    IDX = 4
    direction = scene["measurements"][IDX]
    if direction[0] > 0:
        flip = True

    return flip


def compute_com_3d(keypoints3d: np.ndarray, weights: np.ndarray) -> np.ndarray:
    """
    Compute the center of mass (COM) of a set of 3D keypoints weighted by given weights.

    Args:
        keypoints3d (np.ndarray): Array of shape (N, 3) representing N 3D keypoints.
        weights (np.ndarray): Array of shape (N,) representing the corresponding weights for each keypoint.

    Returns:
        np.ndarray: A (3,) array representing the weighted center of mass.
    """
    # Ensure weights sum to 1 for proper weighting
    total_weight = np.sum(weights)
    if total_weight == 0:
        return np.zeros(3)  # Avoid division by zero

    # Compute the weighted center of mass
    com = np.sum(keypoints3d * weights[:, np.newaxis], axis=0) / total_weight
    return com


def apply_scale(keypoints3d: np.ndarray, scale: float) -> np.ndarray:
    """Apply scaling transformation to 3D keypoints around their center of mass.

    Args:
        keypoints3d (np.ndarray): Array of 3D keypoints with shape (N, 3) where N is the number of keypoints
        weights (np.ndarray): Array of weights for each keypoint with shape (N,)
        scale (float): Scaling factor to apply

    Returns:
        np.ndarray: Scaled 3D keypoints with the same shape as input
    """

    weights = np.array([1000, 1000, 50, 20, 1000, 50,
                        20, 1000, 1000, 5, 5, 1000, 5, 5, 1000, 5, 5])
    # Ensure weights sum to 1 for weighted center calculation
    normalized_weights = weights / np.sum(weights)

    # Calculate the weighted center of mass (shape: (3,))
    center_of_mass = np.sum(
        keypoints3d * normalized_weights[:, np.newaxis], axis=0)

    # Shift points to origin by subtracting center of mass
    centered_keypoints = keypoints3d - center_of_mass

    # Apply scaling
    scaled_keypoints = centered_keypoints * scale

    # Shift back to original position
    scaled_keypoints_original_pos = scaled_keypoints + center_of_mass

    return scaled_keypoints_original_pos


def compute_transform(keypoints2d: np.ndarray, depth_image: np.ndarray, keypoints3d: np.ndarray, K: np.ndarray) -> Dict:
    """AI is creating summary for compute_transform

    Args:
        keypoints2d (np.ndarray): [description]
        depth_image (np.ndarray): [description]
        weights2d (np.ndarray): [description]
        keypoints3d (np.ndarray): [description]
        K (np.ndarray): [description]

    Returns:
        Dict: [description]
    """

    weights2d = 1*np.ones(17)
    indices = np.array([0, 5, 6, 12, 11, 14, 13])
    weights2d[indices] = 1000
    weights3d = np.array([1000, 1000, 50, 20, 1000, 50,
                          20, 1000, 1000, 5, 5, 1000, 5, 5, 1000, 5, 5])
    medium_depth = get_medium_depth(keypoints2d, depth_image)
    xmin_2d = keypoints2d[:, 0].min()
    xmax_2d = keypoints2d[:, 0].max()
    ymin_2d = keypoints2d[:, 1].min()
    ymax_2d = keypoints2d[:, 1].max()
    xmin_2d = (xmin_2d-K[0][2])/K[0][0]*medium_depth
    xmax_2d = (xmax_2d-K[0][2])/K[0][0]*medium_depth
    ymin_2d = (ymin_2d-K[1][2])/K[1][1]*medium_depth
    ymax_2d = (ymax_2d-K[1][2])/K[1][1]*medium_depth
    center_x = np.dot(weights2d, keypoints2d[:, 0])/np.sum(weights2d)
    center_y = np.dot(weights2d, keypoints2d[:, 1])/np.sum(weights2d)
    center_x = (center_x-K[0][2])/K[0][0]*medium_depth
    center_y = (center_y-K[1][2])/K[1][1]*medium_depth

    xmin_3d = keypoints3d[:, 0].min()
    xmax_3d = keypoints3d[:, 0].max()
    zmin_3d = keypoints3d[:, 2].min()
    zmax_3d = keypoints3d[:, 2].max()
    center_x_3d = np.dot(weights3d, keypoints3d[:, 0])/np.sum(weights3d)
    center_y_3d = np.dot(weights3d, keypoints3d[:, 1])/np.sum(weights3d)
    center_z_3d = np.dot(weights3d, keypoints3d[:, 2])/np.sum(weights3d)
    scale = (ymax_2d-ymin_2d)*(xmax_2d-xmin_2d) / \
        ((zmax_3d-zmin_3d)*(xmax_3d-xmin_3d))

    translation = np.array([center_x, medium_depth, -center_y]) - \
        np.array([center_x_3d, center_y_3d, center_z_3d])

    transform = {"translation": translation, "scale": scale}
    return transform


def get_3d_keypoints_2dndepth(pose2d_model, rgb_image: np.ndarray, depth_image: np.ndarray, K: np.ndarray, vis=False) -> np.ndarray:
    """AI is creating summary for get_3d_keypoints_2dndepth

    Args:
        pose2d_model (Pose2DInferencer): [description]
        rgb (np.ndarray): [description]
        depth (np.ndarray): [description]
        K (np.ndarray): [description]

    Returns:
        np.ndarray: [description]
    """
    results = next(pose2d_model(rgb_image))['predictions']

    # print(results)
    keypoints2d_coco = np.array(results[0][0]['keypoints'])  # (N, 2) array
    keypoints2d_scores = np.array(
        results[0][0]['keypoint_scores'])  # (N, 2) array
    keypoints2d_coco = convert_coco_to_mpii(keypoints2d_coco)

    keypoints2d_coco = np.array(keypoints2d_coco)
    if vis:
        show_rgbd_keypoints(rgb_image, keypoints2d_coco, depth_image)
    keypoints = get_3d_keypoints(keypoints2d_coco, depth_image, K)
    return keypoints


def mediapipe_to_mmpose3d(keypoints_media: np.ndarray) -> np.ndarray:
    """
    Convert MediaPipe 3D keypoints (33 x 3) to MMpose Human3.6M 3D format (17 x 3).
    All coordinates should be in the same units (e.g., meters).

    Args:
        keypoints_media (np.ndarray): 33x3 MediaPipe keypoints

    Returns:
        np.ndarray: 17x3 MMpose-style keypoints
    """
    assert keypoints_media.shape[0] == 33, "Expected 33 MediaPipe keypoints."

    # Human3.6M-style mapping from MediaPipe indices
    mapping = {
        0: 0,     # pelvis → pelvis
        1: 24,    # rhip
        2: 26,    # rknee
        3: 28,    # rankle
        4: 23,    # lhip
        5: 25,    # lknee
        6: 27,    # lankle
        7: (11, 12),  # spine: average of shoulders
        8: 23,    # thorax approx with lhip
        9: 1,     # neck
        10: 0,    # head top ~ nose
        11: 11,   # lshoulder
        12: 13,   # lelbow
        13: 15,   # lwrist
        14: 12,   # rshoulder
        15: 14,   # relbow
        16: 16,   # rwrist
    }

    keypoints_3d = np.zeros((17, 3), dtype=np.float32)

    for idx, mp_idx in mapping.items():
        if isinstance(mp_idx, tuple):
            keypoints_3d[idx] = (keypoints_media[mp_idx[0]] + keypoints_media[mp_idx[1]]) / 2.0
        else:
            keypoints_3d[idx] = keypoints_media[mp_idx]

    return keypoints_3d



def get_3d_keypoints_3dndepth(pose3d_model, rgb_image: np.ndarray, depth_image: np.ndarray, K: np.ndarray, vis=False) -> np.ndarray:
    """AI is creating summary for get_3d_keypoints3dndepth

    Args:
        pose3d_model ([type]): [description]
        rgb_image (np.ndarray): [description]
        depth_image (np.ndarray): [description]
        K (np.ndarray): [description]
        vis (bool, optional): [description]. Defaults to False.

    Returns:
        np.ndarray: [description]
    """
    # x = next(pose3d_model(rgb_image))
    # keypoints = x['predictions'][0][0]['keypoints']
    # keypoints = np.array(keypoints)
    # keypoints[:, 0] = -keypoints[:, 0]
    # keypoints2d = pose3d_model._buffer["pose2d_results"].pred_instances.keypoints[0]
    keypoints_media, keypoints2d_coco= media_pipe_debug(rgb_image, pose3d_model)
    keypoints2d = convert_coco_to_mpii(keypoints2d_coco)
    keypoints_media = np.array(keypoints_media)
    keypoints= mediapipe_to_mmpose3d(keypoints_media)

    keypoints = solve_alignment_problem(keypoints, keypoints2d_coco, depth_image, K)
    # keypoints = alignment_dumb(keypoints, keypoints2d, depth_image, K)

    return keypoints, keypoints_media, keypoints2d, keypoints2d_coco






def construct_meida_pipe():
    mp_pose = mp.solutions.pose
    mp_pose_model = mp_pose.Pose(static_image_mode=True, min_detection_confidence=0.5)
    return mp_pose_model


def project_points_to_pixel(points: np.ndarray, K: np.ndarray) -> np.ndarray:
    """
    Projects 3D points onto a 2D plane using the camera intrinsic matrix.

    Args:
        points (np.ndarray): 3D points with shape (N, 3), where each row is a point [x, y, z].
        K (np.ndarray): Camera intrinsic matrix of shape (3, 3).

    Returns:
        np.ndarray: Projected 2D points on the image plane with shape (N, 2).

    """
    N = points.shape[0]
    pixel_points = np.zeros((N, 2))

    for i in range(N):
        # Create homogeneous coordinates [X/Z, Y/Z, 1]
        x, y, z = points[i]
        x, y, z = x, -z, y

        u = K[0, 0] * x / z + K[0, 2]
        v = K[1, 1] * y / z + K[1, 2]
        pixel_points[i] = [u, v]

    return pixel_points


def compute_angle_bisector(A: np.ndarray, B: np.ndarray, C: np.ndarray) -> np.ndarray:
    """AI is creating summary for compute_angle_bisector

    Args:
        A (np.ndarray): [description]
        B (np.ndarray): [description]
        C (np.ndarray): [description]

    Returns:
        np.ndarray: [description]
    """

    # Compute direction vectors
    v = A - B
    w = C - B

    # Normalize the direction vectors
    v_unit = v
    w_unit = w
    # Compute the bisector vector
    bisector = v_unit + w_unit

    # Normalize the bisector vector
    bisector_unit = bisector / np.linalg.norm(bisector)

    return bisector_unit


def compute_handle_bar_poselift(pose3d_model=None, rgb_image: np.ndarray = None, depth_image: np.ndarray = None, v_com=[31 / 42.4, 29 / 42.4], vis=False, frame=None):
    """AI is creating summary for compute_handle_bar_poselift

    Args:
        rgb_image (np.ndarray, optional): [description]. Defaults to None.
        depth_image (np.ndarray, optional): [description]. Defaults to None.
        v_com (list, optional): [description]. Defaults to [31 / 42.4, 29 / 42.4].
        vis (bool, optional): [description]. Defaults to False.

    Returns:
        [type]: [description]
    """
    folder = "./realsense_images_bare"
    # folder = "./debug"
    rgb_path = f"{folder}/color{frame}.png"
    depth_path = f"{folder}/depth{frame}.png"
    # Load and process the RGB image
    if rgb_image is None or depth_image is None:
        rgb_image = cv2.imread(rgb_path)
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        depth_image = depth_image.astype(np.float32)  # Convert to meters
        depth_image = depth_image / 1000.0
        depth_image = depth_inpainting(depth_image)

    keypoints = get_3d_keypoints_3dndepth(
        pose3d_model, rgb_image, depth_image, K, vis)
    
   
    keypoints2d = pose3d_model._buffer["pose2d_results"].pred_instances.keypoints[0]
    keypoints2d_mpii = convert_coco_to_mpii(keypoints2d)
    keypoints2d_scores = pose3d_model._buffer["pose2d_results"].pred_instances.keypoint_scores[0]
    # keypoints2d_projected = project_points_to_pixel(keypoints, K)

    if vis:

        show_rgbd_keypoints(rgb_image, depth_image,
                            keypoints2d_mpii, keypoints2d_projected=None)

    # Connect keypoints with lines, skeleton-style
    connections = [
        (0, 1), (1, 2), (2, 3),  # Example connections, modify as needed
        (0, 4), (4, 5), (5, 6),
        (0, 7), (7, 8), (8, 9),
        (9, 10), (8, 11), (11, 12),
        (12, 13), (8, 14), (14, 15),
        (15, 16)
    ]

    # Weights for each keypoint, keypoins of the trunk should have larger weights
    weights = np.array([1000, 1000, 50, 20, 1000, 50,
                        20, 1000, 1000, 5, 5, 1000, 5, 5, 1000, 5, 5])
    keypoints = np.array(keypoints)

    # Fit a plane to the keypoints. find the coronal plane
    coronal_plane = fit_plane_to_points(keypoints, weights)
    A, B, C, D = coronal_plane

    normal = np.array([A, B, C])
    # vec_head_neck = keypoints[9] - keypoints[8]
    # vec_leg = compute_angle_bisector(keypoints[4], keypoints[5], keypoints[6])
    # p1, p2, p3 = depth_lift(keypoints2d_mpii[12], depth_image, K), depth_lift(
    #     keypoints2d_mpii[13], depth_image, K), depth_lift(keypoints2d_mpii[14], depth_image, K)
    p1,p2,p3=keypoints[8], keypoints[7], keypoints[0]
    vec_leg = compute_angle_bisector(p1, p2, p3)

    normal = normal / np.linalg.norm(normal)  # Normalize the normal vector
    sign = 1
    if np.dot(vec_leg, normal) < 0:
        print("normal flipped")
        normal = -normal
        sign = -1

    # find the sagittal plane
    sagittal_plane = find_perpendicular_plane_through_points(
        coronal_plane, keypoints[0], keypoints[8])

    A1, B1, C1, D1 = sagittal_plane
    normal1 = np.array([A1, B1, C1])
    normal1 = normal1 / np.linalg.norm(normal1)  # Normalize the normal vector

    # print("3d keypoints:", keypoints)
    if vis:
        fig, ax = visualize_points_and_plane(
            keypoints, weights, coronal_plane, sagittal_plane, connections=connections, sign=sign, vec_leg=vec_leg)
        fig.canvas.mpl_connect('key_press_event', on_key)
        ax.scatter(p1[0], p1[1], p1[2], color='red', s=100, marker='D')
        ax.scatter(p2[0], p2[1], p2[2], color='green', s=100, marker='D')
        ax.scatter(p3[0], p3[1], p3[2], color='red', s=100, marker='D')
        plt.show()
        plt.close()

    def check_flip():
        points = np.array([[0, 0, 0], normal])
        projected_points = project_points_to_plane(points, sagittal_plane)

        if projected_points[1][0] > projected_points[0][0]:
            return True
        else:
            return False

    projected_points = project_points_to_plane(keypoints, sagittal_plane)
    sagittal_points = projected_points[[3, 2, 0, 7, 8, 9, 15, 16]]
    link_masses = {0: 0.0797, 1: 0.1354, 2: 0.3446,
                   3: 0.2593, 4: 0.1065, 5: 0.0419, 6: 0.0325}
    test_scene = {}
    test_scene['name'] = "test"
    test_scene['measurements'] = []
    for i in range(1, len(sagittal_points)):
        p = [sagittal_points[i][0]-sagittal_points[i-1][0],
             sagittal_points[i][1]-sagittal_points[i-1][1]]
        test_scene['measurements'].append(np.array(p))
    normalize_arm(test_scene)
    flip = check_flip()
    print("need to flip!", flip)

    for i in range(len(test_scene['measurements'])):
        if flip:
            test_scene['measurements'][i][0] = - \
                test_scene['measurements'][i][0]
    center_of_mass_from_origin = first_5_center_of_mass(
        test_scene, link_masses)
    center_of_mass_from_shoulder_in_04 = [
        center_of_mass_from_origin[0] - sagittal_points[4][0], center_of_mass_from_origin[1] - sagittal_points[4][1]]
    test_scene['measured_com_from_shoulder_in_0,4'] = center_of_mass_from_shoulder_in_04
    test_scene['measured_com_from_origin'] = center_of_mass_from_origin
    # one need to specify the normalized velocity of the center of mass
    # todo: this should be calculated from the video, or something else
    v_com = compute_vcom(test_scene, link_masses)
    test_scene['measured_v_com'] = np.array(v_com)
    elevation_angle_length(test_scene)
    test_scene['theta0,4'] = test_scene['link_3']['elevation_angle']  # 47
    test_scene['theta5'] = test_scene['link_3']['elevation_angle'] + \
        test_scene['link_5']['elevation_angle']  # 100
    test_scene['theta6'] = 180 + (180 - test_scene['link_5']['elevation_angle'] -
                                  test_scene['link_6']['elevation_angle'])  # 239
    test_scene['theta_com'] = 270 - \
        test_scene['com_from_shoulder_elevation_angle']  # 208

    vector_length_calculations(test_scene, link_masses)

    best_theta5, best_theta6, max_overall_score = optimum_placement(
        test_scene, vis=False)
    if vis:
        vis_best_pose(best_theta5, best_theta6, test_scene,
                      test_scene['measured_com_from_origin'], test_scene['measured_v_com'])
    points, points2 = get_endpoints_in_sagittal_plane(
        best_theta5, best_theta6, test_scene)
    endpoint = points2[-1]
    if flip:
        endpoint[0] = -endpoint[0]
    endpoint = np.array(endpoint) + np.array(sagittal_points[0])
    com = compute_com_3d(keypoints, weights)
    dist = 1
    endpoint3d, orientation = recover_3d_from_2d(sagittal_plane, endpoint)
    A1, B1, C1, D1 = sagittal_plane
    normal1 = np.array([A1, B1, C1])
    normal1 = normal1 / np.linalg.norm(normal1)  # Normalize the normal vector

    bar_length = 0.2
    a = endpoint3d+orientation*bar_length
    direction = np.array([normal[0], normal[1], 0])  # Ensure direction is a unit vector
    # Ensure direction is a unit vector
    direction= direction/ np.linalg.norm(direction)
    robot_x_y = com + dist*normal

    b = endpoint3d-orientation*bar_length

    if vis:
        coronal_plane = None
        fig, ax = visualize_points_and_plane(keypoints, weights, coronal_plane,
                                             sagittal_plane=None, connections=connections, handle_barpoint=endpoint3d)
        ax.plot([endpoint3d[0], a[0]], [endpoint3d[1], a[1]],
                [endpoint3d[2], a[2]], color='green')
        ax.plot([endpoint3d[0], b[0]], [endpoint3d[1], b[1]],
                [endpoint3d[2], b[2]], color='green')
        ax.scatter(robot_x_y[0], robot_x_y[1],
                   robot_x_y[2], color='red', s=100, marker='D')
        c = robot_x_y - normal*0.2
        ax.plot([robot_x_y[0], c[0]], [robot_x_y[1], c[1]],
                [robot_x_y[2], c[2]], color='red')
        fig.canvas.mpl_connect('key_press_event', on_key)
    endpoint3d[1], endpoint3d[2] = endpoint3d[2], endpoint3d[1]
    endpoint3d[1] = -endpoint3d[1]
    a[1], a[2] = a[2], a[1]
    a[1] = -a[1]
    b[1], b[2] = b[2], b[1]
    b[1] = -b[1]

    orientation[1], orientation[2] = orientation[2], orientation[1]
    orientation[1] = -orientation[1]

    quaternion = vector_to_quaternion(orientation)
    robot_x_y[1], robot_x_y[2] = robot_x_y[2], robot_x_y[1]
    robot_x_y[1] = -robot_x_y[1]

    normal[1], normal[2] = normal[2], normal[1]
    normal[1] = -normal[1]
    print(f"Quaternion: {quaternion}")
    print("robot position", robot_x_y)

    normal_quaternion = vector_to_quaternion(-normal)

    if vis:
        # coronal_plane = None
        # fig, ax = visualize_points_and_plane(keypoints, weights, coronal_plane,
        #                                      sagittal_plane=None, connections=connections, handle_barpoint=endpoint3d)
        # ax.plot([endpoint3d[0], a[0]], [endpoint3d[1], a[1]],
        #         [endpoint3d[2], a[2]], color='green')
        # ax.plot([endpoint3d[0], b[0]], [endpoint3d[1], b[1]],
        #         [endpoint3d[2], b[2]], color='green')
        # ax.scatter(robot_x_y[0], robot_x_y[1],
        #            robot_x_y[2], color='red', s=100, marker='D')
        # c = robot_x_y - normal*0.2
        # ax.plot([robot_x_y[0], c[0]], [robot_x_y[1], c[1]],
        #         [robot_x_y[2], c[2]], color='red')
        # fig.canvas.mpl_connect('key_press_event', on_key)
        vis_point_clouds(endpoint3d, orientation, rgb_image, depth_image, K)
        plt.show()
        plt.close()

    result = {"handlebar_position": endpoint3d,
              "robot_position": robot_x_y, "handlebar_quaternion": quaternion, "handlebar_vector": orientation, "robot_orientation": normal_quaternion, "robot_vector": -normal,
              "keypoints2d": keypoints2d_mpii, "keypoints2d_scores": keypoints2d_scores}

    # print("result", result)
    return result


def check_arm_raising(pose2d_model,  rgb_image: np.ndarray)->Dict:
    """AI is creating summary for check_gesture

    Args:
        pose_2d_model ([type]): [description]
        bounding_box (np.ndarray): [description]
        rgb_image (np.ndarray): [description]

    Returns:
        Dict: [description]
    """
    results = next(pose2d_model(rgb_image))['predictions']
    keypoints2d_coco = np.array(results[0][0]['keypoints'])  # (N, 2) array
    keypoints2d_coco = convert_coco_to_mpii(keypoints2d_coco)
    keypoints2d_coco = np.array(keypoints2d_coco)
    left_arm = keypoints2d_coco[2][1]>keypoints2d_coco[4][1]
    right_arm = keypoints2d_coco[5][1]>keypoints2d_coco[7][1]
    return {"left_arm":left_arm, "right_arm":right_arm}



def get_mediapipe_keypoints(image_rgb, mp_pose_model=None):
    """
    Detect 2D and 3D keypoints from an RGB image using MediaPipe Pose.

    Returns:
        tuple:
            - list of 3D keypoints [(x, y, z), ...]
            - list of 2D keypoints [(x, y), ...]
            - image shape
            - detection success flag (True/False)
    """
    if mp_pose_model is None:
            
        mp_pose = mp.solutions.pose
        mp_pose_model = mp_pose.Pose(static_image_mode=True, min_detection_confidence=0.5)

        results = mp_pose_model.process(image_rgb)
        mp_pose_model.close()
    else:
        results = mp_pose_model.process(image_rgb)


    if not results.pose_landmarks:
        return None, None, image_rgb.shape, False

    h, w, _ = image_rgb.shape
    keypoints_2d = []
    keypoints_3d = []

    for landmark in results.pose_landmarks.landmark:
        keypoints_2d.append((landmark.x * w, landmark.y * h))
        keypoints_3d.append((landmark.x, landmark.y, landmark.z))

    return keypoints_3d, keypoints_2d, image_rgb.shape, True


def visualize_keypoints(image_rgb, keypoints_3d, keypoints_2d):
    """
    Visualize 3D and 2D keypoints on subplots. for media_pipe
    """
    mp_pose = mp.solutions.pose

    fig = plt.figure(figsize=(12, 5))

    # 3D subplot
    ax1 = fig.add_subplot(121, projection='3d')
    keypoints_array = np.array(keypoints_3d)
    ax1.scatter(keypoints_array[:, 0], keypoints_array[:, 1], keypoints_array[:, 2], c='b', marker='o')

    for connection in mp_pose.POSE_CONNECTIONS:
        start_idx, end_idx = connection
        p1 = keypoints_array[start_idx]
        p2 = keypoints_array[end_idx]
        ax1.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], c='r', linewidth=1)

    for idx, point in enumerate(keypoints_array):
        ax1.text(point[0], point[1], point[2], str(idx), fontsize=8, color='black')

    ax1.set_title('3D Pose Keypoints with Indices')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_xlim([0, 1])
    ax1.set_ylim([0, 1])
    ax1.set_zlim([-1, 1])

    # 2D subplot
    ax2 = fig.add_subplot(122)
    ax2.imshow(image_rgb)
    for idx, (x, y) in enumerate(keypoints_2d):
        ax2.plot(x, y, 'go', markersize=5)
        ax2.text(x, y, str(idx), fontsize=8, color='white', bbox=dict(facecolor='black', alpha=0.5))

    for connection in mp_pose.POSE_CONNECTIONS:
        start_idx, end_idx = connection
        x1, y1 = keypoints_2d[start_idx]
        x2, y2 = keypoints_2d[end_idx]
        ax2.plot([x1, x2], [y1, y2], 'r-', linewidth=1)

    ax2.set_title('2D Pose Keypoints with Indices')
    ax2.axis('off')
    plt.tight_layout()
    plt.show()

def convert_to_coco_keypoints(keypoints_2d, image_shape):
    """
    Convert MediaPipe 2D keypoints to COCO format as an (n, 2) array in pixel coordinates.
    
    Returns:
        np.ndarray: (17, 2) array with COCO keypoints, where each row is (x, y) in pixels.
                    Missing keypoints are set to np.nan.
    """
    # Mapping from COCO to MediaPipe indices
    mp_to_coco_map = {
        0: 0,    # Nose
        1: 2,    # Left Eye
        2: 5,    # Right Eye
        3: 7,    # Left Ear
        4: 8,    # Right Ear
        5: 11,   # Left Shoulder
        6: 12,   # Right Shoulder
        7: 13,   # Left Elbow
        8: 14,   # Right Elbow
        9: 15,   # Left Wrist
        10: 16,  # Right Wrist
        11: 23,  # Left Hip
        12: 24,  # Right Hip
        13: 25,  # Left Knee
        14: 26,  # Right Knee
        15: 27,  # Left Ankle
        16: 28   # Right Ankle
    }

    coco_keypoints = np.full((17, 2), np.nan, dtype=np.float32)

    for coco_idx, mp_idx in mp_to_coco_map.items():
        if mp_idx < len(keypoints_2d):
            coco_keypoints[coco_idx] = keypoints_2d[mp_idx]  # already in pixel

    return coco_keypoints

def media_pipe_debug(image_rgb, mp_pose_model=None):
    """
    Main function to extract keypoints, visualize, and convert to COCO format.

    Returns:
        tuple: (keypoints_3d, coco_keypoints) or (None, None) if detection fails
    """
    keypoints_3d, keypoints_2d, shape, success = get_mediapipe_keypoints(image_rgb, mp_pose_model)

    if not success:
        print("No pose detected.")
        return None, None

    visualize_keypoints(image_rgb, keypoints_3d, keypoints_2d)
    coco_keypoints = convert_to_coco_keypoints(keypoints_2d, shape)

    return keypoints_3d, coco_keypoints




def compute_handle_bar_by_rgbd_media_pipe(pipe_model=None, rgb_image: np.ndarray = None, depth_image: np.ndarray = None, v_com=[31 / 42.4, 29 / 42.4], vis=False, frame=None):
    """AI is creating summary for compute_handle_bar_by_rgbd

    Args:
        rgb_image (np.ndarray, optional): [description]. Defaults to None.
        depth_image (np.ndarray, optional): [description]. Defaults to None.
        v_com (list, optional): [description]. Defaults to [31 / 42.4, 29 / 42.4].
        vis (bool, optional): [description]. Defaults to False.

    Returns:
        [type]: [description]
    """

    # Define the dataset folder
    folder = "./realsense_images_bare"

    # folder = "./debug/"
    if frame is None:
        frame = 10
    # frame = 384
    # frame = 28
    # frame = 256

    rgb_path = f"{folder}/color{frame}.png"
    depth_path = f"{folder}/depth{frame}.png"
    print(f"Processing frame {frame} with RGB path: {rgb_path} and Depth path: {depth_path}")
    # Load and process the RGB image
    if rgb_image is None or depth_image is None:
        rgb_image = cv2.imread(rgb_path)
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)

        depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        depth_image = depth_image.astype(np.float32)  # Convert to meters
        depth_image = depth_image / 1000.0
        # depth_image = cv2.medianBlur(depth_image, 5)  # Kernel size = 5
        depth_image = depth_inpainting(depth_image)

    # results = next(pose2d_model(rgb_image))['predictions']



    # keypoints2d_coco = np.array(results[0][0]['keypoints'])  # (N, 2) array
    keypoints_media, keypoints2d_coco= media_pipe_debug(rgb_image, pipe_model)
    keypoints_media = np.array(keypoints_media)
    
    

    keypoints2d_scores =np.array([1.0]*17)  # Placeholder for keypoint scores, assuming all are valid
    keypoints2d_coco = convert_coco_to_mpii(keypoints2d_coco)

    if vis:
        show_rgbd_keypoints(rgb_image, depth_image, keypoints2d_coco)
    keypoints = get_3d_keypoints(keypoints2d_coco, depth_image, K)
    keypoints = np.array(keypoints)

    valid = check_validity(keypoints)

    # Connect keypoints with lines, skeleton-style
    connections = [
        (0, 1), (1, 2), (2, 3),  # Example connections, modify as needed
        (0, 4), (4, 5), (5, 6),
        (0, 7), (7, 8), (8, 9),
        (9, 10), (8, 11), (11, 12),
        (12, 13), (8, 14), (14, 15),
        (15, 16)
    ]

    # Weights for each keypoint, keypoins of the trunk should have larger weights
    weights = np.array([1000, 1000, 50, 20, 1000, 50,
                        20, 1000, 1000, 5, 5, 1000, 5, 5, 1000, 5, 5])

    if valid:

        # better for the visualization, as well as the optimal bar position calculation
        keypoints[:, 1] = -keypoints[:, 1]
        keypoints[:, [1, 2]] = keypoints[:, [2, 1]]   #swap y and z

        

        keypoints_media[:, 1] = -keypoints_media[:, 1]
        keypoints_media[:, [1, 2]] = keypoints_media[:, [2, 1]]

        # Fit a plane to the keypoints. find the coronal plane

        coronal_plane = fit_plane_to_points(keypoints, weights)

        # find the sagittal plane

        sagittal_plane = find_perpendicular_plane_through_points(
            coronal_plane, keypoints[0], keypoints[8])
        A,B,C,_=coronal_plane
      
        human_orientation=keypoints_media[0] - keypoints_media[8]
        normal=np.array([A, B, C])
        normal=normal / np.linalg.norm(normal)  # Normalize the normal vector
        sign=1
        if np.dot(human_orientation, normal) < 0:
            normal = -normal
            sign=-1

        if vis:
            fig, ax = visualize_points_and_plane(
                keypoints, weights, coronal_plane, sagittal_plane, connections=connections, sign=sign)
            fig.canvas.mpl_connect('key_press_event', on_key)
            plt.show()

            plt.close()

        projected_points = project_points_to_plane(keypoints, sagittal_plane)
    # sagittal points
    else:
        # Initialize projected_points as a NumPy array
        
        projected_points = np.zeros((17, 2))
        normal = np.array([1, 0, 0])  # Default normal if plane fitting fails
        p1,p2,p3=keypoints[4],keypoints[5],keypoints[6]
        human_orientation=keypoints_media[0] - keypoints_media[8]
        sagittal_plane = np.array([0,1,0, 0])  # Default sagittal plane
        normal=normal / np.linalg.norm(normal)  # Normalize the normal vector
        if np.dot(human_orientation, normal) < 0:
            normal = -normal
        # Assign values based on mapping
        for i, j in map_2d_to_3d.items():
            projected_points[j][0], projected_points[j][1] = keypoints2d_coco[i][0], - \
                keypoints2d_coco[i][1]

        # Compute the midpoint for index 7
        projected_points[7] = (projected_points[8] + projected_points[0]) / 2
    sagittal_points = projected_points[[3, 2, 0, 7, 8, 9, 15, 16]]
    link_masses = {0: 0.0797, 1: 0.1354, 2: 0.3446,
                   3: 0.2593, 4: 0.1065, 5: 0.0419, 6: 0.0325}
    test_scene = {}
    test_scene['name'] = "test"
    test_scene['measurements'] = []
    for i in range(1, len(sagittal_points)):
        p = np.array([sagittal_points[i][0]-sagittal_points[i-1][0],
                      sagittal_points[i][1]-sagittal_points[i-1][1]])
        test_scene['measurements'].append(p)
    normalize_arm(test_scene)
    def check_flip():
        print(f"normal: {normal}")
        points = np.array([[0, 0, 0], normal])
        if valid:
            projected_points = project_points_to_plane(points, sagittal_plane)
        else:
            projected_points = project_points_to_plane(points, sagittal_plane, np.array([1, 0,0]))
        print("projected points:", projected_points)

        if projected_points[1][0] > projected_points[0][0]:
            return True
        else:
            return False

    flip = check_flip()
    print("flip:", flip)

    for i in range(len(test_scene['measurements'])):
        if flip:
            test_scene['measurements'][i][0] = - \
                test_scene['measurements'][i][0]
    center_of_mass_from_origin = first_5_center_of_mass(
        test_scene, link_masses)
    center_of_mass_from_shoulder_in_04 = [
        center_of_mass_from_origin[0] - sagittal_points[4][0], center_of_mass_from_origin[1] - sagittal_points[4][1]]
    test_scene['measured_com_from_shoulder_in_0,4'] = center_of_mass_from_shoulder_in_04
    test_scene['measured_com_from_origin'] = center_of_mass_from_origin
    # one need to specify the normalized velocity of the center of mass
    # todo: this should be calculated from the video, or something else
    v_com = compute_vcom(test_scene, link_masses)
    test_scene['measured_v_com'] = np.array(v_com)
    elevation_angle_length(test_scene)
    test_scene['theta0,4'] = test_scene['link_3']['elevation_angle']  # 47
    test_scene['theta5'] = test_scene['link_3']['elevation_angle'] + \
        test_scene['link_5']['elevation_angle']  # 100
    test_scene['theta6'] = 180 + (180 - test_scene['link_5']['elevation_angle'] -
                                  test_scene['link_6']['elevation_angle'])  # 239
    test_scene['theta_com'] = 270 - \
        test_scene['com_from_shoulder_elevation_angle']  # 208

    vector_length_calculations(test_scene, link_masses)

    best_theta5, best_theta6, max_overall_score = optimum_placement(
        test_scene, vis=False)
    if vis:
        vis_best_pose(best_theta5, best_theta6, test_scene,
                      test_scene['measured_com_from_origin'], test_scene['measured_v_com'])
    points, points2 = get_endpoints_in_sagittal_plane(
        best_theta5, best_theta6, test_scene)
    endpoint = points2[-1]
    if flip:
        endpoint[0] = -endpoint[0]
    endpoint = np.array(endpoint) + np.array(sagittal_points[0])
    com = compute_com_3d(keypoints, weights)
    dist = 1

    if valid:
        endpoint3d, orientation = recover_3d_from_2d(sagittal_plane, endpoint)
        bar_length = 0.2
        a = endpoint3d+orientation*bar_length
        A, B, C, D = coronal_plane
        normal = np.array([A, B, C])
        human_orientation=keypoints_media[0] - keypoints_media[8]
        print("debug",human_orientation, normal)
        normal = normal / np.linalg.norm(normal)  # Normalize the normal vector
        if np.dot(human_orientation, normal) < 0:
            normal = -normal
        direction = np.array([normal[0], normal[1], 0])  # Ensure direction is a unit vector
        # Ensure direction is a unit vector
        direction= direction/ np.linalg.norm(direction)
        robot_x_y = com + dist*direction

        b = endpoint3d-orientation*bar_length
        if vis:
            coronal_plane = None
            fig, ax = visualize_points_and_plane(keypoints, weights, coronal_plane,
                                                 sagittal_plane=None, connections=connections, handle_barpoint=endpoint3d)
            ax.plot([endpoint3d[0], a[0]], [endpoint3d[1], a[1]],
                    [endpoint3d[2], a[2]], color='green')
            ax.plot([endpoint3d[0], b[0]], [endpoint3d[1], b[1]],
                    [endpoint3d[2], b[2]], color='green')
            ax.scatter(robot_x_y[0], robot_x_y[1],
                       robot_x_y[2], color='red', s=100, marker='D')
            c = robot_x_y - normal*0.2
            ax.plot([robot_x_y[0], c[0]], [robot_x_y[1], c[1]],
                    [robot_x_y[2], c[2]], color='red')
            fig.canvas.mpl_connect('key_press_event', on_key)
            plt.show()
            plt.close()

        endpoint3d[1], endpoint3d[2] = endpoint3d[2], endpoint3d[1]
        endpoint3d[1] = -endpoint3d[1]
        a[1], a[2] = a[2], a[1]
        a[1] = -a[1]
        b[1], b[2] = b[2], b[1]
        b[1] = -b[1]

        orientation[1], orientation[2] = orientation[2], orientation[1]
        orientation[1] = -orientation[1]

        quaternion = vector_to_quaternion(orientation)
        robot_x_y[1], robot_x_y[2] = robot_x_y[2], robot_x_y[1]
        robot_x_y[1] = -robot_x_y[1]

        normal[1], normal[2] = normal[2], normal[1]
        normal[1] = -normal[1]
        print("com:", com)
        print(f"Quaternion: {quaternion}")
        print("robot position", robot_x_y)
    else:
        z = get_average_depth(keypoints2d_coco, depth_image)
        x = z * (endpoint[0] - K[0][2]) / K[0][0]
        y = z * (-endpoint[1] - K[1][2]) / K[1][1]
        bbox_min, bbox_max = get_human_bounding_box(keypoints)
        endpoint3d = np.array([x, y, z])

        endpoint3d[1]=np.clip(endpoint3d[1], bbox_min[1]+0.3, bbox_max[1]-0.3)
        orientation = np.array([0, 0, 1])
        bar_length = 0.2
        a = endpoint3d+orientation*bar_length
        b = endpoint3d-orientation*bar_length
        quaternion = vector_to_quaternion(orientation)
        if flip:
            normal = np.array([1, 0, 0])
        else:
            normal = np.array([-1, 0, 0])
        robot_x_y = com + dist*normal
        if vis:
            coronal_plane = None
            fig, ax = visualize_points_and_plane(keypoints, weights, coronal_plane,
                                                 sagittal_plane=None, connections=connections, handle_barpoint=endpoint3d)
            ax.plot([endpoint3d[0], a[0]], [endpoint3d[1], a[1]],
                    [endpoint3d[2], a[2]], color='green')
            ax.plot([endpoint3d[0], b[0]], [endpoint3d[1], b[1]],
                    [endpoint3d[2], b[2]], color='green')
            ax.scatter(robot_x_y[0], robot_x_y[1],
                       robot_x_y[2], color='red', s=100, marker='D')

            c = robot_x_y - normal*0.2
            ax.plot([robot_x_y[0], c[0]], [robot_x_y[1], c[1]],
                    [robot_x_y[2], c[2]], color='red')
            fig.canvas.mpl_connect('key_press_event', on_key)
            plt.show()
            plt.close()
        # normal[0], normal[1] = normal[1], normal[0]
    print("the normal",normal)
    normal_quaternion = vector_to_quaternion(-normal)
    print("normal quaternion",normal_quaternion)
    result = {"handlebar_position": endpoint3d,
              "robot_position": robot_x_y, "handlebar_quaternion": quaternion, "handlebar_vector": orientation, "robot_orientation": normal_quaternion, "robot_vector": -normal,
              "keypoints2d": keypoints2d_coco, "keypoints2d_scores": keypoints2d_scores}
    # print(result)
    if vis:
        vis_point_clouds(endpoint3d, orientation, rgb_image, depth_image, K)
    return result








def compute_handle_bar_poselift(pose3d_model=None, rgb_image: np.ndarray = None, depth_image: np.ndarray = None, v_com=[31 / 42.4, 29 / 42.4], vis=False, frame=None):
    """AI is creating summary for compute_handle_bar_poselift

    Args:
        rgb_image (np.ndarray, optional): [description]. Defaults to None.
        depth_image (np.ndarray, optional): [description]. Defaults to None.
        v_com (list, optional): [description]. Defaults to [31 / 42.4, 29 / 42.4].
        vis (bool, optional): [description]. Defaults to False.

    Returns:
        [type]: [description]
    """
    folder = "./realsense_images_bare"
    # folder = "./debug"
    rgb_path = f"{folder}/color{frame}.png"
    depth_path = f"{folder}/depth{frame}.png"
    # Load and process the RGB image
    if rgb_image is None or depth_image is None:
        rgb_image = cv2.imread(rgb_path)
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        depth_image = depth_image.astype(np.float32)  # Convert to meters
        depth_image = depth_image / 1000.0
        depth_image = depth_inpainting(depth_image)

    keypoints , keypoints_media, keypoints2d_mpii, keypoints2d_coco = get_3d_keypoints_3dndepth(
        pose3d_model, rgb_image, depth_image, K, vis)
   
    # keypoints2d_mpii = convert_coco_to_mpii(keypoints2d)
    keypoints2d_scores =  np.array([1.0]*17)  # Placeholder for keypoint scores, assuming all are valid
    # keypoints2d_projected = project_points_to_pixel(keypoints, K)

    if vis:

        show_rgbd_keypoints(rgb_image, depth_image,
                            keypoints2d_mpii, keypoints2d_projected=None)

    # Connect keypoints with lines, skeleton-style
    connections = [
        (0, 1), (1, 2), (2, 3),  # Example connections, modify as needed
        (0, 4), (4, 5), (5, 6),
        (0, 7), (7, 8), (8, 9),
        (9, 10), (8, 11), (11, 12),
        (12, 13), (8, 14), (14, 15),
        (15, 16)
    ]

    # Weights for each keypoint, keypoins of the trunk should have larger weights
    weights = np.array([1000, 1000, 50, 20, 1000, 50,
                        20, 1000, 1000, 5, 5, 1000, 5, 5, 1000, 5, 5])
    keypoints = np.array(keypoints)

    # Fit a plane to the keypoints. find the coronal plane
    coronal_plane = fit_plane_to_points(keypoints, weights)
    A, B, C, D = coronal_plane

    normal = np.array([A, B, C])
    # vec_head_neck = keypoints[9] - keypoints[8]
    # vec_leg = compute_angle_bisector(keypoints[4], keypoints[5], keypoints[6])
    # p1, p2, p3 = depth_lift(keypoints2d_mpii[12], depth_image, K), depth_lift(
    #     keypoints2d_mpii[13], depth_image, K), depth_lift(keypoints2d_mpii[14], depth_image, K)
    
    human_orientation=keypoints_media[0] - keypoints_media[8]

    normal = normal / np.linalg.norm(normal)  # Normalize the normal vector
    sign = 1
    if np.dot(human_orientation, normal) < 0:
        print("normal flipped")
        normal = -normal
        sign = -1

    # find the sagittal plane
    sagittal_plane = find_perpendicular_plane_through_points(
        coronal_plane, keypoints[0], keypoints[8])

    A1, B1, C1, D1 = sagittal_plane
    normal1 = np.array([A1, B1, C1])
    normal1 = normal1 / np.linalg.norm(normal1)  # Normalize the normal vector

    # print("3d keypoints:", keypoints)
    if vis:
        fig, ax = visualize_points_and_plane(
            keypoints, weights, coronal_plane, sagittal_plane, connections=connections, sign=sign)
        fig.canvas.mpl_connect('key_press_event', on_key)
 
        plt.show()
        plt.close()

    def check_flip():
        points = np.array([[0, 0, 0], normal])
        projected_points = project_points_to_plane(points, sagittal_plane)

        if projected_points[1][0] > projected_points[0][0]:
            return True
        else:
            return False

    projected_points = project_points_to_plane(keypoints, sagittal_plane)
    sagittal_points = projected_points[[3, 2, 0, 7, 8, 9, 15, 16]]
    link_masses = {0: 0.0797, 1: 0.1354, 2: 0.3446,
                   3: 0.2593, 4: 0.1065, 5: 0.0419, 6: 0.0325}
    test_scene = {}
    test_scene['name'] = "test"
    test_scene['measurements'] = []
    for i in range(1, len(sagittal_points)):
        p = [sagittal_points[i][0]-sagittal_points[i-1][0],
             sagittal_points[i][1]-sagittal_points[i-1][1]]
        test_scene['measurements'].append(np.array(p))
    normalize_arm(test_scene)
    flip = check_flip()
    print("need to flip!", flip)

    for i in range(len(test_scene['measurements'])):
        if flip:
            test_scene['measurements'][i][0] = - \
                test_scene['measurements'][i][0]
    center_of_mass_from_origin = first_5_center_of_mass(
        test_scene, link_masses)
    center_of_mass_from_shoulder_in_04 = [
        center_of_mass_from_origin[0] - sagittal_points[4][0], center_of_mass_from_origin[1] - sagittal_points[4][1]]
    test_scene['measured_com_from_shoulder_in_0,4'] = center_of_mass_from_shoulder_in_04
    test_scene['measured_com_from_origin'] = center_of_mass_from_origin
    # one need to specify the normalized velocity of the center of mass
    # todo: this should be calculated from the video, or something else
    v_com = compute_vcom(test_scene, link_masses)
    test_scene['measured_v_com'] = np.array(v_com)
    elevation_angle_length(test_scene)
    test_scene['theta0,4'] = test_scene['link_3']['elevation_angle']  # 47
    test_scene['theta5'] = test_scene['link_3']['elevation_angle'] + \
        test_scene['link_5']['elevation_angle']  # 100
    test_scene['theta6'] = 180 + (180 - test_scene['link_5']['elevation_angle'] -
                                  test_scene['link_6']['elevation_angle'])  # 239
    test_scene['theta_com'] = 270 - \
        test_scene['com_from_shoulder_elevation_angle']  # 208

    vector_length_calculations(test_scene, link_masses)

    best_theta5, best_theta6, max_overall_score = optimum_placement(
        test_scene, vis=False)
    if vis:
        vis_best_pose(best_theta5, best_theta6, test_scene,
                      test_scene['measured_com_from_origin'], test_scene['measured_v_com'])
    points, points2 = get_endpoints_in_sagittal_plane(
        best_theta5, best_theta6, test_scene)
    endpoint = points2[-1]
    if flip:
        endpoint[0] = -endpoint[0]
    endpoint = np.array(endpoint) + np.array(sagittal_points[0])
    com = compute_com_3d(keypoints, weights)
    dist = 1
    endpoint3d, orientation = recover_3d_from_2d(sagittal_plane, endpoint)
    A1, B1, C1, D1 = sagittal_plane
    normal1 = np.array([A1, B1, C1])
    normal1 = normal1 / np.linalg.norm(normal1)  # Normalize the normal vector

    bar_length = 0.2
    a = endpoint3d+orientation*bar_length
    direction = np.array([normal[0], normal[1], 0])  # Ensure direction is a unit vector
    # Ensure direction is a unit vector
    direction= direction/ np.linalg.norm(direction)
    robot_x_y = com + dist*normal

    b = endpoint3d-orientation*bar_length

    if vis:
        coronal_plane = None
        fig, ax = visualize_points_and_plane(keypoints, weights, coronal_plane,
                                             sagittal_plane=None, connections=connections, handle_barpoint=endpoint3d)
        ax.plot([endpoint3d[0], a[0]], [endpoint3d[1], a[1]],
                [endpoint3d[2], a[2]], color='green')
        ax.plot([endpoint3d[0], b[0]], [endpoint3d[1], b[1]],
                [endpoint3d[2], b[2]], color='green')
        ax.scatter(robot_x_y[0], robot_x_y[1],
                   robot_x_y[2], color='red', s=100, marker='D')
        c = robot_x_y - normal*0.2
        ax.plot([robot_x_y[0], c[0]], [robot_x_y[1], c[1]],
                [robot_x_y[2], c[2]], color='red')
        fig.canvas.mpl_connect('key_press_event', on_key)
    endpoint3d[1], endpoint3d[2] = endpoint3d[2], endpoint3d[1]
    endpoint3d[1] = -endpoint3d[1]
    a[1], a[2] = a[2], a[1]
    a[1] = -a[1]
    b[1], b[2] = b[2], b[1]
    b[1] = -b[1]

    orientation[1], orientation[2] = orientation[2], orientation[1]
    orientation[1] = -orientation[1]

    quaternion = vector_to_quaternion(orientation)
    robot_x_y[1], robot_x_y[2] = robot_x_y[2], robot_x_y[1]
    robot_x_y[1] = -robot_x_y[1]

    normal[1], normal[2] = normal[2], normal[1]
    normal[1] = -normal[1]
    print(f"Quaternion: {quaternion}")
    print("robot position", robot_x_y)

    normal_quaternion = vector_to_quaternion(-normal)

    if vis:
        # coronal_plane = None
        # fig, ax = visualize_points_and_plane(keypoints, weights, coronal_plane,
        #                                      sagittal_plane=None, connections=connections, handle_barpoint=endpoint3d)
        # ax.plot([endpoint3d[0], a[0]], [endpoint3d[1], a[1]],
        #         [endpoint3d[2], a[2]], color='green')
        # ax.plot([endpoint3d[0], b[0]], [endpoint3d[1], b[1]],
        #         [endpoint3d[2], b[2]], color='green')
        # ax.scatter(robot_x_y[0], robot_x_y[1],
        #            robot_x_y[2], color='red', s=100, marker='D')
        # c = robot_x_y - normal*0.2
        # ax.plot([robot_x_y[0], c[0]], [robot_x_y[1], c[1]],
        #         [robot_x_y[2], c[2]], color='red')
        # fig.canvas.mpl_connect('key_press_event', on_key)
        vis_point_clouds(endpoint3d, orientation, rgb_image, depth_image, K)
        plt.show()
        plt.close()

    result = {"handlebar_position": endpoint3d,
              "robot_position": robot_x_y, "handlebar_quaternion": quaternion, "handlebar_vector": orientation, "robot_orientation": normal_quaternion, "robot_vector": -normal,
              "keypoints2d": keypoints2d_mpii, "keypoints2d_scores": keypoints2d_scores}

    # print("result", result)
    return result