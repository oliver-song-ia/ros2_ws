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

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.animation import FuncAnimation
import numpy as np
import math
import open3d as o3d

def get_endpoints_in_sagittal_plane(best_theta5, best_theta6, scene):
    points = [(0, 0)]  # Start with the origin
    joints = scene['measurements']
    # Compute absolute positions of joints
    for dx, dy in joints[0:5]:
        last_x, last_y = points[-1]
        points.append((last_x + dx, last_y + dy))
    points = np.array(points)  # Convert to NumPy array for easier plotting
    last_x, last_y = points[4]
    points2 = [(last_x, last_y)]

    elevation_angle5 = best_theta5 + \
        math.degrees(math.atan2(joints[3][1], joints[3][0]))

    dx5, dy5 = scene['r5_len']*math.cos(math.radians(
        elevation_angle5)), scene['r5_len']*math.sin(math.radians(elevation_angle5))

    points2.append((last_x+dx5, last_y+dy5))

    elevation_angle6 = elevation_angle5 + best_theta6

    dx6, dy6 = scene['r6_len']*math.cos(math.radians(
        elevation_angle6)), scene['r6_len']*math.sin(math.radians(elevation_angle6))

    points2.append((last_x+dx5+dx6, last_y+dy5+dy6))
    return np.array(points), np.array(points2)


def show_rgbd_keypoints(rgb_image:np.ndarray, depth_image:np.ndarray, keypoints2d_coco:np.ndarray, keypoints2d_projected:np.ndarray=None):
    """AI is creating summary for show_rgbd_keypoints

    Args:
        rgb_image (np.ndarray): [description]
        depth_image (np.ndarray): [description]
        keypoints (np.ndarray): [description]
    """
    fig = plt.figure()
    plt.subplot(1, 2, 1)
    plt.imshow(rgb_image)
    for i, (x, y) in enumerate(keypoints2d_coco):
        plt.scatter(x, y, c='red')
        plt.text(x, y, f'{i}', color='white', fontsize=9)
        if keypoints2d_projected is not None:
            plt.scatter(keypoints2d_projected[i][0], keypoints2d_projected[i][1], c='green')
            # plt.text(keypoints2d_projected[i][0], keypoints2d_projected[i][1], f'{i}', color='green', fontsize=9)
            
    plt.axis('off')

    plt.subplot(1, 2, 2)
    plt.imshow(depth_image, cmap="jet")
    # plt.colorbar()

    for i, (x, y) in enumerate(keypoints2d_coco):
        plt.scatter(x, y, c='red')
        plt.text(x, y, f'{i}', color='white', fontsize=9)
    plt.axis('off')
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.show()

    plt.close()


def on_key(event):
    if event.key == 'escape':  # Check if ESC is pressed
        plt.close(event.canvas.figure)  # Close only the active figure


def visualize_2d_pose(img: np.ndarray, keypoints: np.ndarray, format='coco') -> None:
    """AI is creating summary for visualize_2d_pose

    Args:
        img (np.ndarray): [description]
        keypoints (np.ndarray): [description]
        format (str, optional): [description]. Defaults to 'coco'.
    """
    if format == 'coco':
        connections = [
            (0, 1),  # nose to left eye
            (0, 2),  # nose to right eye
            (1, 3),  # left eye to left ear
            (2, 4),  # right eye to right ear
            (5, 6),  # left shoulder to right shoulder
            (5, 7),  # left shoulder to left elbow
            (7, 9),  # left elbow to left wrist
            (6, 8),  # right shoulder to right elbow
            (8, 10),  # right elbow to right wrist
            (5, 11),  # left shoulder to left hip
            (6, 12),  # right shoulder to right hip
            (11, 13),  # left hip to left knee
            (13, 15),  # left knee to left ankle
            (12, 14),  # right hip to right knee
            (14, 16)  # right knee to right ankle
        ]
    else:
        connections = [
            (0, 1),  # head to neck
            (1, 2),  # neck to right shoulder
            (2, 3),  # right shoulder to right elbow
            (3, 4),  # right elbow to right wrist
            (1, 5),  # neck to left shoulder
            (5, 6),  # left shoulder to left elbow
            (6, 7),  # left elbow to left wrist
            (1, 8),  # neck to hip center
            (8, 9),  # hip center to right hip
            (9, 10),  # right hip to right knee
            (10, 11),  # right knee to right ankle
            (8, 12),  # hip center to left hip
            (12, 13),  # left hip to left knee
            (13, 14),  # left knee to left ankle
        ]

    # Get image dimensions
    img_height, img_width = img.shape[:2]

    # Create figure
    plt.figure(figsize=(10, 12))

    # Display the image
    plt.imshow(img)

    # Plot keypoints
    for i, (x, y) in enumerate(keypoints):
        # Ensure keypoints are within image boundaries
        if 0 <= x < img_width and 0 <= y < img_height:
            circle = Circle((x, y), radius=5, color='red',
                            fill=True, alpha=0.7)
            plt.gca().add_patch(circle)
            plt.text(x + 5, y, f"{i}", fontsize=8, color='white',
                     bbox=dict(facecolor='black', alpha=0.5))

    # Draw connections between keypoints
    for connection in connections:
        start_idx, end_idx = connection
        plt.plot(
            [keypoints[start_idx, 0], keypoints[end_idx, 0]],
            [keypoints[start_idx, 1], keypoints[end_idx, 1]],
            'lime', linewidth=2, alpha=0.7
        )

    plt.title('Human Pose Estimation')
    plt.axis('off')
    plt.tight_layout()
    plt.show()
    
    

def vis_human(joints: np.ndarray, scene="none"):
    points = [(0, 0)]  # Start with the origin

    # Compute absolute positions of joints
    for dx, dy in joints[0:5]:
        last_x, last_y = points[-1]
        points.append((last_x + dx, last_y + dy))

    points = np.array(points)  # Convert to NumPy array for easier plotting

    # Plot points
    plt.scatter(points[:, 0], points[:, 1], color='red')

    # Plot connections
    plt.plot(points[:, 0], points[:, 1],
             color='blue', linestyle='-', marker='o')
    last_x, last_y = points[4]
    points2 = [(last_x, last_y)]

    for dx, dy in joints[5:]:
        points2.append((last_x+dx, last_y+dy))
        last_x, last_y = points2[-1]
    points2 = np.array(points2)

    plt.scatter(points2[:, 0], points2[:, 1], color='red')
    # Plot connections
    plt.plot(points2[:, 0], points2[:, 1],
             color='green', linestyle='-', marker='o')

    # Labels and styling
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title(f"Human Joints Visualization {scene}")
    plt.legend()
    plt.axis("equal")  # Maintain aspect ratio
    plt.grid(True)
    plt.show()
    

def visualize_points_and_plane(points, weights, plane_params=None,  sagittal_plane=None, connections=None, handle_barpoint=None, sign=1, vec_leg=None):
    """
    Visualize the 3D points and the fitted plane with equal axis scales.

    Parameters:
    -----------
    points : np.ndarray
        Array of shape (n, 3) containing the 3D points.
    weights : np.ndarray
        Array of shape (n,) containing the weights for each point.
    plane_params : tuple
        (a, b, c, d) coefficients of the plane equation ax + by + cz + d = 0
    """
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Normalize weights for visualization
    norm_weights = weights / weights.max()

    # Plot points with size and color according to weights
    scatter = ax.scatter(
        points[:, 0], points[:, 1], points[:, 2],
        c=weights,
        s=norm_weights * 100,  # Scale sizes
        cmap='viridis',
        alpha=0.7
    )

    # Annotate each point with a label
    for i, (x, y, z) in enumerate(points):
        ax.text(x, y, z, f"{i}", color='black', fontsize=10)

    # Add a colorbar for the weights
    cbar = plt.colorbar(scatter, ax=ax, pad=0.1)
    cbar.set_label('Point Weight')

    # Get the bounding box of the points
    x_min, x_max = points[:, 0].min(), points[:, 0].max()
    y_min, y_max = points[:, 1].min(), points[:, 1].max()
    z_min, z_max = points[:, 2].min(), points[:, 2].max()

    # Find the global min and max across all dimensions
    global_min = min(x_min, y_min, z_min)
    global_max = max(x_max, y_max, z_max)

    # Add some padding
    padding = (global_max - global_min) * 0.1
    global_min -= padding
    global_max += padding

    # Create equal ranges for all axes
    x_range = y_range = z_range = (global_min, global_max)
    # x_range = (x_min, x_max)
    # y_range = (y_min, y_max)
    # z_range = (z_min, z_max)

    # Choose the plane grid based on which component of the normal vector is largest
    # Create a grid for the plane
    if plane_params is not None:
        a, b, c, d = plane_params
        max_component = np.argmax(np.abs([a, b, c]))

        # Create grid points that span the equal ranges
        grid_points = np.linspace(global_min, global_max, 20)

        if max_component == 0:  # x-component is largest, so the plane is more "vertical"
            y_grid, z_grid = np.meshgrid(grid_points, grid_points)
            x_grid = (-d - b * y_grid - c * z_grid) / a
        elif max_component == 1:  # y-component is largest
            x_grid, z_grid = np.meshgrid(grid_points, grid_points)
            y_grid = (-d - a * x_grid - c * z_grid) / b
        else:  # z-component is largest, so the plane is more "horizontal"
            x_grid, y_grid = np.meshgrid(grid_points, grid_points)
            z_grid = (-d - a * x_grid - b * y_grid) / c

        # Plot the plane
        surf = ax.plot_surface(x_grid, y_grid, z_grid, alpha=0.3, color='cyan')

    if sagittal_plane is not None:
        a2, b2, c2, d2 = sagittal_plane
        x_grid2, y_grid2 = np.meshgrid(grid_points, grid_points)
        z_grid2 = (-d2 - a2 * x_grid2 - b2 * y_grid2) / c2
        surf2 = ax.plot_surface(x_grid2, y_grid2, z_grid2,
                                alpha=0.3, color='magenta')

    # Add a normal vector from the centroid
    if plane_params is not None:
        centroid = np.mean(points, axis=0)
        normal_length = (global_max - global_min) / 5
        normal_vector = np.array([a, b, c]) * normal_length
        if sign == -1:
            normal_vector = -normal_vector

        ax.quiver(centroid[0], centroid[1], centroid[2],
                normal_vector[0], normal_vector[1], normal_vector[2],
                color='red', arrow_length_ratio=0.1)
        
        if vec_leg is not None:
            ax.quiver(points[2][0], points[2][1], points[2][2],
                vec_leg[0], vec_leg[1], vec_leg[2],
                color='blue', arrow_length_ratio=0.1)

    # plot connections
    for start, end in connections:
        ax.plot3D(
            [points[start, 0], points[end, 0]],
            [points[start, 1], points[end, 1]],
            [points[start, 2], points[end, 2]],
            color='red',
            linewidth=2,
            alpha=0.5
        )

    if handle_barpoint is not None:
        ax.scatter(handle_barpoint[0], handle_barpoint[1],
                   handle_barpoint[2], color='green', s=100, marker='D')
        ax.text(handle_barpoint[0], handle_barpoint[1],
                handle_barpoint[2], 'Handle Bar', color='black', fontsize=10)

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Points and Fitted Plane')

    # Set equal axis limits
    ax.set_xlim(x_range)
    ax.set_ylim(y_range)
    ax.set_zlim(z_range)

    # Force equal aspect ratio
    # ax.set_box_aspect([1, 1, 1])

    # Add text showing the plane equation
    if plane_params is not None:
        equation = f'{a:.4f}x + {b:.4f}y + {c:.4f}z + {d:.4f} = 0'
        fig.text(0.1, 0.01, f'Plane Equation: {equation}', fontsize=12)

    plt.tight_layout()
    
    return fig, ax


def vis_best_pose(best_theta5, best_theta6, scene, com=None, v_com=None, frame=0):
    """AI is creating summary for vis_best_pose

    Args:
        best_theta5 ([type]): [description]
        best_theta6 ([type]): [description]
        scene ([type]): [description]
        com ([type], optional): [description]. Defaults to None.
        v_com ([type], optional): [description]. Defaults to None.
    """
    points, points2 = get_endpoints_in_sagittal_plane(
        best_theta5, best_theta6, scene)
    fig = plt.figure()
    # Plot points
    plt.scatter(points[:, 0], points[:, 1], color='red')

    # Plot connections
    plt.plot(points[:, 0], points[:, 1],
             color='blue', linestyle='-', marker='o')

    factor = 1
    if com is not None:
        print( -factor*v_com[1], factor*v_com[0], "orientation")
        plt.arrow(com[0], com[1], -factor*v_com[1], factor*v_com[0],
                  head_width=0.1*factor, head_length=0.1*factor, fc='orange', ec='orange')

    plt.scatter(points2[:, 0], points2[:, 1], color='red')
    # Plot connections
    plt.plot(points2[:, 0], points2[:, 1],
             color='green', linestyle='-', marker='o')

    # Labels and styling
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title(f"Human Joints Visualization {scene['name']}")
    plt.legend()
    plt.axis("equal")  # Maintain aspect ratio
    # plt.savefig(f"./debug/2d/{frame}.png", dpi=300, bbox_inches='tight')
    plt.grid(True)
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.show()


def vis_2d_pose(keypoints2d: np.ndarray, weights: np.ndarray, connections: np.ndarray = None):
    """AI is creating summary for vis_2d_pose

    Args:
        keypoints2d (np.ndarray): [description]
        weights (np.ndarray): [description]
        connections (List, optional): [description]. Defaults to None.
    """
    fig = plt.figure()
    plt.scatter(keypoints2d[:, 0], keypoints2d[:, 1], c='red')

    for i, (x, y) in enumerate(keypoints2d):
        plt.text(x, y, f'{i}', color='black', fontsize=9)

    if connections is not None:
        for start, end in connections:
            plt.plot(
                [keypoints2d[start, 0], keypoints2d[end, 0]],
                [keypoints2d[start, 1], keypoints2d[end, 1]],
                color='blue'
            )
    plt.title("Projected 2d pose on sagittal plane")
    # plt.savefig('2d_pose.png')
    # plt.show()
    fig.canvas.mpl_connect('key_press_event', on_key)
    # plt.close()
def vis_point_clouds(position: np.ndarray, orientation: np.ndarray, rgb: np.ndarray, depth: np.ndarray, K: np.ndarray, output_path: str = "point_cloud.ply"):
    """
    Create and save a point cloud from RGB and depth images with a position and orientation arrow to a .ply file.
    
    Args:
        position (np.ndarray): 3D position of the arrow start point, shape (3,)
        orientation (np.ndarray): 3D orientation vector of the arrow, shape (3,)
        rgb (np.ndarray): RGB image as numpy array of shape (H, W, 3) with values 0-255
        depth (np.ndarray): Depth image as numpy array of shape (H, W) in meters
        K (np.ndarray): Camera intrinsic matrix of shape (3, 3)
        output_path (str): Path to save the .ply file (default: "point_cloud.ply")
    
    Returns:
        None: Saves the point cloud and arrow to a .ply file
    """
    # Convert numpy arrays to Open3D images
    rgb_o3d = o3d.geometry.Image(rgb.astype(np.uint8))
    depth_o3d = o3d.geometry.Image(depth.astype(np.float32))
    
    # Create RGBD image from RGB and depth
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        rgb_o3d,
        depth_o3d,
        depth_scale=1.0,  # Assuming depth is already in meters
        depth_trunc=10.0,  # Maximum depth value in meters
        convert_rgb_to_intensity=False
    )
    
    # Create camera intrinsic object
    height, width = depth.shape
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(
        width=width,
        height=height,
        fx=K[0, 0],  # focal length x
        fy=K[1, 1],  # focal length y
        cx=K[0, 2],  # principal point x
        cy=K[1, 2]   # principal point y
    )
    
    # Create point cloud from RGBD image
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd,
        intrinsic
    )
    
    # Flip the point cloud (Open3D coordinate system fix)
    # pcd.transform([[1, 0, 0, 0],
    #                [0, -1, 0, 0],
    #                [0, 0, -1, 0],
    #                [0, 0, 0, 1]])
    
    # Create an arrow as a small PointCloud
    orientation = orientation / np.linalg.norm(orientation)  # Unit vector
    arrow_length = 0.5  # Adjust this for arrow length
    arrow_end = position + arrow_length * orientation
    
    # Sample points along the arrow for a thicker representation
    num_points = 50  # More points for visibility
    t = np.linspace(0, 1, num_points)
    arrow_points = np.outer((1 - t), position) + np.outer(t, arrow_end)
    arrow_colors = np.tile([1, 0, 0], (num_points, 1))  # Red color for the arrow
    
    # Create PointCloud for the arrow
    arrow_pcd = o3d.geometry.PointCloud()
    arrow_pcd.points = o3d.utility.Vector3dVector(arrow_points)
    arrow_pcd.colors = o3d.utility.Vector3dVector(arrow_colors)

    arrow_end2 = position- arrow_length * orientation
    arrow_points2 = np.outer((1 - t), position) + np.outer(t, arrow_end2)
    arrow_colors2 = np.tile([1, 0, 0], (num_points, 1))  # Green color for the arrow

    arrow_pcd2 = o3d.geometry.PointCloud()
    arrow_pcd2.points = o3d.utility.Vector3dVector(arrow_points2)
    arrow_pcd2.colors = o3d.utility.Vector3dVector(arrow_colors2)

    # Combine the original point cloud and the arrow point cloud
    combined_pcd = pcd + arrow_pcd +arrow_pcd2  # Merges points and colors
    combined_pcd.transform([[1, 0, 0, 0],
                             [0, -1, 0, 0],
                             [0, 0, -1, 0],
                             [0, 0, 0, 1]])
    # Save the combined point cloud to a .ply file
    o3d.io.write_point_cloud(output_path, combined_pcd)