#!/usr/bin/env python

import cv2
import numpy as np
import colorsys
from scipy.interpolate import griddata
from scipy.ndimage import uniform_filter
import open3d as o3d
import copy


def set_zero_area_to_255(image, window_size):
    # rows, cols = image.shape
    # kernel = np.ones((window_size, window_size))
    filtered_image = uniform_filter(image, size=window_size, mode='constant')

    zero_mask = (filtered_image == 0)
    image[zero_mask] = 255
    
    return image


def nearest_neighbor_inpaint(image, mask):
    inpainted_image = image.copy()

    # Create grids for the original image and the inpainted image
    y, x = np.mgrid[:image.shape[0], :image.shape[1]]
    y = y.astype(np.float32)
    x = x.astype(np.float32)

    # Find non-black pixel coordinates and their corresponding colors
    non_black_coords = np.where(mask == 0)
    non_black_colors = image[non_black_coords]

    # Find black pixel coordinates
    black_coords = np.where(mask > 0)

    # Inpaint using Nearest Neighbor interpolation
    inpainted_colors = griddata(np.column_stack((non_black_coords[1], non_black_coords[0])), non_black_colors, np.column_stack((black_coords[1], black_coords[0])), method='nearest')
    inpainted_image[black_coords] = inpainted_colors

    return inpainted_image

def remove_black_snowflakes(image, threshold=1):
    # Convert the image to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply binary thresholding
    _, binary_image = cv2.threshold(gray_image, threshold, 255, cv2.THRESH_BINARY_INV)

    # Apply Nearest Neighbor inpainting
    inpainted_image = nearest_neighbor_inpaint(image, binary_image)

    return inpainted_image


def filter_points_by_rectangular_area(img_coords, points, x1, y1, x2, y2):
    # Create a mask for the rectangular area
    mask = (img_coords[:, 0] >= x1) & (img_coords[:, 0] <= x2) & (img_coords[:, 1] >= y1) & (img_coords[:, 1] <= y2)
    
    # Apply the mask to filter the points
    filtered_points = points[mask]
    
    return filtered_points


def point_cloud_to_image(points, h_fov, v_fov, img_width, img_height, min_distance, distance_weight=0.1, intensity_weight=0.9, alpha_factor=0.9,blur_sigma=0.1):
    # Convert FOVs and resolutions from degrees to radians
    h_fov = np.radians(h_fov)
    v_fov = np.radians(v_fov)
    # Initialize the image
    image = np.zeros((img_height, img_width, 3), dtype=np.float32)
    # Compute the Euclidean distance for each point
    point_distances = np.linalg.norm(points[:, :3], axis=1)
    # Filter out points that are closer than the minimum distance
    points = points[point_distances <= min_distance]
    point_distances = point_distances[point_distances <= min_distance]
    # Project 3D points to 2D space
    points_2d = np.zeros((points.shape[0], 2))
    points_2d[:, 0] = np.arctan2(points[:, 1], points[:, 0])
    points_2d[:, 1] = np.arctan2(points[:, 2], np.linalg.norm(points[:, :2], axis=1))
    # Map 2D points to image coordinates
    img_coords = np.zeros_like(points_2d, dtype=np.int16)
    img_coords[:, 0] = np.round((points_2d[:, 0] + h_fov / 2) * img_width / h_fov).astype(np.int16)
    img_coords[:, 1] = np.round((points_2d[:, 1] + v_fov / 2) * img_height / v_fov).astype(np.int16)
    # Create a mask for valid image coordinates
    valid_mask = (0 <= img_coords[:, 0]) & (img_coords[:, 0] < img_width) & (0 <= img_coords[:, 1]) & (img_coords[:, 1] < img_height)
    # Apply mask to image coordinates and corresponding intensities
    valid_img_coords = img_coords[valid_mask]
    valid_intensities = points[:, 3][valid_mask]
    valid_distances = point_distances[valid_mask]
    # Normalize intensities and distances
    normalized_intensities = valid_intensities / np.max(valid_intensities)
    normalized_distances = (valid_distances - np.min(valid_distances)) / (np.max(valid_distances) - np.min(valid_distances))
    # Combine intensity and distance information
    combined_metric = distance_weight * normalized_distances + intensity_weight * normalized_intensities
    # combined_metric_ref = (1-distance_weight) * normalized_distances + (1-intensity_weight) * normalized_intensities
    # Convert combined metric to HSV colors with increased saturation and value
    hsv_colors = np.array([colorsys.hsv_to_rgb(i, 1, 1) for i in combined_metric]) * np.array([1, 1.5, 1.5])
    hsv_colors = np.clip(hsv_colors, 0, 1)  # Clip values to the valid range [0, 1]
    # Update image with HSV colors using advanced indexing
    hsv_colors = np.array([colorsys.hsv_to_rgb(i, 1, 1) for i in combined_metric])
    # Update image with HSV colors using advanced indexing
    image[valid_img_coords[:, 1], valid_img_coords[:, 0]] = hsv_colors
    # Convert to 3-channel image
    image = (image * 255).astype(np.uint8)
    # Flip the image along the vertical axis
    image = np.flipud(image)
    image = set_zero_area_to_255(image, 10)    
    image = remove_black_snowflakes(image)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return image,valid_img_coords, points[valid_mask]

def save_point_cloud_as_pcd(point_cloud_data, output_file):
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(point_cloud_data)
    o3d.io.write_point_cloud(output_file, point_cloud)
    
def point_cloud_to_pcd(point_cloud_data):
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(point_cloud_data)
    return point_cloud

def get_box(box,index):
    x1 = int(box[index][0])
    y1 = int(box[index][1])
    x2 = int(box[index][2])
    y2 = int(box[index][3])
    return x1,y1,x2,y2


def get_index(list_1):
    index_person=[]
    for i in range(len(list_1)):
        if list_1[i] == 0.0:
            index_person.append(i)
        else:
            pass
    return index_person


def get_ori_box(x1,y1):
    return x1,275-y1


def get_init_position(pnt,eps,min_points):
    """ extract drone pointcloud

    Args:
        pnt: N x 3 point clouds
        eps: the distance to neighbors in a cluster
        min_points: the minimum number of points

    Returns:
        [array]: initial position of UAV
    """
    pointcloud = copy.deepcopy(pnt)
    
    labels = np.array(pointcloud.cluster_dbscan(eps, min_points, print_progress=True))
    dis = []
    for label_i in np.unique(labels):
        person_label = np.array(np.where(labels==label_i))
        person_pnt = pointcloud.select_by_index(person_label[0])
        if (np.linalg.norm(person_pnt.get_center() - np.array([0,0,0]))) == 0:
            dis.append(1000)
        else: 
            dis.append(np.linalg.norm(person_pnt.get_center() - np.array([0,0,0])))

    index = np.where(dis == np.min(dis))
    drone_label = np.array(np.where(labels==index[0][0]-1))
    return pointcloud.select_by_index(drone_label[0]).get_center()