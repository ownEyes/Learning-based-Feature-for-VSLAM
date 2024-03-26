import cv2
import numpy as np
from sklearn.neighbors import NearestNeighbors

def build_training_pairs(img_pairs, depth_pairs, relative_poses, 
                         intrinsics_matrix, k=10, distance_threshold=30,
                         pairs=None):
    if pairs is None:
        pairs = []

    for ((img1_path, img2_path), (depth1_path, depth2_path), pose) in zip(img_pairs, depth_pairs, relative_poses):
        # Load images and depth maps
        img1 = cv2.imread(img1_path, cv2.IMREAD_GRAYSCALE)
        img2 = cv2.imread(img2_path, cv2.IMREAD_GRAYSCALE)
        depth1 = cv2.imread(depth1_path, cv2.IMREAD_UNCHANGED)

        # Ensure the depth image is of unsigned 16-bit type, which is expected for depth images
        if depth1.dtype == np.uint16:
        # Perform the bit manipulation operation on the entire array
            depth1 = np.bitwise_or(np.left_shift(depth1, 13), np.right_shift(depth1, 3))
        else:
            print("Depth image is not in the expected format.")
        
        # Step 1: Extract SIFT features from the first image
        # keypoints1 = shi_tomasi_keypoint_only_on_grids(img1)
        keypoints1 = []
        corners1 = cv2.goodFeaturesToTrack(img1, 100, 0.01, 10)
        if corners1 is not None:
            # Adjust corner positions to global image coordinates and create KeyPoints
            for corner in np.float32(corners1):
                # Create a dummy size and angle as Shi-Tomasi doesn't provide these
                kp = cv2.KeyPoint(corner[0][0], corner[0][1], size=1)  # Size is arbitrary
                keypoints1.append(kp)
    
        # Convert keypoints to an array
        points1 = cv2.KeyPoint_convert(keypoints1)

        # Step 2 & 3: Convert 2D points to 3D and transform them according to the relative pose
        filtered_points1,points3D = convert_to_3d(points1, depth1, intrinsics_matrix)
        transformed_points3D = transform_points(points3D, pose)

        # Step 4: Reproject to the second image    
        img2_shape = img2.shape[:2]
        filtered_points, filtered_reprojected_points2 = reproject_and_filter(filtered_points1, transformed_points3D, intrinsics_matrix, img2_shape)

        # Check if reprojected points are empty
        if filtered_reprojected_points2.size == 0:
            print("No reprojected points within image bounds. Skipping this pair.")
            continue  # Skip the rest of the loop and proceed to the next iteration
        
        # Step 5: For negative pairs, find distant features in the second image
        # sift = cv2.SIFT_create()
        # keypoints2 = sift.detect(img2, None)
        # keypoints2 = shi_tomasi_keypoint_only_on_grids(img2)

        keypoints2 = []
        corners2 = cv2.goodFeaturesToTrack(img2, 100, 0.01, 10)
        if corners2 is not None:
            # Adjust corner positions to global image coordinates and create KeyPoints
            for corner in np.float32(corners2):
                # Create a dummy size and angle as Shi-Tomasi doesn't provide these
                kp = cv2.KeyPoint(corner[0][0], corner[0][1], size=1)  # Size is arbitrary
                keypoints2.append(kp)
        points2 = cv2.KeyPoint_convert(keypoints2)
        
        # print("Points2D shape:", points2.shape)
        # print("Filtered Points1 shape:", filtered_points1.shape)
        # print("Transformed Points3D shape:", transformed_points3D.shape)
        # print("Reprojected Points2 shape:", filtered_reprojected_points2.shape)

        
        # Find nearest features and filter out to form negative pairs
        negative_points2 = negative_sample_mining(points2, filtered_reprojected_points2, k, distance_threshold)
        
        
        # print("number of points in imag1:", len(filtered_points))
        # print("number of pos points in imag2:", len(filtered_reprojected_points2))
        # print("number of neg points in imag2:", len(negative_points2))
        
        # Store positive and negative pairs with image path information
        pairs.append({
            'img_paths': (img1_path, img2_path),
            'points1': filtered_points,
            'pos_points2': filtered_reprojected_points2,
            'neg_points2': negative_points2
        })

    return pairs

def convert_to_3d(points2D, depth, intrinsics):
    """
    Convert 2D points to 3D using depth information (scaled by 1/1000 to convert to meters) 
    and camera intrinsics.
    
    :param points2D: Array of 2D points obtained from keypoints (N, 2)
    :param depth: Depth image, depth values are in millimeters
    :param intrinsics: Camera intrinsic matrix (3, 3)
    :return: Array of 3D points (N, 3)
    """
    #points3D = []
    
    # Intrinsic matrix parameters
    fx = intrinsics[0, 0]
    fy = intrinsics[1, 1]
    cx = intrinsics[0, 2]
    cy = intrinsics[1, 2]
    
    # Extract depth values for each point, converting depth from mm to meters
    z = depth[points2D[:, 1].astype(int), points2D[:, 0].astype(int)] / 1000.0
    valid = z > 0

    
    # Filter points2D based on valid mask
    filtered_points2D = points2D[valid]
    
    # Compute 3D coordinates
    x = (points2D[:, 0][valid] - cx) * z[valid] / fx
    y = (points2D[:, 1][valid] - cy) * z[valid] / fy
    points3D = np.vstack((x, y, z[valid])).T
    
    # return np.array(points3D)
    return filtered_points2D, points3D

def transform_points(points3D, pose):
    """
    Apply the relative pose transformation to 3D points.
    
    :param points3D: Array of 3D points (N, 3)
    :param pose: Relative pose transformation matrix (4, 4)
    :return: Array of transformed 3D points (N, 3)
    """
    # Convert 3D points to homogeneous coordinates by adding a column of ones
    points_homogeneous = np.hstack((points3D, np.ones((points3D.shape[0], 1))))
    
    pose_3x4 = np.array(pose)

    # Convert the 3x4 pose matrix to a 4x4 transformation matrix
    T = np.vstack((pose_3x4, [0, 0, 0, 1]))
    
    # Apply the transformation matrix to each point
    # transformed_points_homogeneous = np.dot(points_homogeneous, T)
    T_inverse = np.linalg.inv(T)  # Compute the inverse of T
    transformed_points_homogeneous = np.dot(T_inverse, points_homogeneous.T)
    
    # Convert back from homogeneous to 3D coordinates by dividing by the last element
    transformed_points3D = transformed_points_homogeneous[:3, :].T / transformed_points_homogeneous[3, :][:, np.newaxis]
    
    return transformed_points3D


def reproject_and_filter(points2D, points3D_transformed, intrinsics, img2_shape):
    """
    Reproject transformed 3D points back to the second image and filter out points outside the image bounds.
    
    :param points2D: Original 2D points from the first image (N, 2)
    :param points3D_transformed: Transformed 3D points to be reprojected (N, 3)
    :param intrinsics: Camera intrinsic matrix (3, 3)
    :param img2_shape: Shape of the second image (height, width)
    :return: Filtered 2D points in both images (N', 2)
    """
    # Reproject the transformed 3D points to 2D
    fx, fy = intrinsics[0, 0], intrinsics[1, 1]
    cx, cy = intrinsics[0, 2], intrinsics[1, 2]
    x = (fx * points3D_transformed[:, 0] / points3D_transformed[:, 2]) + cx
    y = (fy * points3D_transformed[:, 1] / points3D_transformed[:, 2]) + cy
    reprojected_points2 = np.vstack((x, y)).T

    # Check bounds and filter points
    img2_height, img2_width = img2_shape
    valid = (x >= 0) & (x < img2_width) & (y >= 0) & (y < img2_height)
    
    return points2D[valid], reprojected_points2[valid]

def negative_sample_mining(points2, reprojected_points, k, threshold):
    """
    Find features in 'points2' that are the most distant from 'reprojected_points',
    subject to being farther away than 'threshold'. If all k nearest neighbors are
    closer than 'threshold', return the most distant one among them.

    :param points2: Detected SIFT features in the second image (M, 2)
    :param reprojected_points: Reprojected points in the second image (N, 2)
    :param k: Number of nearest neighbors to consider
    :param threshold: Distance threshold for considering a point as distant
    :return: Distant features from 'points2'
    """
    distant_features = []
    
    # Initialize NearestNeighbors with k neighbors
    nn = NearestNeighbors(n_neighbors=k, algorithm='auto').fit(points2)
    
    # Find k nearest neighbors for each reprojected point
    distances, indices = nn.kneighbors(reprojected_points)
    
    # Iterate through each reprojected point and its neighbors
    for reproj_point_idx, (dist, idx) in enumerate(zip(distances, indices)):
        # Filter out neighbors closer than the threshold and find the farthest
        distant_idx = np.argmax(dist)  # Index of the farthest neighbor
        if dist[distant_idx] < threshold:
            # If the farthest is closer than the threshold, still consider it as a negative feature
            distant_features.append(points2[idx[distant_idx]])
        else:
            # If there's any neighbor farther than the threshold, consider the farthest among them
            distant_filtered_indices = idx[dist >= threshold]
            if len(distant_filtered_indices) > 0:
                # Append the feature that is farthest among those beyond the threshold
                distant_features.append(points2[distant_filtered_indices[np.argmax(dist[dist >= threshold])]])
            else:
                # If no features are beyond the threshold, append the farthest among the k nearest
                distant_features.append(points2[idx[distant_idx]])

    return np.array(distant_features)
