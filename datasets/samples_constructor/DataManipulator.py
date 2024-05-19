import shutil
import hashlib
import yaml
import matplotlib.pyplot as plt
import os
import re
import numpy as np
import h5py
from random import sample
import cv2
import matplotlib
matplotlib.use('Agg')


def read_config():
    # Load the configuration from the YAML file
    with open('configuration.yaml', 'r') as file:
        config = yaml.safe_load(file)

    # Safely resolve base paths
    config['dataset_path'] = os.path.abspath(
        os.path.expanduser(config['dataset_path']))
    config['output_path'] = os.path.abspath(
        os.path.expanduser(config['output_path']))
    config['hdf5_file_path'] = os.path.join(
        config['output_path'], 'pairs.hdf5')

    os.makedirs(config['output_path'], exist_ok=True)

    return config


def load_sequences():
    # Initialize an empty list to hold the sequence paths
    sequence_paths = []

    # Open the sequence.txt file and read its contents
    with open('sequences.txt', 'r') as file:
        for line in file:
            # Strip any leading/trailing whitespace from the line, including newline characters
            sequence_path = line.strip()
            # Only add the sequence path to the list if it is not empty
            if sequence_path:  # This will be False for empty strings
                sequence_paths.append(sequence_path)
    return sequence_paths


def load_paths(config, sequence_name):
    # Construct specific paths based on the base paths and sequence name
    config['seq_read_path'] = os.path.join(
        config['dataset_path'], sequence_name)
    config['img_read_path'] = os.path.join(config['seq_read_path'], 'img')
    config['depth_read_path'] = os.path.join(config['seq_read_path'], 'depth')
    config['intrinsics_path'] = os.path.join(
        config['seq_read_path'], 'intrinsics.txt')
    config['poses_path'] = os.path.join(
        config['seq_read_path'], config['poses_file'])

    config['seq_out_path'] = os.path.join(config['output_path'], sequence_name)
    config['img_out_path'] = os.path.join(config['seq_out_path'], 'img')
    config['demo_out_path'] = os.path.join(config['seq_out_path'], 'demo')

    return config


def create_and_copy(config):
    os.makedirs(config['seq_out_path'], exist_ok=True)
    os.makedirs(config['demo_out_path'], exist_ok=True)
    shutil.copytree(config['img_read_path'], config['img_out_path'])
    return


def load_seq(config):
    # load all img and depth file name
    imgs = list_image_files(config['img_read_path'])
    depths = list_image_files(config['depth_read_path'])

    # Read the intrinsics file into a NumPy array (matrix)
    intrinsics_matrix = np.loadtxt(config['intrinsics_path'])

    # Read the pose file and process each line
    with open(config['poses_path'], 'r') as file:
        poses = [process_line(line) for line in file]

    return imgs, depths, intrinsics_matrix, poses


def sort_and_buid_pairs(imgs, depths):
    # make sure the filenames are in correct order
    imgs = sorted(imgs, key=sort_key)
    depths = sorted(depths, key=sort_key)

    # Create pairs for images and depths
    img_pairs = create_neighboring_pairs(imgs)
    depth_pairs = create_neighboring_pairs(depths)

    return img_pairs, depth_pairs


def list_image_files(directory, extensions=['.jpg', '.jpeg', '.png', '.bmp', '.tif', '.tiff']):
    """
    Lists all image files in a specified directory.

    :param directory: The directory to search for image files.
    :param extensions: A list of image file extensions to include.
    :return: A list of paths to image files in the directory.
    """
    image_files = []
    for filename in os.listdir(directory):
        if any(filename.endswith(extension) for extension in extensions):
            image_files.append(os.path.join(directory, filename))
    return image_files

# Function to process a single line of the pose file


def process_line(line):
    # Split the line into values and convert them to floats
    values = [float(value) for value in line.split()]
    # the line is in the format "R R R t R R R t R R R t",
    # where R represents rotation matrix elements and t represents translation vector elements,
    pose = [values[0:4], values[4:8], values[8:12]]
    return pose


def sort_key(filename):
    """
    Custom sort function to extract the numerical part at the beginning of the filename
    and use it as the sorting key.
    """
    match = re.match(r'(\d+)', os.path.basename(filename))
    if match:
        return int(match.group(1))
    return 0  # Default value if no match is found

# Function to create pairs of neighboring files


def create_neighboring_pairs(files):
    return [(files[i], files[i+1]) for i in range(len(files) - 1)]


def compute_hash(pair):
    # Concatenate the pair's content into a single string
    content_string = f"{pair['img_paths']}-{pair['points1'].tobytes()}-{pair['pos_points2'].tobytes()}-{pair['neg_points2'].tobytes()}"

    # Use SHA-256 hash function
    return hashlib.sha256(content_string.encode('utf-8')).hexdigest()


def save_pairs_to_hdf5(hdf, pairs, hdf5_folder):
    def make_relative(path):
        return os.path.relpath(path, hdf5_folder)

    pairs_group = hdf.require_group('pairs')
    print(f"Total saved pairs: {len(pairs_group)}")

    for pair in pairs:
        # Compute the hash for the current pair
        pair_hash = compute_hash(pair)

        # Check if any existing pair has the same hash
        existing_pair = None
        for name, grp in pairs_group.items():
            if grp.attrs.get('hash') == pair_hash:
                existing_pair = grp
                break

        # If the pair already exists, skip adding it
        if existing_pair is not None:
            continue

        # Generate a new unique name for this pair
        pair_name = f"pair_{len(pairs_group)}"
        pair_group = pairs_group.create_group(pair_name)

        # Store the pair data and hash as attributes
        rel_img_paths = (make_relative(
            pair['img_paths'][0]), make_relative(pair['img_paths'][1]))
        pair_group.create_dataset('img_paths', data=np.array(
            rel_img_paths, dtype=h5py.string_dtype()))
        pair_group.create_dataset('points1', data=np.array(pair['points1']))
        pair_group.create_dataset(
            'pos_points2', data=np.array(pair['pos_points2']))
        pair_group.create_dataset(
            'neg_points2', data=np.array(pair['neg_points2']))
        pair_group.attrs['hash'] = pair_hash


def save_sampled_pairs(pairs, demo_path):
    if not pairs:
        print("No pairs to display.")
        return

    # Ensure the demo_path directory exists
    os.makedirs(demo_path, exist_ok=True)

    # Sample 10 pairs if there are enough pairs
    sampled_pairs = sample(pairs, min(10, len(pairs)))

    # Iterate through the sampled pairs and save the images
    for i, pair in enumerate(sampled_pairs):
        # Load images and convert to RGB
        img1_path, img2_path = pair['img_paths']
        img1 = cv2.cvtColor(cv2.imread(
            img1_path, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
        img2 = cv2.cvtColor(cv2.imread(
            img2_path, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)

        # Combine the images horizontally
        combined_img = np.hstack((img1, img2))

        # Create a figure with 1 row and 2 columns for subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        ax1.imshow(combined_img)
        ax1.set_title(f"Positive Pair {i+1}")

        # Draw green lines for positive points
        for p1, p2 in zip(pair['points1'], pair['pos_points2']):
            # Green for positive points in the first image
            ax1.scatter(p1[0], p1[1], color='g', s=10)
            # Offset by img1 width for alignment
            ax1.scatter(p2[0] + img1.shape[1], p2[1], color='g', s=10)

        ax1.axis('off')
        ax2.imshow(combined_img)
        ax2.set_title(f"Negative Pair {i+1}")

        for p3, p4 in zip(pair['points1'], pair['neg_points2']):
            ax2.scatter(p3[0], p3[1], color='r', s=10)
            ax2.scatter(p4[0] + img1.shape[1], p4[1], color='r', s=10)

        ax2.axis('off')

        # Save the figure
        plt.savefig(os.path.join(demo_path, f'pair_{i+1}.png'))
        plt.close(fig)


def count_img_paths(hdf):
    pairs_group = hdf.get('pairs')
    if pairs_group is None:
        print("No pairs saved.")
        return

    total_img_paths = 0

    for pair_name, pair_group in pairs_group.items():
        # Read the entire dataset into memory
        img_paths = pair_group['img_paths'][:]
        # Count the number of image paths in the dataset
        num_img_paths = len(img_paths)
        total_img_paths += num_img_paths  # Accumulate the total count

        # print(f"{pair_name} has {num_img_paths} image paths.")

    print(f"Total saved pairs: {int(total_img_paths/2)}")
