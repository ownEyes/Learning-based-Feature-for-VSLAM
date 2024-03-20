#!/bin/bash

set -e  # Exit immediately if a command exits with a non-zero status.
trap 'echo "Script interrupted before completion." >&2; exit 1' INT  # Handle script interruption gracefully.

# Function to check required commands
check_command() {
    if ! command -v "$1" &> /dev/null; then
        echo "$1 could not be found, please install it." >&2
        exit 1
    fi
}

# Define the local directory where you want to save the sequences
SCRIPT_DIR=$(dirname "$(realpath "$0")")  # Get the real path to the script, resolving symlinks
DATA_DIR="$SCRIPT_DIR/../sun3d/"
EXTRACT_DIR="$SCRIPT_DIR/../sun3d_extracted/"

# Create EXTRACT_DIR if it doesn't exist
mkdir -p "$EXTRACT_DIR"

# Build the subset extractor if not already built
BUILD_DIR="$SCRIPT_DIR/build"
if [ ! -d "$BUILD_DIR" ]; then
    echo "Building sun3d subset extractor..."
    mkdir -p "$BUILD_DIR" && cd "$BUILD_DIR"
    cmake "$SCRIPT_DIR"
    make -j4
    echo "Sun3d subset extractor is built successfully"
fi

# Path to the file containing sequence names
SEQUENCES_FILE="$SCRIPT_DIR/sequences.txt"

# Check if sequences file exists
if [ ! -f "$SEQUENCES_FILE" ]; then
    echo "Sequences file ($SEQUENCES_FILE) not found, please create it." >&2
    exit 1
fi

process_sequence() {
    local sequence="${1}/"  # Append a slash to the end of the sequence name
    local target_dir="$DATA_DIR"
    mkdir -p "$target_dir"
    echo "Extracting sequence: $sequence to $target_dir"
    "$SCRIPT_DIR/bin/SUN3DGroundtruthOptimizer" "$sequence" "$target_dir"
    # Define the directory where the extracted data will be stored
    EXTRACT_DIR="$SCRIPT_DIR/../sun3d_extracted/${sequence}"
    mkdir -p "$EXTRACT_DIR/img" "$EXTRACT_DIR/depth"

    # Copy images listed in extracted_images.txt to the /img directory
    while IFS= read -r line; do
        cp "$line" "$EXTRACT_DIR/img/"
    done < "$SCRIPT_DIR/extracted_images.txt"

     # Copy depth maps listed in extracted_depths.txt to the /depth directory
    while IFS= read -r line; do
        cp "$line" "$EXTRACT_DIR/depth/"
    done < "$SCRIPT_DIR/extracted_depths.txt"

    # source ~/miniconda3/etc/profile.d/conda.sh
    # conda activate ros2_dl
    # evo_ape kitti "$SCRIPT_DIR/extracted_extrinsics.txt" "$SCRIPT_DIR/optimized_extrinsics.txt" -va --plot  --save_results "$EXTRACT_DIR/ape_results.zip"
    # evo_rpe kitti "$SCRIPT_DIR/extracted_extrinsics.txt" "$SCRIPT_DIR/optimized_extrinsics.txt" --pose_relation angle_deg --delta 0.01 --delta_unit m --plot  --save_results "$EXTRACT_DIR/rpe_results.zip"
    # evo_traj kitti "$SCRIPT_DIR/extracted_extrinsics.txt" "$SCRIPT_DIR/optimized_extrinsics.txt" -p --full_check --save_results "$EXTRACT_DIR/traj_results.zip"

    # Copy the extrinsic and intrinsic files to the new directory
    mv "$SCRIPT_DIR/extracted_extrinsics.txt" "$EXTRACT_DIR/"
    mv "$SCRIPT_DIR/optimized_extrinsics.txt" "$EXTRACT_DIR/"
    mv "$SCRIPT_DIR/relative_transformations.txt" "$EXTRACT_DIR/"
    cp "$target_dir/$sequence/intrinsics.txt" "$EXTRACT_DIR/"

    


    rm -f "$SCRIPT_DIR/extracted_images.txt" "$SCRIPT_DIR/extracted_depths.txt"
}

# Read each line from sequences file and download the sequence
while IFS= read -r sequence; do
    process_sequence "$sequence"
done < "$SEQUENCES_FILE"

echo "All sequences have been extracted."
echo "Finished at: $(date '+%Y-%m-%d %H:%M:%S')"