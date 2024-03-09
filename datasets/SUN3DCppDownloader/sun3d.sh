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

# Create DATA_DIR if it doesn't exist
mkdir -p "$DATA_DIR"

# List of sequences to download
sequences=(
    "hotel_umd/maryland_hotel3/"
    "mit_76_studyroom/76-1studyroom2/"
    "mit_dorm_next_sj/dorm_next_sj_oct_30_2012_scan1_erika/"
    "mit_32_d507/d507_2/"
    "mit_lab_hj/lab_hj_tea_nov_2_2012_scan1_erika/"
    "harvard_c6/hv_c6_1/"
    "harvard_c5/hv_c5_1/"
    "harvard_c8/hv_c8_3/"
)

# Build the dataset downloader if not already built
BUILD_DIR="$SCRIPT_DIR/build"
if [ ! -d "$BUILD_DIR" ]; then
    echo "Building sun3d sequence downloader..."
    mkdir -p "$BUILD_DIR" && cd "$BUILD_DIR"
    cmake "$SCRIPT_DIR"
    make -j4
    echo "Sun3d sequence downloader is built successfully"
fi

# Download function
download_sequence() {
    local sequence=$1
    local target_dir="$DATA_DIR"
    mkdir -p "$target_dir"
    echo "Downloading sequence: $sequence to $target_dir"
    "$BUILD_DIR/SUN3DCppDownloader" "$sequence" "$target_dir"
}

# Loop through each sequence and use SUN3DCppDownloader to download it
for sequence in "${sequences[@]}"; do
    download_sequence "$sequence"
done

echo "All sequences have been downloaded."
