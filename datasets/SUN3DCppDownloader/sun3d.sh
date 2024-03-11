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

# Build the dataset downloader if not already built
BUILD_DIR="$SCRIPT_DIR/build"
if [ ! -d "$BUILD_DIR" ]; then
    echo "Building sun3d sequence downloader..."
    mkdir -p "$BUILD_DIR" && cd "$BUILD_DIR"
    cmake "$SCRIPT_DIR"
    make -j4
    echo "Sun3d sequence downloader is built successfully"
fi

# Path to the file containing sequence names
SEQUENCES_FILE="$SCRIPT_DIR/sequences.txt"

# Check if sequences file exists
if [ ! -f "$SEQUENCES_FILE" ]; then
    echo "Sequences file ($SEQUENCES_FILE) not found, please create it." >&2
    exit 1
fi

# Download function
download_sequence() {
    local sequence="${1}/"  # Append a slash to the end of the sequence name
    local target_dir="$DATA_DIR"
    mkdir -p "$target_dir"
    echo "Downloading sequence: $sequence to $target_dir"
    "$SCRIPT_DIR/bin/SUN3D_Downloader" "$sequence" "$target_dir"
}

# Read each line from sequences file and download the sequence
while IFS= read -r sequence; do
    download_sequence "$sequence"
done < "$SEQUENCES_FILE"

echo "All sequences have been downloaded."
echo "Finished at: $(date '+%Y-%m-%d %H:%M:%S')"
