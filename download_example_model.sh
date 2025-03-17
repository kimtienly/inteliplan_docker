#!/bin/bash
set -e  # Exit immediately if a command fails

MODEL_DIR="/inteliplan_ws/src/inteliplan_robot/inteliplan_interface/models"
ZIP_PATH="/tmp/models.zip"

# Ensure the models directory exists
mkdir -p "$MODEL_DIR"

# Check if the folder is empty before downloading
if [ -z "$(ls -A $MODEL_DIR)" ]; then
    echo "Downloading models zip file..."
    
    # Use the file ID for the ZIP file from Google Drive
    gdown 1ylmPOxXACW0A-mc_BadXXiYRY_BffM0A -O "$ZIP_PATH"
    
    echo "Extracting models..."
    unzip "$ZIP_PATH" -d "$MODEL_DIR"
    
    echo "Cleaning up..."
    rm "$ZIP_PATH"
    
    echo "Download and extraction complete."
else
    echo "Models folder is not empty, skipping download."
fi

# Continue with the container's main process
exec "$@"
