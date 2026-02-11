#!/bin/bash

# Configuration
APP_NAME="PyFlash"
SCRIPT="auto_flash.py"
PACK_FILE="Puya.PY32F0xx_DFP.1.2.8.pack"
UI_FILE="index.html"
STATUS_FILE="status.json"

echo "🚀 Starting build for $APP_NAME..."

# Ensure we are in the script directory
cd "$(dirname "$0")"

# Clean previous builds
rm -rf build dist

# Run PyInstaller
# --onefile: single executable
# --add-data: path/to/source:destination_in_bundle
# Note: On Mac, the separator for add-data is colon (:)
pyinstaller --noconfirm --onefile --windowed \
    --name "$APP_NAME" \
    --add-data "$PACK_FILE:." \
    --add-data "$UI_FILE:." \
    --add-data "$STATUS_FILE:." \
    --add-data "Glass.aiff:." \
    --add-data "Ping.aiff:." \
    "$SCRIPT"

if [ $? -eq 0 ]; then
    echo "✅ Build successful! Executable is in dist/$APP_NAME"
    
    # Optional: Create DMG (requires create-dmg)
    # brew install create-dmg
    # create-dmg "dist/$APP_NAME.app"
else
    echo "❌ Build failed."
    exit 1
fi
