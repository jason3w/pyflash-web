#!/bin/bash

# Configuration
PACK="$HOME/Downloads/Puya.PY32F0xx_DFP.1.2.8.pack"
HEX="$HOME/Downloads/Project.hex"
TARGET="py32f002bx5"

while true; do
    echo "---------------------------------------"
    echo "READY: Press board to pins..."
    
    # 1. Wait for the USB Probe to be visible to the system
    until pyocd list | grep -q "DAPLink\|CMSIS-DAP\|ST-Link"; do
        sleep 0.2
    done

    # 2. Settle Delay: Wait for electrical contact to stabilize
    sleep 0.3
    echo "⚡️ Flashing..."

    # 3. Attempt Flash
    # We use -u to specify the pack and --target to match your success command
    if pyocd flash -t $TARGET --pack "$PACK" "$HEX"; then
        echo "✅ SUCCESS"
        afplay /System/Library/Sounds/Glass.aiff
        
        echo "WAITing for removal..."
        # 4. Wait for the probe to be disconnected/removed
        while pyocd list | grep -q "DAPLink\|CMSIS-DAP\|ST-Link"; do
            sleep 0.5
        done
        echo "Released."
    else
        echo "❌ ERROR"
        afplay /System/Library/Sounds/Basso.aiff
        # Short cooldown on error to prevent rapid-fire log spam
        sleep 1
    fi
done