#!/bin/bash

# Configuration
CHIP="PY32F002Bx5"
PACK="Puya.PY32F0xx_DFP.1.2.8.pack"
HEX="Project.hex"
PROBE_ID="CZ_2023_6688"  # Your specific DAPLink ID

echo "---------------------------------------"
echo "🚀 PROBE-RS PRODUCTION: TARGETING $PROBE_ID"
echo "---------------------------------------"

while true; do
    echo "READY: Press board onto pins..."

    # 1. Wait until the specific probe sees a chip
    # We add --probe to lock it to your CZ device
    until probe-rs info --probe "$PROBE_ID" --chip "$CHIP" --chip-description "$PACK" > /dev/null 2>&1; do
        sleep 0.1
    done

    echo "⚡️ Chip detected! Flashing..."

    # 2. Flash using the specific probe
    if probe-rs download --probe "$PROBE_ID" --chip "$CHIP" --chip-description "$PACK" --speed 10000 "$HEX"; then
        echo "✅ SUCCESS!"
        afplay /System/Library/Sounds/Glass.aiff
        
        echo "RELEASE BOARD."
        # Wait for disconnect
        while probe-rs info --probe "$PROBE_ID" --chip "$CHIP" --chip-description "$PACK" > /dev/null 2>&1; do
            sleep 0.2
        done
        echo "---------------------------------------"
    else
        echo "❌ FAILED. Check pogo pin alignment."
        afplay /System/Library/Sounds/Basso.aiff
        sleep 1
    fi
done