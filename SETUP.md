# PY32 Auto-Flash Setup Guide

This guide outlines how to replicate the stable flashing environment on a new machine.

## 1. Hardware Requirements
- **Debugger**: DAPLink (or any CMSIS-DAP compatible probe).
- **Target**: PY32F002Bx5 series microcontroller.
- **Connection**: SWD (GND, SWDIO, SWCLK). 3.3V is usually supplied by the debugger or an external source.

## 2. Software Dependencies
Ensure [Python 3](https://www.python.org/downloads/) is installed, then install `pyocd`:

```bash
pip install pyocd
```

## 3. Project File Structure
For the script and visuals to work, maintain the following structure in your project folder:

- `auto_flash.py`: The main automation script.
- `Project.hex`: The firmware to be flashed.
- `Puya.PY32F0xx_DFP.1.2.8.pack`: CMSIS-Pack for PY32 support.
- `index.html`: Web visualizer.
- `Glass.aiff`: (Optional) Success sound file.
- `status.json`: (Generated) Created automatically by the script to sync with the web UI.

## 4. Execution Instructions

### A. Start the Automation Script
Run the script in **unbuffered mode** to ensure real-time terminal feedback:

```bash
python3 -u auto_flash.py
```

### B. Start the Web Visualizer (Optional)
In a **separate terminal window**, start a local web server in the same directory:

```bash
python3 -m http.server 8001
```

Once the server is running, open your browser to:
[http://localhost:8001](http://localhost:8001)

## 5. Script Reliability Features
- **Auto-Retry**: The script attempts to connect up to 3 times per board to handle contact bounce.
- **Portability**: It automatically resolves paths for the pack file and sounds relative to the script's location.
- **Visual Sync**: Updates the `status.json` file which is polled by `index.html`.
