#!/bin/bash
echo "=== DRONE UPLOAD - WORKING VERSION ==="

# Add PlatformIO to PATH
export PATH="$HOME/.platformio/penv/bin:$PATH"

# Check pio
if ! command -v pio &> /dev/null; then
    echo "❌ pio not found!"
    exit 1
fi

echo "✅ PlatformIO ready: $(which pio)"

# Check wiring
echo ""
echo "=== FINAL WIRING CHECK ==="
echo "FTDI → Pro Mini:"
echo "1. BLACK (GND)  → Pro Mini GND"
echo "2. RED   (VCC)  → Pro Mini RAW (NOT VCC!)"
echo "3. ORANGE (TX)  → Pro Mini RXI (Pin 2)"
echo "4. YELLOW (RX)  → Pro Mini TXO (Pin 3)"
echo "5. GREEN  (DTR) → Pro Mini DTR"
echo ""
echo "Press Enter when ready..."
read

# Build
echo "Building drone code..."
pio run

if [ $? -ne 0 ]; then
    echo "❌ Build failed!"
    exit 1
fi

echo ""
echo "=== UPLOADING ==="
echo "Using FTDI auto-reset..."
pio run --target upload

if [ $? -eq 0 ]; then
    echo "✅✅✅ SUCCESS! ✅✅✅"
    echo "Open serial monitor: pio device monitor"
else
    echo "❌ Auto-reset failed. Trying manual..."
    echo ""
    echo "1. Disconnect GREEN DTR wire"
    echo "2. Press and HOLD reset button"
    echo "3. Press Enter"
    read
    sleep 1
    echo "4. RELEASE reset NOW!"
    pio run --target upload
fi
