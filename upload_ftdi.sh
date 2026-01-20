#!/bin/bash
echo "=== FTDI PRO MINI UPLOAD SCRIPT ==="
echo ""

# Find FTDI port
echo "Looking for FTDI device..."
PORTS=$(ls /dev/ttyUSB* 2>/dev/null)
if [ -z "$PORTS" ]; then
    echo "❌ No USB serial ports found!"
    echo "Connect FTDI cable and try again."
    exit 1
fi

FTDI_PORT=$(echo "$PORTS" | head -1)
echo "✅ Found FTDI at: $FTDI_PORT"

# Update platformio.ini with correct port
echo "Updating platformio.ini with port: $FTDI_PORT"
sed -i "s|upload_port = .*|upload_port = $FTDI_PORT|" platformio.ini

# Set permissions
echo "Setting permissions..."
sudo chmod 666 $FTDI_PORT 2>/dev/null

echo ""
echo "=== WIRING CHECKLIST ==="
echo "FTDI cable → Pro Mini:"
echo "1. BLACK (GND)  → Pro Mini GND"
echo "2. RED   (VCC)  → Pro Mini RAW (NOT VCC!)"
echo "3. ORANGE (TX)  → Pro Mini RXI (Pin 2)"
echo "4. YELLOW (RX)  → Pro Mini TXO (Pin 3)"
echo "5. GREEN  (DTR) → Pro Mini DTR"
echo ""
echo "Press Enter when ALL wires are connected..."
read

echo ""
echo "=== BUILDING CODE ==="
pio run

if [ $? -ne 0 ]; then
    echo "❌ Build failed! Check your code."
    exit 1
fi

echo ""
echo "=== UPLOADING ==="
echo "Using FTDI auto-reset (DTR pin)..."
pio run --target upload

# Check result
if [ $? -eq 0 ]; then
    echo ""
    echo "✅✅✅ SUCCESS! ✅✅✅"
    echo ""
    echo "Code uploaded successfully!"
    echo ""
    echo "=== NEXT STEPS ==="
    echo "1. Open serial monitor to see output:"
    echo "   pio device monitor"
    echo ""
    echo "2. You should see:"
    echo "   - 3 short beeps"
    echo "   - Initialization messages"
    echo "   - 'DRONE READY'"
    echo ""
    echo "3. Test with transmitter:"
    echo "   - Throttle MIN + AUX1 HIGH to arm"
    echo "   - Increase throttle slowly"
else
    echo ""
    echo "❌ Upload failed!"
    echo ""
    echo "=== TROUBLESHOOTING ==="
    echo "1. Trying manual reset method..."
    echo ""
    echo "   Disconnect GREEN DTR wire"
    echo "   Press and HOLD reset button on Pro Mini"
    echo "   Press Enter to continue..."
    read
    echo "   Starting upload in 2 seconds..."
    sleep 2
    echo "   RELEASE reset button NOW!"
    pio run --target upload
    
    if [ $? -eq 0 ]; then
        echo "✅ Manual reset worked!"
    else
        echo "❌ Still failed. Check:"
        echo "   - Power (VCC→RAW, not VCC)"
        echo "   - TX/RX crossover (TX→RXI, RX→TXO)"
        echo "   - FTDI voltage jumper (set to 5V)"
    fi
fi
