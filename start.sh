#!/bin/bash
# ============================================================
#  start.sh — One-Click Launcher for Siren Sense
#  Activates venv, finds ESP32-CAM automatically, starts ML
#  Pipeline: Laptop ML → ESP32-CAM → Roboflow → ESP-NOW → STM32
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo ""
echo "  🚨  Siren Sense — One-Click Launcher"
echo "  ======================================"
echo ""

# 1. Activate virtual environment
if [ -f "venv/bin/activate" ]; then
    source venv/bin/activate
    echo "  ✅ Virtual environment activated"
elif [ -f ".venv/bin/activate" ]; then
    source .venv/bin/activate
    echo "  ✅ Virtual environment activated (.venv)"
else
    echo "  ❌ No venv found! Run first:"
    echo "     python3 -m venv venv && source venv/bin/activate"
    echo "     pip install -r requirements_pc.txt"
    exit 1
fi

# 2. Check model exists
if [ ! -f "models/emergency_detector.tflite" ]; then
    echo "  ❌ Model not found! Train first:"
    echo "     python3 src/train_model.py --epochs 50"
    exit 1
fi
echo "  ✅ Model found"

# 3. Launch — ESP32 is auto-discovered via mDNS
echo ""
echo "  🔍 Auto-discovering ESP32-CAM (ambulance.local)..."
echo "  🎤 Starting microphone + ML inference..."
echo ""

python3 src/siren_to_esp32.py \
    --mode wifi \
    --threshold 0.80 \
    --cooldown 3.0 \
    --consecutive 2
