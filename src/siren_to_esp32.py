#!/usr/bin/env python3
"""
siren_to_esp32.py — Laptop ML Inference → ESP32-CAM Trigger
=============================================================

Runs the Siren-Sense TFLite model on the laptop's microphone in real-time.
When an emergency siren is detected, triggers the ESP32-CAM ambulance
detection pipeline via WiFi (HTTP) or UART (serial).

The ESP32-CAM then:
  1. Captures a camera image
  2. Sends it to YOLO API for ambulance confirmation
  3. If confirmed, transmits LoRa alert to STM32 traffic controller

Modes:
    wifi  — sends HTTP GET to http://ambulance.local/trigger (auto-discovered)
    uart  — sends "SIREN\\n" over USB-UART serial
    both  — tries WiFi first, falls back to UART

Usage:
    python3 src/siren_to_esp32.py                           # auto-discovers ESP32
    python3 src/siren_to_esp32.py --mode wifi                # same, explicit wifi
    python3 src/siren_to_esp32.py --esp32-ip 192.168.1.50    # manual IP
    python3 src/siren_to_esp32.py --mode uart --uart-port /dev/ttyUSB0

Author:  Siren-Sense TinyML Project
License: MIT
"""

from __future__ import annotations

import argparse
import logging
import socket
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import numpy as np

# Re-use the existing inference pipeline
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from inference_pi import (
    AudioRingBuffer,
    EmergencyDetector,
    extract_mel_spectrogram_numpy,
    load_class_names,
    DEFAULT_MODEL,
    DEFAULT_LABELS,
    DEFAULT_THRESHOLD,
    COOLDOWN_SECONDS,
    CONSECUTIVE_HITS,
    SAMPLE_RATE,
    DURATION,
    NUM_SAMPLES,
)

try:
    import sounddevice as sd
except ImportError:
    logging.error("sounddevice is required. Install: pip install sounddevice")
    sys.exit(1)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)

# Default mDNS hostname (must match MDNS_HOSTNAME in ambulance_detector.ino)
MDNS_HOSTNAME = "ambulance"


# ====================================================================
# ESP32 AUTO-DISCOVERY via mDNS
# ====================================================================

def discover_esp32(hostname: str = MDNS_HOSTNAME, port: int = 80) -> Optional[str]:
    """
    Auto-discover ESP32-CAM on the local network using mDNS.
    The ESP32 advertises itself as '<hostname>.local'.
    Returns the IP address string, or None if not found.
    """
    mdns_name = f"{hostname}.local"
    logger.info("[Discovery] Looking for ESP32 at %s ...", mdns_name)

    # Method 1: Direct socket resolution (works on most Linux systems)
    try:
        ip = socket.getaddrinfo(mdns_name, port, socket.AF_INET)[0][4][0]
        logger.info("[Discovery] ✅ Found ESP32 at %s (%s)", mdns_name, ip)
        return ip
    except socket.gaierror:
        pass

    # Method 2: Try with avahi-resolve (Linux with avahi-daemon)
    try:
        import subprocess
        result = subprocess.run(
            ["avahi-resolve", "-n", mdns_name],
            capture_output=True, text=True, timeout=5,
        )
        if result.returncode == 0 and result.stdout.strip():
            ip = result.stdout.strip().split()[-1]
            logger.info("[Discovery] ✅ Found ESP32 via avahi: %s", ip)
            return ip
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    # Method 3: Try with getent (some Linux systems)
    try:
        import subprocess
        result = subprocess.run(
            ["getent", "hosts", mdns_name],
            capture_output=True, text=True, timeout=5,
        )
        if result.returncode == 0 and result.stdout.strip():
            ip = result.stdout.strip().split()[0]
            logger.info("[Discovery] ✅ Found ESP32 via getent: %s", ip)
            return ip
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    # Method 4: Try zeroconf Python library if installed
    try:
        from zeroconf import Zeroconf, ServiceBrowser
        import time as _time

        zc = Zeroconf()
        info = zc.get_service_info("_http._tcp.local.", f"{hostname}._http._tcp.local.", timeout=5000)
        if info:
            ip = socket.inet_ntoa(info.addresses[0])
            logger.info("[Discovery] ✅ Found ESP32 via zeroconf: %s", ip)
            zc.close()
            return ip
        zc.close()
    except ImportError:
        pass
    except Exception:
        pass

    logger.warning("[Discovery] Could not find %s on the network.", mdns_name)
    logger.warning("[Discovery] Tip: install avahi-daemon:  sudo apt install avahi-daemon")
    return None


# ====================================================================
# ESP32 TRIGGER — WiFi HTTP
# ====================================================================

def trigger_esp32_wifi(esp32_ip: str, port: int = 80) -> bool:
    """
    Send HTTP GET to ESP32-CAM /trigger endpoint.
    Returns True if trigger was accepted.
    """
    import urllib.request
    import urllib.error
    import json

    url = f"http://{esp32_ip}:{port}/trigger"
    logger.info("[WiFi] Triggering ESP32: %s", url)

    try:
        req = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(req, timeout=5) as resp:
            body = resp.read().decode("utf-8")
            logger.info("[WiFi] ESP32 response: %s", body)

            try:
                data = json.loads(body)
                status = data.get("status", "")
                if status == "triggered":
                    return True
                elif status == "cooldown":
                    remaining = data.get("remaining_seconds", "?")
                    logger.info("[WiFi] ESP32 is in cooldown (%ss remaining).", remaining)
                    return True  # Not an error — cooldown is expected
                else:
                    return True
            except json.JSONDecodeError:
                return True  # Got a response, probably fine

    except urllib.error.URLError as e:
        logger.error("[WiFi] Connection to ESP32 failed: %s", e)
        return False
    except Exception as e:
        logger.error("[WiFi] Unexpected error: %s", e)
        return False


def check_esp32_health(esp32_ip: str, port: int = 80) -> bool:
    """Check if ESP32-CAM is reachable."""
    import urllib.request
    import urllib.error

    url = f"http://{esp32_ip}:{port}/health"
    try:
        with urllib.request.urlopen(url, timeout=3) as resp:
            return resp.read().decode("utf-8").strip() == "OK"
    except Exception:
        return False


def get_esp32_status(esp32_ip: str, port: int = 80) -> Optional[dict]:
    """Get ESP32-CAM system status."""
    import urllib.request
    import json

    url = f"http://{esp32_ip}:{port}/status"
    try:
        with urllib.request.urlopen(url, timeout=3) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except Exception:
        return None


# ====================================================================
# ESP32 TRIGGER — UART
# ====================================================================

_uart_port = None


def init_uart(port_name: str, baud: int = 115200):
    """Open serial port for UART trigger."""
    global _uart_port
    try:
        import serial
    except ImportError:
        logger.error("pyserial is required for UART mode. Install: pip install pyserial")
        sys.exit(1)

    try:
        _uart_port = serial.Serial(port_name, baud, timeout=1)
        logger.info("[UART] Port %s opened at %d baud.", port_name, baud)
        return True
    except Exception as e:
        logger.error("[UART] Cannot open %s: %s", port_name, e)
        return False


def trigger_esp32_uart() -> bool:
    """Send 'SIREN\\n' command over UART."""
    global _uart_port
    if _uart_port is None or not _uart_port.is_open:
        logger.error("[UART] Port not open!")
        return False

    try:
        _uart_port.write(b"SIREN\n")
        _uart_port.flush()
        logger.info("[UART] Sent SIREN trigger to ESP32.")
        return True
    except Exception as e:
        logger.error("[UART] Write error: %s", e)
        return False


def find_uart_port() -> str:
    """Auto-detect USB-UART adapter port."""
    import glob
    candidates = (
        glob.glob("/dev/ttyUSB*") +
        glob.glob("/dev/ttyACM*") +
        glob.glob("/dev/tty.usbserial*")
    )
    if candidates:
        return candidates[0]
    return "/dev/ttyUSB0"


# ====================================================================
# COMBINED TRIGGER
# ====================================================================

def trigger_esp32(
    mode: str,
    esp32_ip: Optional[str] = None,
    http_port: int = 80,
) -> bool:
    """
    Trigger ESP32-CAM based on selected mode.
    Returns True if trigger was sent successfully.
    """
    if mode == "wifi":
        if not esp32_ip:
            logger.error("ESP32 IP not configured!")
            return False
        return trigger_esp32_wifi(esp32_ip, http_port)

    elif mode == "uart":
        return trigger_esp32_uart()

    elif mode == "both":
        # Try WiFi first
        if esp32_ip and trigger_esp32_wifi(esp32_ip, http_port):
            return True
        # Fall back to UART
        logger.info("[Trigger] WiFi failed, trying UART...")
        return trigger_esp32_uart()

    else:
        logger.error("Unknown trigger mode: %s", mode)
        return False


# ====================================================================
# MAIN INFERENCE + TRIGGER LOOP
# ====================================================================

def run_siren_detector(
    model_path: str,
    class_names: List[str],
    emergency_label: str,
    threshold: float,
    cooldown: float,
    consecutive: int,
    trigger_mode: str,
    esp32_ip: Optional[str],
    http_port: int,
) -> None:
    """
    Infinite loop: record audio → ML inference → trigger ESP32 on detection.
    """
    detector = EmergencyDetector(model_path, class_names)
    ring = AudioRingBuffer(NUM_SAMPLES)

    # Check ESP32 connectivity before starting
    if trigger_mode in ("wifi", "both") and esp32_ip:
        logger.info("Checking ESP32-CAM connectivity...")
        if check_esp32_health(esp32_ip, http_port):
            logger.info("✅ ESP32-CAM is reachable at %s", esp32_ip)
            status = get_esp32_status(esp32_ip, http_port)
            if status:
                logger.info("  Camera: %s | LoRa: %s | Lane: %s",
                    "OK" if status.get("camera") else "FAIL",
                    "OK" if status.get("lora") else "FAIL",
                    status.get("lane_id", "?"))
        else:
            logger.warning("⚠️  ESP32-CAM not reachable at %s. Will keep trying.", esp32_ip)

    # Start microphone stream
    logger.info("Starting audio stream @ %d Hz …", SAMPLE_RATE)
    stream = sd.InputStream(
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype="float32",
        blocksize=int(SAMPLE_RATE * 0.5),
        callback=ring.callback,
    )

    hit_count = 0
    last_trigger_time = 0.0
    inference_count = 0
    trigger_count = 0

    print("\n" + "=" * 65)
    print("  🚨  Siren Sense ML → ESP32-CAM Ambulance Detector")
    print(f"  🎯  Target label  : {emergency_label}")
    print(f"  📊  Threshold     : {threshold:.0%}")
    print(f"  🔁  Consecutive   : {consecutive} frames")
    print(f"  ⏱️   Cooldown      : {cooldown} s")
    print(f"  🎤  Audio window  : {DURATION} s")
    print(f"  📡  Trigger mode  : {trigger_mode}")
    if esp32_ip:
        print(f"  🌐  ESP32-CAM IP  : {esp32_ip}:{http_port}")
    print("  🛑  Press Ctrl+C to stop")
    print("=" * 65 + "\n")

    try:
        stream.start()
        time.sleep(DURATION + 0.5)

        while True:
            audio = ring.get_audio()
            if audio is None:
                time.sleep(0.1)
                continue

            try:
                mel_spec = extract_mel_spectrogram_numpy(audio)
            except Exception as exc:
                logger.warning("Feature extraction failed: %s", exc)
                time.sleep(0.2)
                continue

            label, confidence, probs = detector.predict(mel_spec)
            inference_count += 1

            # Live status display
            if inference_count % 2 == 0:
                probs_str = "  ".join(
                    f"{class_names[i]}:{probs[i]:.2f}"
                    for i in range(len(class_names))
                )
                sys.stdout.write(f"\r  [{probs_str}]  triggers:{trigger_count}   ")
                sys.stdout.flush()

            # Detection logic
            if label == emergency_label and confidence >= threshold:
                hit_count += 1
            else:
                hit_count = max(0, hit_count - 1)

            now = time.time()
            if hit_count >= consecutive and (now - last_trigger_time) > cooldown:
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(
                    f"\n\n  🚨 SIREN DETECTED! [{timestamp}] "
                    f"confidence={confidence:.1%}"
                )
                print("  📡 Triggering ESP32-CAM ambulance verification...")

                # Send trigger to ESP32-CAM
                success = trigger_esp32(trigger_mode, esp32_ip, http_port)
                if success:
                    trigger_count += 1
                    print("  ✅ ESP32-CAM triggered! Camera→YOLO→LoRa pipeline running.\n")
                else:
                    print("  ❌ Failed to reach ESP32-CAM! Check connection.\n")

                hit_count = 0
                last_trigger_time = now

            time.sleep(1.0)

    except KeyboardInterrupt:
        logger.info("Stopping …")
    finally:
        stream.stop()
        stream.close()
        if _uart_port and _uart_port.is_open:
            _uart_port.close()
        logger.info(
            "Done. Total inferences: %d | Total triggers sent: %d",
            inference_count, trigger_count,
        )


# ====================================================================
# CLI
# ====================================================================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run ML siren detection on laptop mic. "
            "When detected, trigger ESP32-CAM for camera→YOLO→LoRa ambulance verification."
        ),
    )
    parser.add_argument(
        "--mode", choices=["wifi", "uart", "both"], default="wifi",
        help="How to trigger ESP32-CAM (default: wifi).",
    )
    parser.add_argument(
        "--esp32-ip", type=str, default=None,
        help="ESP32-CAM IP address. Auto-discovered via mDNS if not given.",
    )
    parser.add_argument(
        "--http-port", type=int, default=80,
        help="ESP32 HTTP server port (default: 80).",
    )
    parser.add_argument(
        "--uart-port", type=str, default=None,
        help="Serial port for UART mode (auto-detects if not given).",
    )
    parser.add_argument(
        "--uart-baud", type=int, default=115200,
        help="UART baud rate (default: 115200).",
    )
    parser.add_argument(
        "--model", type=str, default=str(DEFAULT_MODEL),
        help="Path to .tflite model file.",
    )
    parser.add_argument(
        "--labels", type=str, default=str(DEFAULT_LABELS),
        help="Path to class_names.txt.",
    )
    parser.add_argument(
        "--emergency-label", type=str, default="emergency",
        help="Class label for emergency sounds.",
    )
    parser.add_argument(
        "--threshold", type=float, default=DEFAULT_THRESHOLD,
        help="Detection confidence threshold (0-1).",
    )
    parser.add_argument(
        "--cooldown", type=float, default=COOLDOWN_SECONDS,
        help="Seconds between trigger events.",
    )
    parser.add_argument(
        "--consecutive", type=int, default=CONSECUTIVE_HITS,
        help="Consecutive detections required before triggering.",
    )
    parser.add_argument(
        "--list-devices", action="store_true",
        help="List audio devices and exit.",
    )
    parser.add_argument(
        "--check-esp32", action="store_true",
        help="Check ESP32-CAM status and exit.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.list_devices:
        print(sd.query_devices())
        return

    # Auto-discover ESP32 if IP not given
    if args.mode in ("wifi", "both") and not args.esp32_ip:
        logger.info("No --esp32-ip given. Auto-discovering ESP32 via mDNS...")
        discovered_ip = discover_esp32(port=args.http_port)
        if discovered_ip:
            args.esp32_ip = discovered_ip
        else:
            # Fall back to mDNS hostname directly (some systems resolve .local in urllib)
            args.esp32_ip = f"{MDNS_HOSTNAME}.local"
            logger.info("[Discovery] Using hostname: %s", args.esp32_ip)

    # Check ESP32 status if requested
    if args.check_esp32:
        ip = args.esp32_ip or f"{MDNS_HOSTNAME}.local"
        status = get_esp32_status(ip, args.http_port)
        if status:
            import json
            print(f"ESP32-CAM Status (at {ip}):")
            print(json.dumps(status, indent=2))
        else:
            print(f"Cannot reach ESP32-CAM at {ip}:{args.http_port}")
            print("Tip: Is your ESP32 powered on and on the same WiFi?")
        return

    # Initialize UART if needed
    if args.mode in ("uart", "both"):
        port_name = args.uart_port or find_uart_port()
        if not init_uart(port_name, args.uart_baud):
            if args.mode == "uart":
                sys.exit(1)
            else:
                logger.warning("UART init failed, will use WiFi only.")

    # Load class names
    class_names = load_class_names(Path(args.labels))

    emergency_label = args.emergency_label
    if emergency_label not in class_names:
        candidates = [c for c in class_names if "emergency" in c.lower() or "siren" in c.lower()]
        if candidates:
            emergency_label = candidates[0]
            logger.info("Auto-detected emergency label: '%s'", emergency_label)
        else:
            logger.error(
                "Emergency label '%s' not found in class names %s",
                emergency_label, class_names,
            )
            sys.exit(1)

    run_siren_detector(
        model_path=args.model,
        class_names=class_names,
        emergency_label=emergency_label,
        threshold=args.threshold,
        cooldown=args.cooldown,
        consecutive=args.consecutive,
        trigger_mode=args.mode,
        esp32_ip=args.esp32_ip,
        http_port=args.http_port,
    )


if __name__ == "__main__":
    main()
