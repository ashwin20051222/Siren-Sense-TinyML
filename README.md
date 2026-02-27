# 🚨 TinyML Emergency Sound Detection

> **Detailed setup & training instructions are available in** [INSTRUCTIONS.md](INSTRUCTIONS.md)


# 🚨 Emergency Sound Detection (Siren Sense)

This repository contains a pipeline to detect emergency sounds (sirens, alarms) from short audio clips. It includes data collection helpers, a training pipeline, and a compact quantized TFLite model for real-time inference on small devices.

This README explains how to install, run, and use the provided model with common sensors (USB microphone, I2S microphone on ESP32), and how to run inference either on your laptop, on a Raspberry Pi, or on an ESP32 microcontroller.

--

**Contents (high level)**
- `dataset/` — labeled WAV files (3s clips) organized into `emergency/` and `non_emergency/`
- `models/` — saved artifacts: `emergency_detector.h5`, `emergency_detector.tflite`, `class_names.txt`
- `src/` — helper scripts: `record_data.py`, `train_model.py`, `inference_pi.py`
- `requirements_pc.txt` — dependencies for training/development
- `requirements_pi.txt` — lightweight runtime deps for Raspberry Pi

--

**Quick summary (commands)**

1) Create venv and install (training laptop):

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements_pc.txt
```

2) (Optional) Record samples with your laptop microphone:

```bash
python3 src/record_data.py --label emergency --count 50
python3 src/record_data.py --label non_emergency --count 50
```

3) Train (on laptop / server):

```bash
python3 src/train_model.py --epochs 50 --batch-size 32
```

4) Run real-time inference locally (laptop) or on Pi using the TFLite model:

```bash
# On laptop (if you installed TensorFlow):
python3 src/inference_pi.py --model models/emergency_detector.tflite --labels models/class_names.txt --threshold 0.8

# On Raspberry Pi (use tflite-runtime, see requirements_pi.txt):
python3 src/inference_pi.py --threshold 0.8
```

See the `src/inference_pi.py` CLI for available options: `--model`, `--labels`, `--threshold`, `--cooldown`, `--consecutive`, and `--list-devices`.

--

**How to use sensors (microphones)**

- USB microphone (recommended for laptop / Pi): Plug the microphone into USB and run `python3 src/inference_pi.py --list-devices` to find the device index. By default `inference_pi.py` opens the default input device at 16 kHz.

- Built-in laptop microphone: can be used directly but quality depends on hardware and ambient noise.

- ESP32 I2S microphone (two main modes):
  - Mode A — ESP32 streams audio samples to your laptop over serial/Wi-Fi: the ESP32 runs a sketch that reads the I2S mic and sends raw PCM frames; your laptop runs a small receiver script that buffers audio and passes it to `extract_mel_spectrogram_numpy()` and the TFLite interpreter. This is useful when the ESP32 does not run the model locally.
  - Mode B — ESP32 runs the TinyML model locally: convert a model to a microcontroller-friendly format (TFLite for Microcontrollers) and flash it to the board. The ESP32 then outputs detection events over serial or toggles a GPIO/LED.

Below are step-by-step instructions for both modes.

--

**Mode A — Use ESP32 as microphone (stream audio to laptop)**

Requirements:
- ESP32 board (WROOM or similar)
- I2S MEMS microphone (e.g. SPH0645 or INMP441)
- Arduino IDE / PlatformIO (to upload sketch to ESP32)
- Laptop with Python and `requirements_pc.txt` installed

Steps:

1. Upload an Arduino/ESP32 sketch that reads I2S microphone frames and writes PCM samples to serial (e.g., as newline-separated raw int16 values or as binary frames). Example approach:
   - Configure I2S on the ESP32 to capture 16 kHz mono audio.
   - Send fixed-size frames, e.g. 3 seconds worth or smaller blocks (500 ms) as binary to the serial port.

2. On your laptop, run a small Python receiver (example outline):
   - Open the serial port (e.g., `/dev/ttyUSB0`) at the correct baud rate.
   - Reassemble frames into a 3-second buffer at 16000 Hz.
   - Convert the received buffer into float32 [-1, 1], run `extract_mel_spectrogram_numpy()` (see `src/inference_pi.py`), then call the TFLite interpreter with the same preprocessing and shape.

3. Run the receiver script and the local inference loop. If you don't have a receiver script in this repo, the approach above is straightforward to implement using `pyserial`.

Notes and tips:
- Sending raw audio over serial is bandwidth-limited; prefer 16 kHz mono 16-bit PCM. 3 seconds @16k = 96k samples × 2 bytes = ~192 KB, which is large for serial; consider sending 500 ms chunks and reassembling on the laptop.
- Alternatively stream via Wi-Fi (ESP32 acts as a socket server) to avoid serial bandwidth limits.

--

**Mode B — Run the model on ESP32 (TinyML)**

This option runs inference on the microcontroller. It requires that the model and preprocessing fit the board's RAM/flash.

Summary steps:

1. Verify model size and input requirements. The model in `models/emergency_detector.tflite` is quantized — check the input tensor shape in `src/inference_pi.py` to match expected preprocessing.

2. Convert the `.tflite` into a C array (for Arduino) or embed it via the PlatformIO filesystem. Example (Linux):

```bash
## Convert tflite to C source array
xxd -i models/emergency_detector.tflite > model_data.cc
```

3. Use TensorFlow Lite for Microcontrollers in your Arduino/PlatformIO project:
   - Add the `TensorFlowLite` (micro) library or use the Arduino_TensorFlowLite package.
   - Add code to read the I2S mic, compute the required features (note: computing full 128x188 mel spectrogram on ESP32 may be expensive — consider retraining a model that accepts raw waveform frames or MFCCs optimized for the MCU).
   - Load the embedded model and run inference using the TFLite micro interpreter.

4. Flash the ESP32 and monitor serial for detection events (e.g., `screen /dev/ttyUSB0 115200`).

Practical notes:
- Many microcontroller TinyML audio pipelines use MFCC or smaller input shapes to fit RAM. If the provided model is too large for your board, consider training a smaller model or extracting features on the laptop instead.
- If you want a fully working ESP32 example and need help adapting the model to MFCCs or generating the Arduino sketch, I can create an example sketch and a minimal receiver script.

--

**Detailed laptop (local) instructions**

1. Install dependencies (training/development machine):

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements_pc.txt
```

2. Check audio devices:

```bash
python3 src/inference_pi.py --list-devices
```

3. Run real-time detection using the TFLite model (uses the same pure-NumPy mel extraction as training to keep behavior consistent):

```bash
python3 src/inference_pi.py --model models/emergency_detector.tflite --labels models/class_names.txt --threshold 0.8
```

The script prints continuous probability estimates and reports a detection when the configured confidence and consecutive-count are reached. Use `--cooldown` and `--consecutive` to tune sensitivity.

--

**Raspberry Pi runtime**

Install the lightweight runtime and run the same `inference_pi.py` script:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements_pi.txt
python3 src/inference_pi.py --threshold 0.8
```

The Pi pipeline is the most straightforward way to run the provided model in near-real-time using a USB microphone.

--

FAQ / Troubleshooting

- Q: My detection is too sensitive / too quiet.
  - A: Adjust `--threshold`, `--consecutive`, and `--cooldown` on the `inference_pi.py` command line.

- Q: Can I run this on a microcontroller with no changes?
  - A: Possibly, but you will likely need to downsize the model or change preprocessing (MFCCs or smaller time window). If you want, I can help generate a smaller TinyML-friendly model.

- Q: How to log detections?
  - A: `inference_pi.py` prints detection timestamps to stdout. You can pipe them to a file or add callback code to send HTTP requests or toggle GPIO pins.

--

If you want, I can: add a serial receiver script for Mode A, or scaffold an ESP32 Arduino sketch for Mode B and show how to convert the TFLite file into a C array and build the firmware. Tell me which option you prefer and I will implement it.

--

License: MIT
