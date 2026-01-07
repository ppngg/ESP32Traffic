# Smart Traffic Light Control System

An intelligent traffic light management system that uses AI-powered computer vision to detect traffic density and dynamically adjust signal timing. The system integrates ESP32-based edge computing with AWS IoT for cloud connectivity and real-time traffic monitoring.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Architecture](#architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Dependencies](#software-dependencies)
- [Project Structure](#project-structure)
- [Installation & Setup](#installation--setup)
- [Configuration](#configuration)
- [Usage](#usage)
- [Communication Protocol](#communication-protocol)
- [Traffic Light State Machine](#traffic-light-state-machine)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## Overview

This project implements a smart traffic light system that:

1. **Detects Traffic Density**: Uses an ESP32 camera module with TensorFlow Lite Micro to detect motorbikes and compute traffic density in real-time
2. **Dynamic Timing**: Automatically adjusts green light duration based on detected traffic density using a traffic flow algorithm
3. **Cloud Connectivity**: Connects to AWS IoT Core via MQTT for remote monitoring and control
4. **Bidirectional Control**: Receives density updates via MQTT and can operate autonomously with on-device detection

The system supports two operation modes:
- **Autonomous Mode**: Uses on-device camera and AI model to detect traffic and adjust timing
- **Remote Mode**: Receives traffic density data via MQTT from external sensors or monitoring systems

## Features

### Core Features

- ✅ **AI-Powered Traffic Detection**: Real-time motorbike detection using TensorFlow Lite Micro (INT8 quantized model)
- ✅ **Dynamic Green Light Timing**: Traffic-responsive signal timing based on density calculations
- ✅ **AWS IoT Integration**: Secure MQTT communication with AWS IoT Core using mutual TLS authentication
- ✅ **Dual-Direction Control**: Independent traffic density tracking for two directions
- ✅ **Region of Interest (ROI) Configuration**: Configurable detection area within camera frame
- ✅ **Burst Detection Mode**: Multiple frame analysis with median filtering for robust density measurement
- ✅ **Serial Image Output**: Base64-encoded JPEG images over serial for debugging and visualization
- ✅ **Configurable Scoring**: Adjustable detection confidence thresholds

### Technical Features

- **ESP-IDF Framework**: Native ESP32 development using Espressif IoT Development Framework
- **TensorFlow Lite Micro**: Edge AI inference optimized for ESP32
- **ESP32-Camera Driver**: Support for OV2640 camera module
- **FreeRTOS Task Management**: Multi-threaded operation with scheduled tasks
- **Software Timer System**: Non-blocking timer implementation for state machine
- **Arduino Compatibility**: Traffic light control module compatible with Arduino libraries

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Smart Traffic Light System              │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────┐      ┌──────────────┐                    │
│  │   Camera     │      │   ESP32      │                    │
│  │  (OV2640)    │──────│   (ESP-IDF)  │                    │
│  └──────────────┘      └──────┬───────┘                    │
│                               │                            │
│                    ┌──────────┴──────────┐                 │
│                    │                     │                 │
│         ┌──────────▼──────────┐ ┌───────▼────────┐        │
│         │ TensorFlow Lite     │ │  Traffic Light │        │
│         │ Micro Engine        │ │  State Machine │        │
│         │ (Motorbike Detector)│ │  (Arduino)     │        │
│         └──────────┬──────────┘ └───────┬────────┘        │
│                    │                     │                 │
│                    └──────────┬──────────┘                 │
│                               │                            │
│                    ┌──────────▼──────────┐                 │
│                    │   Density Calculator│                 │
│                    │   (H-Fill Algorithm)│                 │
│                    └──────────┬──────────┘                 │
│                               │                            │
│                    ┌──────────▼──────────┐                 │
│                    │   Green Time        │                 │
│                    │   Calculator        │                 │
│                    └─────────────────────┘                 │
│                                                             │
│                    ┌──────────▼──────────┐                 │
│                    │   MQTT Client       │                 │
│                    │   (AWS IoT Core)    │                 │
│                    └─────────────────────┘                 │
│                                                             │
└─────────────────────────────────────────────────────────────┘
                                │
                                │ MQTT (TLS)
                                │
                    ┌───────────▼───────────┐
                    │    AWS IoT Core       │
                    │    (Cloud Backend)    │
                    └───────────────────────┘
```

### System Components

1. **Camera Module (OV2640)**
   - Captures QVGA (320x240) RGB565 frames
   - Configured via ESP32-Camera driver

2. **AI Inference Engine**
   - TensorFlow Lite Micro interpreter
   - INT8 quantized model for efficient edge inference
   - Region of Interest (ROI) filtering
   - Horizontal fill (h-fill) density calculation

3. **Traffic Light Controller**
   - State machine with 4 states (GREEN1, YELLOW1, GREEN2, YELLOW2)
   - Dynamic timing based on traffic density
   - GPIO-controlled LED outputs

4. **MQTT Communication**
   - AWS IoT Core integration
   - Mutual TLS authentication
   - Subscribes to `esp32/traffic/density_now` topic
   - Receives density updates in JSON format

5. **Scheduler System**
   - Cooperative multitasking scheduler
   - 10ms tick resolution
   - Non-blocking timer implementation

## Hardware Requirements

### Required Components

- **ESP32 Development Board**
  - Recommended: ESP32-S3 (for better AI performance)
  - Alternative: ESP32 with PSRAM support
  - Minimum: ESP32 with at least 4MB Flash

- **Camera Module**
  - OV2640 camera sensor
  - Compatible boards:
    - ESP32-S3-EYE (built-in camera)
    - XIAO ESP32S3 Sense (built-in camera)
    - Custom ESP32 + OV2640 breakout

- **Traffic Light Hardware**
  - 4 GPIO-controlled LEDs (or relays for real traffic lights)
  - Pin assignments (configurable):
    - D3: Direction 1 Red
    - D4: Direction 1 Green/Yellow
    - D5: Direction 2 Red
    - D6: Direction 2 Green/Yellow

- **Power Supply**
  - 5V USB power adapter (recommended for camera operation)
  - Alternative: 5V regulated power supply with sufficient current (≥500mA)

- **Optional Components**
  - Serial-to-USB adapter for debugging
  - Wi-Fi access point for network connectivity

### Pin Configuration

#### ESP32-S3-EYE Configuration
```
Camera Pins (Pre-configured):
- VSYNC: GPIO 6
- HREF: GPIO 7
- PCLK: GPIO 13
- XCLK: GPIO 15
- SIOD: GPIO 4
- SIOC: GPIO 5
- D0-D7: GPIO 8,9,10,11,12,16,17,18

Traffic Light Pins:
- D3: GPIO 6 (Direction 1 Red)
- D4: GPIO 7 (Direction 1 Green/Yellow)
- D5: GPIO 8 (Direction 2 Red)
- D6: GPIO 9 (Direction 2 Green/Yellow)
```

#### XIAO ESP32S3 Sense Configuration
```
Camera Pins (Pre-configured):
- VSYNC: GPIO 38
- HREF: GPIO 47
- PCLK: GPIO 13
- XCLK: GPIO 10
- SIOD: GPIO 40
- SIOC: GPIO 39
- D0-D7: GPIO 15,17,18,16,14,12,11,48
```

## Software Dependencies

### ESP-IDF Requirements

- **ESP-IDF Version**: v5.0 or later
- **Python**: 3.8 or later (for ESP-IDF tools)
- **CMake**: 3.5 or later
- **Git**: For component management

### ESP-IDF Components

The project uses the following ESP-IDF managed components:

- `espressif__esp32-camera`: Camera driver for OV2640
- `espressif__esp-tflite-micro`: TensorFlow Lite Micro for ESP32
- `espressif__esp-nn`: Optimized neural network operations
- `espressif__esp_secure_cert_mgr`: Certificate management (optional)
- `coreMQTT`: MQTT client library
- `backoffAlgorithm`: Network reconnection logic

### Python Dependencies (for testing/debugging)

```bash
pip install pyserial pillow numpy
# Optional: for model inference testing
pip install tflite-runtime
# or
pip install tensorflow
```

### Required Files

Before building, ensure you have:

1. **TensorFlow Lite Model**: [`epoch_100_int8.tflite`](https://drive.google.com/file/d/1zyIHK2BcL_hNi34KyIryjksd4vGLv1GC/view?usp=drive_link)
   - Place in project root directory
   - INT8 quantized format required
   - Model should output YOLO-style detections

2. **AWS IoT Certificates**: Place in `main/certs/`
   - `root_cert_auth.crt`: AWS IoT Root CA certificate (required)
   - `client.crt`: Device certificate (required if not using secure element)
   - `client.key`: Device private key (required if not using secure element)

3. **Wi-Fi Configuration**: Set via menuconfig or `idf_component.yml`

## Project Structure

```
traffic_light/
├── CMakeLists.txt              # Main CMake build configuration
├── partitions.csv              # Partition table definition
├── sdkconfig.defaults          # ESP-IDF default configuration
├── epoch_100_int8.tflite       # TensorFlow Lite model (required)
│
├── main/                       # Main application code
│   ├── CMakeLists.txt
│   ├── app_main.c              # ESP-IDF application entry point
│   ├── demo_config.h           # MQTT/AWS IoT configuration
│   ├── motorbike_detector.h    # AI detection module header
│   ├── motorbike_detector.cpp  # AI detection implementation
│   ├── mqtt_demo_mutual_auth.c # MQTT client implementation
│   ├── certs/                  # AWS IoT certificates
│   │   ├── root_cert_auth.crt
│   │   ├── client.crt (optional)
│   │   └── client.key (optional)
│   └── idf_component.yml       # Component dependencies
│
├── traffic_light/              # Traffic light control module
│   ├── traffic_light.h         # Traffic light state machine header
│   ├── traffic_light.cpp       # Traffic light control logic
│   ├── connect.h               # AWS IoT connection header
│   ├── connect.cpp             # AWS IoT MQTT implementation
│   ├── scheduler.h             # Task scheduler header
│   ├── scheduler.c             # Cooperative scheduler implementation
│   ├── software_time.h         # Software timer header
│   ├── software_time.c         # Timer implementation
│   ├── secrects.h              # Wi-Fi and AWS credentials (create this)
│   └── project_copy_20251218113714.ino  # Legacy Arduino sketch
│
├── libraries/                  # Third-party libraries
│   ├── coreMQTT/               # MQTT client library
│   ├── backoffAlgorithm/       # Reconnection algorithm
│   └── common/                 # Common utilities
│
├── managed_components/         # ESP-IDF managed components
│   ├── espressif__esp32-camera/
│   ├── espressif__esp-tflite-micro/
│   ├── espressif__esp-nn/
│   └── espressif__esp_secure_cert_mgr/
│
└── receive_images_and_infer.py # Python tool for image debugging
```

## Installation & Setup

### 1. Install ESP-IDF

Follow the [official ESP-IDF installation guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/).

```bash
# On Linux/Mac
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s3  # or esp32 for standard ESP32
. ./export.sh

# On Windows
# Use ESP-IDF Installer from Espressif website
```

### 2. Clone the Project

```bash
git clone <repository-url>
cd traffic_light
```

### 3. Install Dependencies

The project uses ESP-IDF component manager. Dependencies are automatically downloaded during build.

### 4. Configure AWS IoT Certificates

```bash
# Copy your AWS IoT certificates to main/certs/
cp your-root-ca.pem main/certs/root_cert_auth.crt
cp your-device-certificate.pem main/certs/client.crt
cp your-device-private-key.pem main/certs/client.key
```

### 5. Configure Wi-Fi and AWS IoT

Create `traffic_light/secrects.h`:

```c
#ifndef SECRETS_H
#define SECRETS_H

// Wi-Fi Configuration
#define WIFI_SSID "your-wifi-ssid"
#define WIFI_PASSWORD "your-wifi-password"

// AWS IoT Configuration
#define AWS_IOT_ENDPOINT "your-endpoint.iot.region.amazonaws.com"
#define THINGNAME "your-thing-name"

// AWS Certificates (embedded in binary)
extern const char AWS_CERT_CA[];
extern const char AWS_CERT_CRT[];
extern const char AWS_CERT_PRIVATE[];

#endif
```

Or configure via menuconfig:

```bash
idf.py menuconfig
# Navigate to:
# - Example Configuration → Wi-Fi SSID
# - Example Configuration → Wi-Fi Password
# - Component config → MQTT → Broker URL
```

### 6. Configure Project

```bash
idf.py menuconfig
```

Key configurations:

- **Component config → ESP32S3-Specific → Camera pin assignments**
- **Component config → Motorbike Detector → Enable detector**
- **Component config → Motorbike Detector → ROI percentages**
- **Component config → Motorbike Detector → Score threshold**
- **Component config → MQTT → Broker endpoint and port**

### 7. Build the Project

```bash
idf.py build
```

### 8. Flash and Monitor

```bash
# Flash firmware
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor

# Or do both
idf.py -p /dev/ttyUSB0 flash monitor
```

**Windows**: Use `COM3` or `COM4` instead of `/dev/ttyUSB0`

## Configuration

### Traffic Light Configuration

Edit `traffic_light/traffic_light.cpp` to adjust:

- **GPIO Pins**: Modify `D3`, `D4`, `D5`, `D6` definitions
- **Yellow Light Duration**: Change `YELLOW_TIME` constant (in 10ms ticks, default: 200 = 2 seconds)
- **Default Densities**: Modify `density_dir1` and `density_dir2` initial values

### Green Time Calculation

The system uses a traffic flow model to compute green light duration:

```cpp
int compute_green_seconds(int traffic_density) {
  // Density-based speed and saturation flow
  if (density <= 50) {
    avg_speed = 40 km/h
    sat_flow = 1800 vehicles/hour
    cycle_base = 90 seconds
  } else if (density <= 100) {
    avg_speed = 20 km/h
    sat_flow = 1500 vehicles/hour
    cycle_base = 120 seconds
  }
  
  // Formula: green = (demand_flow * cycle_base) / (2 * sat_flow)
  return computed_green_time;
}
```

Adjust the algorithm in `traffic_light/traffic_light.cpp` as needed.

### AI Detection Configuration

Configure via `idf.py menuconfig`:

- **MBDET_ENABLE**: Enable/disable detector (default: enabled)
- **MBDET_BURST_N**: Number of frames for burst detection (default: 3)
- **MBDET_ROI_X1_PCT**: ROI left boundary percentage (default: 5%)
- **MBDET_ROI_X2_PCT**: ROI right boundary percentage (default: 95%)
- **MBDET_ROI_Y1_PCT**: ROI top boundary percentage (default: 45%)
- **MBDET_ROI_Y2_PCT**: ROI bottom boundary percentage (default: 95%)
- **MBDET_SCORE_THR_PCT**: Detection confidence threshold (default: 65%)
- **MBDET_TOPK**: Maximum detections to consider (default: 10)
- **MBDET_TENSOR_ARENA_BYTES**: Memory allocation for TFLM (default: 200KB)

### MQTT Configuration

**Topic**: `esp32/traffic/density_now`

**Message Format** (JSON):

```json
// Single density for both directions
{
  "density_now": 80  // 0-100 percentage
}

// Or separate densities
{
  "density_now_dir1": 60,  // Direction 1: 0-100%
  "density_now_dir2": 80   // Direction 2: 0-100%
}

// Optional device identifier
{
  "deviceId": "sensor-001",
  "density_now": 75
}
```

**Density Conversion**:
- Percentage (0-100) is converted to vehicle count
- Maximum capacity: 219 vehicles (configurable)
- Formula: `vehicles = (percentage * MAX_VEHICLES) / 100`

## Usage

### Basic Operation

1. **Power on** the ESP32 device
2. **Wi-Fi Connection**: Device connects to configured Wi-Fi network
3. **AWS IoT Connection**: Establishes MQTT connection with mutual TLS
4. **Camera Initialization**: Camera module initializes (if enabled)
5. **Traffic Light Start**: State machine begins operation

### Monitoring Serial Output

```bash
idf.py monitor
```

Expected log output:
```
[MBDET] Camera initialized (RGB565 320x240)
[MBDET] TFLM ready. Input: 192x192x3 int8
[MBDET] Inference time: 245 ms (4.1 fps)
[MBDET] frame: boxes=3 hfill=0.125
[MBDET] density_now=0.125 (12.5%)
[CONNECT] ESP32 - AWS IoT Connected!
[CONNECT] === MQTT MESSAGE ===
[CONNECT] Topic: esp32/traffic/density_now
[CONNECT] Parsed density_now = 80
[CONNECT] Computed GREEN1 seconds = 25
```

### Sending Density Updates via MQTT

Using AWS IoT Console or MQTT client:

```bash
# Publish to topic: esp32/traffic/density_now
mosquitto_pub -h your-endpoint.iot.region.amazonaws.com \
  -p 8883 \
  --cafile root-ca.pem \
  --cert device-cert.pem \
  --key device-key.pem \
  -t esp32/traffic/density_now \
  -m '{"density_now": 75}'
```

### Python Image Receiver Tool

For debugging camera and detection:

```bash
python3 receive_images_and_infer.py \
  --port /dev/ttyUSB0 \
  --baud 115200 \
  --output-dir captured_images \
  --model epoch_100_int8.tflite \
  --roi-x1 5 --roi-x2 95 \
  --roi-y1 45 --roi-y2 95 \
  --score-threshold 0.65
```

This tool:
- Receives base64-encoded JPEG images over serial
- Runs local inference for comparison
- Visualizes ROI and density regions
- Saves annotated images

## Communication Protocol

### MQTT Message Format

**Topic**: `esp32/traffic/density_now`

**Payload (JSON)**:

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `density_now` | number | 0-100 | Traffic density percentage (applies to both directions) |
| `density_now_dir1` | number | 0-100 | Traffic density for direction 1 |
| `density_now_dir2` | number | 0-100 | Traffic density for direction 2 |
| `deviceId` | string | - | Optional sender identifier |

**Examples**:

```json
{"density_now": 50}
{"density_now": 0.5}  // Decimal format (0.0-1.0) also supported
{"density_now_dir1": 60, "density_now_dir2": 80}
{"deviceId": "sensor-001", "density_now": 75}
```

**Minimum Threshold**: Messages with density < 20% are ignored (configurable).

### Serial Output Format

**Image Transmission**:
```
IMG_B64_START
<base64-encoded-jpeg-data>
IMG_B64_END
```

**Log Messages**:
- `[MBDET]`: Motorbike detector logs
- `[CONNECT]`: MQTT connection logs
- `[TRAFFIC]`: Traffic light state logs

## Traffic Light State Machine

The system operates a 4-state finite state machine:

```
STATE1 (GREEN2, RED1)
  ↓ [dynamic timer based on density_dir2]
STATE2 (YELLOW2, RED1)
  ↓ [2 seconds]
STATE3 (GREEN1, RED2)
  ↓ [dynamic timer based on density_dir1]
STATE4 (YELLOW1, RED2)
  ↓ [2 seconds]
[loop back to STATE1]
```

**State Transitions**:
- **STATE1 → STATE2**: When GREEN2 timer expires
- **STATE2 → STATE3**: After 2-second yellow period
- **STATE3 → STATE4**: When GREEN1 timer expires
- **STATE4 → STATE1**: After 2-second yellow period

**Dynamic Timing**:
- Green light duration computed from traffic density
- Minimum: 1 second
- Maximum: Clamped to prevent excessive wait times
- Yellow light: Fixed 2 seconds

## Troubleshooting

### Camera Not Initializing

**Symptoms**: `esp_camera_init failed` errors

**Solutions**:
1. Check camera wiring and pin connections
2. Verify camera power supply (needs stable 5V)
3. Ensure correct camera preset in menuconfig
4. Check PSRAM availability: `idf.py menuconfig` → Enable PSRAM

### MQTT Connection Failures

**Symptoms**: `client.connect()` timeout

**Solutions**:
1. Verify Wi-Fi credentials in `secrects.h`
2. Check AWS IoT endpoint URL
3. Verify certificate files are correct (PEM format)
4. Check AWS IoT policy allows MQTT connection
5. Verify device certificate is activated in AWS IoT Console
6. Check DNS resolution: `ping your-endpoint.iot.region.amazonaws.com`

### TensorFlow Lite Model Issues

**Symptoms**: `AllocateTensors failed` or missing operator errors

**Solutions**:
1. Verify model file `epoch_100_int8.tflite` exists in project root
2. Check model is INT8 quantized
3. Increase `MBDET_TENSOR_ARENA_BYTES` in menuconfig
4. Verify model output format matches expected YOLO-style detections

### Low Detection Accuracy

**Solutions**:
1. Adjust ROI percentages to focus on road area
2. Tune score threshold (lower = more detections, higher = fewer false positives)
3. Ensure good lighting conditions
4. Verify camera focus and positioning
5. Increase burst frame count for more stable readings

### Traffic Light Not Responding

**Solutions**:
1. Verify GPIO pin assignments match hardware
2. Check LED/relay connections
3. Monitor serial output for state machine logs
4. Verify timer system is running (check `SCH_Update()` calls)

### Out of Memory Errors

**Symptoms**: `malloc failed` or `heap_caps_malloc failed`

**Solutions**:
1. Reduce `MBDET_TENSOR_ARENA_BYTES`
2. Reduce frame buffer count: `config.fb_count = 1`
3. Use PSRAM for tensor arena (should be automatic)
4. Reduce burst frame count (`MBDET_BURST_N`)

### Build Errors

**Common Issues**:

1. **Missing model file**:
   ```
   Missing model file: epoch_100_int8.tflite
   ```
   Solution: Place model file in project root directory

2. **Missing certificates**:
   ```
   Missing root CA certificate file
   ```
   Solution: Copy AWS IoT certificates to `main/certs/`

3. **Component not found**:
   ```
   No such file or directory: .../coreMQTT
   ```
   Solution: Run `idf.py reconfigure` to download managed components

## Contributing

Contributions are welcome! Please follow these guidelines:

1. **Code Style**: Follow ESP-IDF coding conventions
2. **Documentation**: Update README and code comments
3. **Testing**: Test changes on hardware before submitting
4. **Pull Requests**: Include description of changes and test results

### Development Workflow

1. Fork the repository
2. Create a feature branch: `git checkout -b feature-name`
3. Make changes and test
4. Commit with descriptive messages
5. Push and create pull request

## License

[Specify your license here - e.g., MIT, Apache 2.0, etc.]

## Acknowledgments

- Espressif Systems for ESP-IDF framework
- TensorFlow team for TensorFlow Lite Micro
- AWS for IoT Core infrastructure
- Contributors and testers

## Additional Resources

- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [TensorFlow Lite Micro Documentation](https://www.tensorflow.org/lite/microcontrollers)
- [AWS IoT Core Developer Guide](https://docs.aws.amazon.com/iot/latest/developerguide/)
- [ESP32-Camera GitHub](https://github.com/espressif/esp32-camera)

---

**Last Updated**: January/2026

For questions or issues, please open an issue on the project repository or mail directly us 
10422113@student.vgu.edu.vn or 10422105@student.vgu.edu.vn
### Compsentation
Thanks Dr Vo Bich Hien a lecturer at Vietnamese German University for guiding us during this projects

