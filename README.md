# ESP32-S3 mmWave Gate Sensor using V-LD3

<p float="left">
  <img src="./doc/esp32s3_board.png" width="45%" style="margin-right: 5%;" />
  <img src="./doc/V-LD3-EVAL-ES.png" width="30%" />
</p>

💡 **Observations:**  

- mmWave configuration (.cfg) inside **mmWaveConfig.h** file;
- If using V-LD3 eval kit, then R220 needs to be removed. This is to avoid bus conflict between FTDI TX line and ESP32 TX line;
- Use the same TX and RX pins on the RMV board that were previously used for the K-LD7.
- Gate application

<img src="./doc/3d-radar-gate.png" width="100%" />

## 🖥️  Viewer Communication

- Baud Rate: 921600 bps
- Data Format: 8 data bits, no parity, 1 stop bit (8N1)
- Port: USB (mapped as a virtual COM port on the host system)

## 📤 JSON Output Format — Gate Application

This document describes the structure of the JSON frames sent by the radar gate system running on ESP32S3.
Each JSON message is serialized in a single line and terminated with a carriage return and newline (\r\n).
This makes it easier to parse over serial/UART or network streams where line-based processing is expected.

Example output (actual data in one line):

```json
{"frame":{"dt":100,"n":1167},"zones":{"z1":0,"z2":0,"z3":0},"targets":[{"id":2,"x":-0.519,"y":0.734,"z":0,"vx":-1.271,"vy":-1.143,"vz":0.001,"ax":0.062,"ay":-0.023,"az":0,"cf":0.989,"gf":3}]}\r\n
```
⚠️ No indentation or pretty-printing is applied. This keeps the message compact and suitable for streaming environments.

Each frame corresponds to a parsed radar data packet and includes:

- Timestamp and frame number
- Detection zone states
- Tracked targets (with position, velocity, acceleration, classification)
- *(Optional)* Detected point cloud (to be added)
- The json is transmitted as a single line ended with "\r\n"

---

## 🧪 Example JSON

```json
{
  "frame": {
    "dt": 100,
    "n": 1167
  },
  "zones": {
    "z1": 0,
    "z2": 0,
    "z3": 0
  },
  "targets": [
    {
      "id": 2,
      "x": -0.519,
      "y": 0.734,
      "z": 0,
      "vx": -1.271,
      "vy": -1.143,
      "vz": 0.001,
      "ax": 0.062,
      "ay": -0.023,
      "az": 0,
      "cf": 0.989,
      "gf": 3
    }
  ]
}
```

### 📦 Top-Level Fields

| Field                 | Type   | Description                            |
| --------------------- | ------ | -------------------------------------- |
| `frame`               | object | Metadata for the current radar frame   |
| `zones`               | object | State of the 3 detection zones         |
| `targets`             | array  | List of tracked targets (can be empty) |
| `points` *(optional)* | array  | Raw detection points if enabled        |

### 🔹 frame Object

| Field | Type | Description                                |
| ----- | ---- | ------------------------------------------ |
| `dt`  | int  | Time delta from last frame in milliseconds |
| `n`   | int  | Frame number (monotonically increasing)    |

### 🔹 zones Object

| Field | Type | Description                           |
| ----- | ---- | ------------------------------------- |
| `z1`  | int  | 0 = zone inactive, 1 = zone detection |
| `z2`  | int  | 0 = zone inactive, 1 = zone detection |
| `z3`  | int  | 0 = zone inactive, 1 = zone detection |

### 🔹 targets Array

Each element corresponds to a detected and tracked object.
Data extracted from TLV 308 (Target List), part of the radar’s tracking engine.

| Field | Type  | Description                                              |
| ----- | ----- | -------------------------------------------------------- |
| `id`  | int   | Target ID (stable while target is tracked)               |
| `x`   | float | Position X [meters]                                      |
| `y`   | float | Position Y [meters]                                      |
| `z`   | float | Position Z [meters]                                      |
| `vx`  | float | Velocity X [m/s]                                         |
| `vy`  | float | Velocity Y [m/s]                                         |
| `vz`  | float | Velocity Z [m/s]                                         |
| `ax`  | float | Acceleration X [m/s²]                                    |
| `ay`  | float | Acceleration Y [m/s²]                                    |
| `az`  | float | Acceleration Z [m/s²]                                    |
| `cf`  | float | Classification confidence (0.0 to 1.0)                   |
| `gf`  | int   | Group or classification ID (e.g., person, vehicle, etc.) |

- Fields are truncated to 3 decimal places.
- Units are metric.
- If no targets are detected in a frame, then "targets" is not included in the json message.


### 📄 Changelog

- **v1.0**: Initial JSON schema for targets and zones
