# ESP32-S3 mmWave Gate Sensor using V-LD3

<p float="left">
  <img src="./doc/esp32s3_board.png" width="45%" style="margin-right: 5%;" />
  <img src="./doc/V-LD3-EVAL-ES.png" width="30%" />
</p>

💡 **Observations:**  

- mmWave configuration (.cfg) inside **mmWaveConfig.h** file;
- Sensor baudrate is not changed by ESP32S3 (baudrate command inside .cfg is ignored);
- If using V-LD3 eval kit, then R220 needs to be removed, or replaced by a larger value. This is to avoid bus conflict between FTDI TX line and ESP32 TX line.
- Gate application

<img src="./doc/3d-radar-gate.png" width="100%" />
