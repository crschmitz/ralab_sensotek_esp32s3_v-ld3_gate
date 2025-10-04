#include <ArduinoJson.h>
#include "Doppler.h"
#include "Config.h"
#include "Usart.h"
#include "Timer.h"
#include "defines.h"
#include "esp_task_wdt.h"
#include "mmWaveConfig.h"

extern Config config;
extern Usart usart;
extern Doppler doppler;
extern Timer timer;

unsigned long startingTime;

const uint8_t MAGIC_WORD[8] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};

StaticJsonDocument<2048> doc;

/* =========================================================================
 *  Binary layout structs (packed)
 * ========================================================================= */
#pragma pack(push, 1)

/* TLV header (SDK naming) */
typedef struct MmwDemo_output_message_tl_t {
  uint32_t type;   /* TLV type */
  uint32_t length; /* Length in bytes (payload only, not including this header) */
} MmwDemo_output_message_tl;

/* Unit block for Point-Cloud-Ext TLV (SDK naming) */
typedef struct MmwDemo_output_message_point_uint_t {
  float    xyzUnit;     /* x/y/z coordinates reporting unit, in m */
  float    dopplerUnit; /* radial velocity reporting unit, in m/s */
  float    snrUnit;     /* SNR reporting unit, in dB */
  float    noiseUnit;   /* Noise reporting unit, in dB */
  uint16_t numDetectedPoints[2]; /* [0]=major motion points, [1]=minor motion points */
} MmwDemo_output_message_point_uint;

/* Point structure (SDK naming) — classic 10-byte format (4h2B) */
typedef struct MmwDemo_output_message_UARTpoint_t {
  int16_t x;       /* Detected point x, 1 LSB = xyzUnit */
  int16_t y;       /* Detected point y, 1 LSB = xyzUnit */
  int16_t z;       /* Detected point z, 1 LSB = xyzUnit */
  int16_t doppler; /* Detected point radial velocity, 1 LSB = dopplerUnit */
  uint8_t snr;     /* Range detection SNR, 1 LSB = snrUnit */
  uint8_t noise;   /* Detected point noise value, 1 LSB = noiseUnit */
} MmwDemo_output_message_UARTpoint;

/* OPTIONAL: alternative 12-byte format sometimes used in newer firmwares (4h2h) */
typedef struct MmwDemo_output_message_UARTpoint_int16_t {
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t doppler;
  int16_t snr;     /* 1 LSB = snrUnit */
  int16_t noise;   /* 1 LSB = noiseUnit */
} MmwDemo_output_message_UARTpoint_int16;

struct TargetF {
  uint32_t id;
  float x, y, z;       // meters
  float vx, vy, vz;    // m/s
  float ax, ay, az;    // m/s^2
  float g;             // gating gain
  float confidence;    // 0..1 or 0..100 (we normalize to 0..1)
  float ec[16];        // covariance matrix (optional use)
};

#pragma pack(pop)

/* =========================================================================
 *  Safe little-endian readers (avoid alignment issues)
 * ========================================================================= */
static inline uint32_t rdU32(const uint8_t* p) { uint32_t v; memcpy(&v, p, 4); return v; }
static inline float    rdF32(const uint8_t* p) { float    v; memcpy(&v, p, 4); return v; }
static inline int16_t  rdI16(const uint8_t* p) { int16_t  v; memcpy(&v, p, 2); return v; }

/* =========================================================================
 *  Decompressed point (for app-level use)
 * ========================================================================= */
struct PointF {
    float x, y, z;     // meters
    float doppler;     // m/s
    float snr;         // in dB or firmware-defined unit
    float noise;       // in dB or firmware-defined unit
};

// Create a HardwareSerial object for UART1 and name it SerialRadar
HardwareSerial SerialRadar(1);

/*******************************************************************************
* Function Name  : Succ
* Description    : Returns the successor of the index of a list
* Input          : i = index ; size = list size
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t Succ(uint16_t i, uint16_t size) {
  if (++i >= size)
    i = 0;
  return i;
}

/*******************************************************************************
* Function Name  : Pred
* Description    : Returns the predecessor of the index of a list
* Input          : i = index ; size = list size
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t Pred(uint16_t i, uint16_t size) {
  if (!i--)
    i = size-1;
  return i;
}

bool isHeaderValid(MmwDemo_output_message_header_t *header) {
  // Check if the magic word matches
  if (memcmp(header->magicWord, MAGIC_WORD, sizeof(MAGIC_WORD)) != 0) {
    // Serial.printf("Invalid magic word...\r\n");
    return false;
  }
  
  // Check if the platform type is valid
  if (header->platform != PLATFORM_TYPE) {
    // Serial.printf("Invalid platform type: %08X\r\n", header->platform);
    return false;
  }

  // Check if packet length is valid
  if (header->totalPacketLen < sizeof(MmwDemo_output_message_header_t) || header->totalPacketLen % 32 != 0) {
    // Serial.printf("Invalid total packet length: %u\r\n", header->totalPacketLen);
    return false;
  }

  // Check if packet length is valid
  if (header->totalPacketLen >= RX_SIZE) {
    // Serial.printf("Message exceeds size: %u\r\n", header->totalPacketLen);
    return false;
  }

  return true;
}

// Printing function
void printHeader(MmwDemo_output_message_header_t *header) {
  Serial.printf("magicWord   : ");
  for (int i = 0; i < 4; ++i)
    Serial.printf("%04X", header->magicWord[i]);
  Serial.println();
  Serial.printf("version     : 0x%08X\r\n", header->version);
  Serial.printf("totalPktLen : %u\r\n", header->totalPacketLen);
  Serial.printf("platform    : 0x%08X\r\n", header->platform);
  Serial.printf("frameNumber : %u\r\n", header->frameNumber);
  Serial.printf("tCpuCycles  : %u\r\n", header->timeCpuCycles);
  Serial.printf("numDetectObj: %u\r\n", header->numDetectedObj);
  Serial.printf("numTLVs     : %u\r\n", header->numTLVs);
  Serial.printf("subFrameNum.: 0x%08X\r\n", header->subFrameNumber);
  Serial.printf("========================================\r\n");
}

static bool parsePointCloudExtTLV(const uint8_t* payload, int length) {
  if (!payload || length < (int)sizeof(MmwDemo_output_message_point_uint))
    return false;

  const auto* unit = reinterpret_cast<const MmwDemo_output_message_point_uint*>(payload);
  const float xyzUnit     = unit->xyzUnit;
  const float dopplerUnit = unit->dopplerUnit;
  const float snrUnit     = unit->snrUnit;
  const float noiseUnit   = unit->noiseUnit;

  payload += sizeof(MmwDemo_output_message_point_uint);
  length  -= sizeof(MmwDemo_output_message_point_uint);
  if (length <= 0) return false;

  const bool is10B = (length % sizeof(MmwDemo_output_message_UARTpoint)) == 0;
  const bool is12B = (length % sizeof(MmwDemo_output_message_UARTpoint_int16)) == 0;
  const int ptSize = is10B ? sizeof(MmwDemo_output_message_UARTpoint)
                  : is12B ? sizeof(MmwDemo_output_message_UARTpoint_int16)
                          : 0;
  if (ptSize == 0) return false;

  const int count = length / ptSize;

  for (int i = 0; i < count; ++i) {
    float x, y, z, v, snr;
    if (is10B) {
      auto* p = reinterpret_cast<const MmwDemo_output_message_UARTpoint*>(payload + ptSize * i);
      x = p->x * xyzUnit;
      y = p->y * xyzUnit;
      z = p->z * xyzUnit;
      v = p->doppler * dopplerUnit;
      snr = ((float)p->snr) * snrUnit;
    } else {
      auto* p = reinterpret_cast<const MmwDemo_output_message_UARTpoint_int16*>(payload + ptSize * i);
      x = p->x * xyzUnit;
      y = p->y * xyzUnit;
      z = p->z * xyzUnit;
      v = p->doppler * dopplerUnit;
      snr = ((float)p->snr) * snrUnit;
    }

    // msg += String((int)(x * 1000)) + " ";
    // msg += String((int)(y * 1000)) + " ";
    // msg += String((int)(z * 1000)) + " ";
    // msg += String((int)(v * 1000)) + " ";
    // msg += String((int)(snr)) + " ";
  }

  // Serial.println(msg);
  return true;
}

#pragma pack(push,1)
struct TargetExt308 {
  uint32_t tid;
  float posX, posY, posZ;
  float velX, velY, velZ;
  float accX, accY, accZ;   // not printed
  float ec[16];             // not printed
  float g;                  // not printed
  float confidence;         // not printed
};
#pragma pack(pop)
static_assert(sizeof(TargetExt308) == 112, "TLV308 entry must be 112 bytes");

static bool parseTargetListTLV_308(const uint8_t* tlvPayload, int lenght)
{
  if (!tlvPayload || lenght <= 0) return false;
  const size_t entrySize = sizeof(TargetExt308);
  if ((size_t)lenght % entrySize != 0) return false;

  const uint32_t count = (uint32_t)((size_t)lenght / entrySize);
  const TargetExt308* tgt = reinterpret_cast<const TargetExt308*>(tlvPayload);


  JsonArray targets = doc.createNestedArray("targets");

  for (uint32_t i = 0; i < count; ++i) {
    JsonObject t = targets.createNestedObject();
    t["id"] = tgt[i].tid;
    t["x"]  = ((int)(tgt[i].posX * 1000)) / 1000.0;
    t["y"]  = ((int)(tgt[i].posY * 1000)) / 1000.0;
    t["z"]  = ((int)(tgt[i].posZ * 1000)) / 1000.0;
    t["vx"] = ((int)(tgt[i].velX * 1000)) / 1000.0;
    t["vy"] = ((int)(tgt[i].velY * 1000)) / 1000.0;
    t["vz"] = ((int)(tgt[i].velZ * 1000)) / 1000.0;
    t["ax"] = ((int)(tgt[i].accX * 1000)) / 1000.0;
    t["ay"] = ((int)(tgt[i].accY * 1000)) / 1000.0;
    t["az"] = ((int)(tgt[i].accZ * 1000)) / 1000.0;
    t["cf"] = ((int)(tgt[i].confidence * 1000)) / 1000.0;
    t["gf"] = tgt[i].g;
    
    // Print id, position (m), velocity (m/s)
    // Serial.printf("@TGT id=%u pos=(%.2f, %.2f, %.2f) m vel=(%.2f, %.2f, %.2f) m/s\n",
    //               (unsigned)tgt[i].tid,
    //               tgt[i].posX, tgt[i].posY, tgt[i].posZ,
    //               tgt[i].velX, tgt[i].velY, tgt[i].velZ);
  }
  return true;
}

void createDefaultConfig() {
  File f = FFat.open("/params.cfg", "w");
  if (!f) {
    Serial.println("Failed to create /params.cfg");
    return;
  }
  f.print(defaultParams);
  f.close();
  Serial.println("Default /params.cfg written to FFat");
}

bool parseTLVs(uint8_t *buffer) {
  bool result;
  int i, outCount;

  // Interpret header from buffer
  MmwDemo_output_message_header_t *header = reinterpret_cast<MmwDemo_output_message_header_t*>(buffer);

  // TLVs start immediately after header
  uint8_t *tlvPtr = buffer + sizeof(MmwDemo_output_message_header_t);
  uint8_t *endPtr = buffer + header->totalPacketLen;

  for (i = 0; i < header->numTLVs; ++i) {
    // Ensure at least 8 bytes are available for TLV header
    if (tlvPtr + 8 > endPtr) {
      // Serial.printf("Error: TLV #%u header out of bounds (offset %ld)\r\n", i + 1, tlvPtr - buffer);
      break;
    }

    // Read TLV header fields safely
    uint32_t tlvType, tlvLength;
    memcpy(&tlvType, tlvPtr, sizeof(uint32_t));
    memcpy(&tlvLength, tlvPtr + 4, sizeof(uint32_t));

    // Serial.printf("TLV #%u\r\n", i + 1);
    // // Serial.printf("  Offset : %ld\r\n", tlvPtr - buffer);
    // Serial.printf("  Type   : %u\r\n", tlvType);
    // Serial.printf("  Length : %u bytes\r\n", tlvLength);

    // // Print raw TLV header bytes for debugging
    // Serial.printf("  Raw header: ");
    // for (int k = 0; k < 8; ++k) {
    //   Serial.printf("%02X ", tlvPtr[k]);
    // }
    // Serial.printf("\r\n");

    // Extract payload and print preview
    uint8_t* payload = tlvPtr + 8;
    size_t payloadLength = tlvLength - 8;

    switch(tlvType) {
      // Point Cloud (Detected Points)
      case TLV_POINT_CLOUD:
        parsePointCloudExtTLV(payload, tlvLength);
        break;
      // Target List (Tracks)
      case TLV_TARGET_LIST:
        parseTargetListTLV_308(payload, tlvLength);
        break;
      // Other TLV types can be handled here...
      default:
        break;
    }

    // Bounds check for the full TLV (header + payload)
    if (tlvPtr + tlvLength > endPtr) {
      // Serial.printf("  Error: TLV data exceeds packet length!\r\n");
      break;
    }

    // Serial.printf("  Payload (first 32 bytes or less): ");
    // for (size_t j = 0; j < payloadLength && j < 32; ++j) {
    //   Serial.printf("%02X ", payload[j]);
    // }
    // if (payloadLength > 32) {
    //   Serial.printf("...");
    // }
    // Serial.printf("\r\n");

    // Advance to next TLV
    tlvPtr += tlvLength + 8;
  }
  // Check if all TLVs were processed
  result = i >= header->numTLVs;
  return result;
}

void readAndPrintConfig() {
  File f = FFat.open("/params.cfg", "r");
  if (!f) {
    Serial.println("Failed to open /params.cfg");
    return;
  }

  Serial.println("=== /params.cfg ===");
  while (f.available()) {
    Serial.write(f.read());
  }
  f.close();
}

// Task funcion for communication with doppler sensor
void sendTaskFunction(void *parameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10);

  for(;;) {
    doppler.exec();
    usart.exec();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);      // Delay until next cycle
  }
}

// Constructor
Doppler::Doppler() {
  this->jsonLine = "";
}

void Doppler::start() {
  if (!FFat.exists("/params.cfg")) {
    Serial.println("/params.cfg not found, creating...");
    createDefaultConfig();
  }
  // readAndPrintConfig();

  // Start serial port
  SerialRadar.setRxBufferSize(8192);
  SerialRadar.begin(115200, SERIAL_8N1, RADAR_RX, RADAR_TX);

  // Start the tasks
  this->startTasks();
}

// Function to start the task
void Doppler::startTasks() {
  // Create the task and pass 'this' pointer to the static wrapper
  xTaskCreate(
    sendTaskFunction,       // Task function
    "sendTaskFunction",     // Name of the task
    8192,                   // Stack size (in words)
    this,                   // Task input parameter (this instance)
    1,                      // Priority of the task
    &sendTaskHandle         // Task handle
  );
}

uint32_t Doppler::getInterval(uint64_t *ticks) {
  uint64_t now = timer.getTickCount();
  uint32_t interval = timer.timeElapsed_ms(*ticks, now);
  if (interval > 9999)
    interval = 9999;
  *ticks = now;
  return interval;
}

bool Doppler::process() {
  bool result = false;

  return result;
}

void Doppler::version() {
  SerialRadar.println("version");
}

void Doppler::flush() {
  int c;
  // Reception state machine
  while (SerialRadar.available()) {
    c = SerialRadar.read();
    mmwave.rx.ctr = 0;
  }
}

void Doppler::exec() {
  enum {
    MMWAVE_IDLE = 0,
    MMWAVE_OPEN_FILE,
    MMWAVE_READ_LINE,
    MMWAVE_SEND_LINE,
    MMWAVE_WAIT_ECHO,
    MMWAVE_DONE,
    MMWAVE_TLV,
  };
  bool echoMatched = false;

  switch (mmwave.state) {
    case MMWAVE_IDLE:
      startingTime = millis();
      mmwave.state = MMWAVE_OPEN_FILE;
      break;

    case MMWAVE_OPEN_FILE:
      mmwave.cfgFile = FFat.open("/params.cfg", "r");
      if (!mmwave.cfgFile) {
        Serial.println("Failed to open /params.cfg");
        mmwave.state = MMWAVE_DONE;
        break;
      }
      mmwave.state = MMWAVE_READ_LINE;
      break;

    case MMWAVE_READ_LINE:
      mmwave.currentLine = "";
      while (mmwave.cfgFile.available()) {
        char c = mmwave.cfgFile.read();
        if (c == '\n') break;
        mmwave.currentLine += c;
      }
      mmwave.currentLine.trim();

      if (!mmwave.cfgFile.available() && mmwave.currentLine.isEmpty()) {
        mmwave.cfgFile.close();
        mmwave.state = MMWAVE_DONE;
        unsigned long elapsed = millis() - startingTime;
        Serial.printf("Sensor configuration completed in %lu ms\r\n", elapsed);
        break;
      }

      if (mmwave.currentLine.startsWith("%") || mmwave.currentLine.isEmpty()) {
        mmwave.state = MMWAVE_READ_LINE;
      } else {
        mmwave.retries = 0;
        mmwave.state = MMWAVE_SEND_LINE;
      }
      break;

    case MMWAVE_SEND_LINE:
      SerialRadar.println(mmwave.currentLine);
      Serial.println(mmwave.currentLine);

      if (mmwave.currentLine.startsWith("baudRate")) {
        int spaceIndex = mmwave.currentLine.indexOf(' ');
        if (spaceIndex > 0) {
          String baudStr = mmwave.currentLine.substring(spaceIndex + 1); // "1250000"
          long baud = baudStr.toInt();  // convert to integer
          Serial.printf("Changing baud rate to %ld bps\r\n", baud);
          // ✅ Ensure last message really went out
          SerialRadar.flush();
          // Now it's safe to switch          
          SerialRadar.end();
          SerialRadar.begin(baud, SERIAL_8N1, RADAR_RX, RADAR_TX);
          // Don't wait for echo, just proceed to next line
          mmwave.state = MMWAVE_READ_LINE;
        }
      } else {
        mmwave.lastSendTime = millis();
        mmwave.rx.ctr = 0;
        mmwave.state = MMWAVE_WAIT_ECHO;
      }
      break;

    case MMWAVE_WAIT_ECHO:
      while (SerialRadar.available()) {
        char c = SerialRadar.read();
        if (mmwave.rx.ctr < RX_SIZE - 1) {
          mmwave.rx.message.buffer[mmwave.rx.ctr++] = c;
        }
        if (c == '\n') {
          mmwave.rx.message.buffer[mmwave.rx.ctr] = '\0';
          mmwave.rx.ctr = 0;
          String received((char*)mmwave.rx.message.buffer);
          received.trim();

          Serial.println(received);

          if (received.equalsIgnoreCase("Done")) {
            mmwave.state = MMWAVE_READ_LINE;
          }
        }
      }
      // After a timeout either retry or finish
      if (millis() - mmwave.lastSendTime >= 1200) {
        // if (++mmwave.retries < 5) {
          mmwave.state = MMWAVE_SEND_LINE;
        // } else {
        //   mmwave.state = MMWAVE_DONE;
        // }
      }
      break;

    case MMWAVE_DONE:
      while (SerialRadar.available()) {
        char c = SerialRadar.read();

        // Receive magic word...
        if (this->mmwave.rx.ctr < sizeof(MAGIC_WORD)) {
          // Check if the received byte matches the expected magic word
          if (c == MAGIC_WORD[this->mmwave.rx.ctr]) {
            this->mmwave.rx.message.buffer[this->mmwave.rx.ctr++] = c;
          // If magic word mismatch, reset counter
          } else {
            this->mmwave.rx.ctr = 0; // Reset if mismatch
          }
        // Receive the header until it's complete
        } else if (this->mmwave.rx.ctr < sizeof(MmwDemo_output_message_header_t)) {
          this->mmwave.rx.message.buffer[this->mmwave.rx.ctr++] = c;
          // Else header is complete...
          if (this->mmwave.rx.ctr == sizeof(MmwDemo_output_message_header_t)) {
            // If header is not valid, reset counter
            if (isHeaderValid(&this->mmwave.rx.message.header)) {
              doc.clear();

              const uint32_t elapsed = doppler.getInterval(&doppler.mmwave.lastTick);
              JsonObject frame = doc.createNestedObject("frame");
              frame["dt"] = elapsed;
              frame["n"] = this->mmwave.rx.message.header.frameNumber;

              JsonObject zones = doc.createNestedObject("zones");
              zones["z1"] = 0;  // Zone 1 status
              zones["z2"] = 0;  // Zone 2 status
              zones["z3"] = 0;  // Zone 3 status

              // printHeader(&this->mmwave.rx.message.header);
            } else {
              // Serial.println("Invalid header received!");
              this->mmwave.rx.ctr = 0; // Reset counter
            }
          }
        // Else receive the TLV data
        } else if (this->mmwave.rx.ctr < this->mmwave.rx.message.header.totalPacketLen) {
          this->mmwave.rx.message.buffer[this->mmwave.rx.ctr++] = c;
          // If message is complete...
          if (this->mmwave.rx.ctr == this->mmwave.rx.message.header.totalPacketLen) {
            // Parse TLVs
            if (parseTLVs(this->mmwave.rx.message.buffer)) {
              if (this->jsonLine.length() > 0) {
                // Merge jsonLine into doc
                DeserializationError error = deserializeJson(doc, this->jsonLine);
                if (error) {
                  Serial.println("Error parsing input JSON line!");
                }
                this->jsonLine = "";
              }
              // if (usart.getCmd() == "readSensor") {
              //   // Reply sensor frame as JSON
              //   serializeJson(doc, Serial);
              //   Serial.println();
              //   usart.setCmd("");
              // }
            } else {
              // Serial.println("Error parsing TLVs!");
            }
            this->mmwave.rx.ctr = 0;
          }
        }
      }
      break;
  }
}
