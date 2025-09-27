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

/* =========================================================================
 *  Parser for TLV type = Point Cloud Ext (commonly 301 or 302 depending on build)
 *  - Expects tlvBegin to point at the start of the TLV header (MmwDemo_output_message_tl)
 *  - Uses SDK struct names for unit and point payload
 *  - Supports both 10-byte (uint8 snr/noise) and 12-byte (int16 snr/noise) variants
 * ========================================================================= */
static bool parsePointCloudExtTLV(const uint8_t* tlvBegin,
                                  const uint8_t* frameEnd,
                                  /* out */ PointF* outPoints,
                                  int maxOut,
                                  /* print first N points to Serial to debug */ int printFirstN,
                                  /* out */ int &outCount) {
  outCount = 0;

  uint32_t elapsed_ms = doppler.getInterval(&doppler.mmwave.lastTick);

  /* Bounds check for TLV header */
  if (tlvBegin + sizeof(MmwDemo_output_message_tl) > frameEnd) {
    Serial.println(F("[PCEXT] Buffer smaller than TLV header."));
    return false;
  }

  /* Read TLV header (type/length) */
  const MmwDemo_output_message_tl* tl = reinterpret_cast<const MmwDemo_output_message_tl*>(tlvBegin);
  const uint8_t* payload    = tlvBegin + sizeof(MmwDemo_output_message_tl);
  const uint8_t* payloadEnd = payload + tl->length;

  if (payloadEnd > frameEnd) {
    Serial.println(F("[PCEXT] Payload exceeds frame bounds."));
    return false;
  }

  /* Must have at least the unit block */
  if (payload + sizeof(MmwDemo_output_message_point_uint) > payloadEnd) {
    Serial.println(F("[PCEXT] Payload too short for unit block."));
    return false;
  }

  /* Read units */
  const MmwDemo_output_message_point_uint* pUnit =
    reinterpret_cast<const MmwDemo_output_message_point_uint*>(payload);

  const float xyzUnit     = pUnit->xyzUnit;
  const float dopplerUnit = pUnit->dopplerUnit;
  const float snrUnit     = pUnit->snrUnit;
  const float noiseUnit   = pUnit->noiseUnit;

  const uint8_t* p = payload + sizeof(MmwDemo_output_message_point_uint);
  const int bytesLeft = (int)(payloadEnd - p);
  if (bytesLeft < 0) {
    Serial.println(F("[PCEXT] Negative bytesLeft after unit block."));
    return false;
  }

  /* Try to resolve which point format we have:
      - 10-byte points (MmwDemo_output_message_UARTpoint)  => (bytesLeft % 10 == 0)
      - 12-byte points (MmwDemo_output_message_UARTpoint_int16) => (bytesLeft % 12 == 0)
      Prefer exact divisibility; if both match (rare), prefer 10-byte to match SDK typedef you provided. */
  int pointSize = 0;
  bool useInt16SnrNoise = false;

  const bool fits10 = (bytesLeft % (int)sizeof(MmwDemo_output_message_UARTpoint)) == 0;
  const bool fits12 = (bytesLeft % (int)sizeof(MmwDemo_output_message_UARTpoint_int16)) == 0;

  if (fits10 && !fits12) {
    pointSize = sizeof(MmwDemo_output_message_UARTpoint);
    useInt16SnrNoise = false;
  } else if (!fits10 && fits12) {
    pointSize = sizeof(MmwDemo_output_message_UARTpoint_int16);
    useInt16SnrNoise = true;
  } else if (fits10 && fits12) {
    /* Ambiguous but very unlikely; default to classic SDK 10-byte layout */
    pointSize = sizeof(MmwDemo_output_message_UARTpoint);
    useInt16SnrNoise = false;
  } else {
    // Serial.printf("[PCEXT] Payload size %d does not match 10B or 12B point formats.\r\n", bytesLeft);
    return false;
  }

  const int numPoints = bytesLeft / pointSize;
  outCount = numPoints;

  // Serial.printf("[PCEXT] TLV type=%u, payload=%u bytes, points=%d, format=%s\r\n",
  //               (unsigned)tl->type, (unsigned)tl->length, numPoints,
  //               useInt16SnrNoise ? "4h2h (12B)" : "4h2B (10B)");

  String customer_message = "@ " + String(elapsed_ms) + " ";

  /* Decompress points */
  int stored = 0;
  for (int i = 0; i < numPoints; ++i) {
    if (p + pointSize > payloadEnd) {
      // Serial.println(F("[PCEXT] Truncated point (bounds cut)."));
      break;
    }

    PointF pf {};
    if (!useInt16SnrNoise) {
      /* 10-byte point (4h2B) */
      const MmwDemo_output_message_UARTpoint* pt =
          reinterpret_cast<const MmwDemo_output_message_UARTpoint*>(p);
      pf.x      = pt->x * xyzUnit;
      pf.y      = pt->y * xyzUnit;
      pf.z      = pt->z * xyzUnit;
      pf.doppler= pt->doppler * dopplerUnit;
      pf.snr    = ((float)pt->snr)   * snrUnit;
      pf.noise  = ((float)pt->noise) * noiseUnit;

      customer_message += String((int) (pf.x * 1000)) + " ";
      customer_message += String((int) (pf.y * 1000)) + " ";
      customer_message += String((int) (pf.z * 1000)) + " ";
      customer_message += String((int) (pf.doppler * 1000)) + " ";
      customer_message += String((int) (pf.snr)) + " ";

    } else {
      /* 12-byte point (4h2h) */
      const MmwDemo_output_message_UARTpoint_int16* pt16 =
          reinterpret_cast<const MmwDemo_output_message_UARTpoint_int16*>(p);
      pf.x      = pt16->x * xyzUnit;
      pf.y      = pt16->y * xyzUnit;
      pf.z      = pt16->z * xyzUnit;
      pf.doppler= pt16->doppler * dopplerUnit;
      pf.snr    = ((float)pt16->snr)   * snrUnit;
      pf.noise  = ((float)pt16->noise) * noiseUnit;

      customer_message += String((int) (pf.x * 1000)) + " ";
      customer_message += String((int) (pf.y * 1000)) + " ";
      customer_message += String((int) (pf.z * 1000)) + " ";
      customer_message += String((int) (pf.doppler * 1000)) + " ";
      customer_message += String((int) (pf.snr)) + " ";
    }

    if (outPoints && stored < maxOut) {
      outPoints[stored] = pf;
      ++stored;
    }

    p += pointSize;
  }

  Serial.printf("%s\r\n", customer_message.c_str());

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

void printTLVs(uint8_t *buffer) {
  int outCount;

  // Interpret header from buffer
  MmwDemo_output_message_header_t *header = reinterpret_cast<MmwDemo_output_message_header_t*>(buffer);

  // TLVs start immediately after header
  uint8_t *tlvPtr = buffer + sizeof(MmwDemo_output_message_header_t);
  uint8_t *endPtr = buffer + header->totalPacketLen;

  for (uint32_t i = 0; i < header->numTLVs; ++i) {
    // Ensure at least 8 bytes are available for TLV header
    if (tlvPtr + 8 > endPtr) {
      Serial.printf("Error: TLV #%u header out of bounds (offset %ld)\r\n", i + 1, tlvPtr - buffer);
      return;
    }

    // Read TLV header fields safely
    uint32_t tlvType, tlvLength;
    memcpy(&tlvType, tlvPtr, sizeof(uint32_t));
    memcpy(&tlvLength, tlvPtr + 4, sizeof(uint32_t));

    // Serial.printf("TLV #%u\r\n", i + 1);
    // Serial.printf("  Offset : %ld\r\n", tlvPtr - buffer);
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

    if (tlvType==301) {
      parsePointCloudExtTLV(tlvPtr, endPtr, nullptr, 0, 20, outCount);
    }

    // Bounds check for the full TLV (header + payload)
    if (tlvPtr + tlvLength > endPtr || tlvLength < 8) {
      Serial.printf("  Error: TLV data exceeds packet length or TLV length too small!\r\n");
      return;
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
      mmwave.lastSendTime = millis();
      mmwave.rx.ctr = 0;
      mmwave.state = MMWAVE_WAIT_ECHO;
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
            if (!isHeaderValid(&this->mmwave.rx.message.header)) {
              // Serial.println("Invalid header received!");
              this->mmwave.rx.ctr = 0; // Reset counter
            } else {
              // printHeader(&this->mmwave.rx.message.header);
            }
          }
        // Else receive the TLV data
        } else if (this->mmwave.rx.ctr < this->mmwave.rx.message.header.totalPacketLen) {
          this->mmwave.rx.message.buffer[this->mmwave.rx.ctr++] = c;
          // If message is complete...
          if (this->mmwave.rx.ctr == this->mmwave.rx.message.header.totalPacketLen) {
            
            printTLVs(this->mmwave.rx.message.buffer);

            this->mmwave.rx.ctr = 0;
          }
        }
      }
      break;
  }
}
