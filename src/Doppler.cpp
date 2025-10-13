#include "Doppler.h"
#include "Usart.h"
#include "Timer.h"
#include "defines.h"
#include "esp_task_wdt.h"

extern "C"
{
#include "esp32/rom/crc.h"
}

extern Usart usart;
extern Doppler doppler;
extern Timer timer;
extern String FirmwareVersion;
extern String FirmwareDate;

const uint8_t MAGIC_WORD[8] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};

/* =========================================================================
 *  Binary layout structs (packed)
 * ========================================================================= */
#pragma pack(push, 1)

/* TLV header (SDK naming) */
typedef struct MmwDemo_output_message_tl_t
{
  uint32_t type;   /* TLV type */
  uint32_t length; /* Length in bytes (payload only, not including this header) */
} MmwDemo_output_message_tl;

/* Unit block for Point-Cloud-Ext TLV (SDK naming) */
typedef struct MmwDemo_output_message_point_uint_t
{
  float xyzUnit;                 /* x/y/z coordinates reporting unit, in m */
  float dopplerUnit;             /* radial velocity reporting unit, in m/s */
  float snrUnit;                 /* SNR reporting unit, in dB */
  float noiseUnit;               /* Noise reporting unit, in dB */
  uint16_t numDetectedPoints[2]; /* [0]=major motion points, [1]=minor motion points */
} MmwDemo_output_message_point_uint;

/* Point structure (SDK naming) — classic 10-byte format (4h2B) */
typedef struct MmwDemo_output_message_UARTpoint_t
{
  int16_t x;       /* Detected point x, 1 LSB = xyzUnit */
  int16_t y;       /* Detected point y, 1 LSB = xyzUnit */
  int16_t z;       /* Detected point z, 1 LSB = xyzUnit */
  int16_t doppler; /* Detected point radial velocity, 1 LSB = dopplerUnit */
  uint8_t snr;     /* Range detection SNR, 1 LSB = snrUnit */
  uint8_t noise;   /* Detected point noise value, 1 LSB = noiseUnit */
} MmwDemo_output_message_UARTpoint;

/* OPTIONAL: alternative 12-byte format sometimes used in newer firmwares (4h2h) */
typedef struct MmwDemo_output_message_UARTpoint_int16_t
{
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t doppler;
  int16_t snr;   /* 1 LSB = snrUnit */
  int16_t noise; /* 1 LSB = noiseUnit */
} MmwDemo_output_message_UARTpoint_int16;

#pragma pack(pop)

/* =========================================================================
 *  Safe little-endian readers (avoid alignment issues)
 * ========================================================================= */
static inline uint32_t rdU32(const uint8_t *p)
{
  uint32_t v;
  memcpy(&v, p, 4);
  return v;
}
static inline float rdF32(const uint8_t *p)
{
  float v;
  memcpy(&v, p, 4);
  return v;
}
static inline int16_t rdI16(const uint8_t *p)
{
  int16_t v;
  memcpy(&v, p, 2);
  return v;
}

/* =========================================================================
 *  Decompressed point (for app-level use)
 * ========================================================================= */
struct PointF
{
  float x, y, z; // meters
  float doppler; // m/s
  float snr;     // in dB or firmware-defined unit
  float noise;   // in dB or firmware-defined unit
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
uint16_t Succ(uint16_t i, uint16_t size)
{
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
uint16_t Pred(uint16_t i, uint16_t size)
{
  if (!i--)
    i = size - 1;
  return i;
}

// helper: add only if not already present
inline void addUniqueUseCase(std::vector<UseCase_enum> &v, UseCase_enum uc)
{
  if (std::find(v.begin(), v.end(), uc) == v.end())
  {
    v.push_back(uc);
  }
}

bool isHeaderValid(MmwDemo_output_message_header_t *header)
{
  // Check if the magic word matches
  if (memcmp(header->magicWord, MAGIC_WORD, sizeof(MAGIC_WORD)) != 0)
  {
    // Serial.printf("Invalid magic word...\r\n");
    return false;
  }

  // Check if the platform type is valid
  if (header->platform != PLATFORM_TYPE)
  {
    // Serial.printf("Invalid platform type: %08X\r\n", header->platform);
    return false;
  }

  // Check if packet length is valid
  if (header->totalPacketLen < sizeof(MmwDemo_output_message_header_t) || header->totalPacketLen % 32 != 0)
  {
    // Serial.printf("Invalid total packet length: %u\r\n", header->totalPacketLen);
    return false;
  }

  // Check if packet length is valid
  if (header->totalPacketLen >= RX_SIZE)
  {
    // Serial.printf("Message exceeds size: %u\r\n", header->totalPacketLen);
    return false;
  }

  return true;
}

// Printing function
void printHeader(MmwDemo_output_message_header_t *header)
{
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

#pragma pack(push, 1)
struct TargetExt308
{
  uint32_t tid;
  float posX, posY, posZ;
  float velX, velY, velZ;
  float accX, accY, accZ; // not printed
  float ec[16];           // not printed
  float g;                // not printed
  float confidence;       // not printed
};
#pragma pack(pop)
static_assert(sizeof(TargetExt308) == 112, "TLV308 entry must be 112 bytes");

// Task funcion for communication with doppler sensor
void sendTaskFunction(void *parameter)
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(10);

  for (;;)
  {
    doppler.exec();
    usart.exec();
    vTaskDelayUntil(&xLastWakeTime, xFrequency); // Delay until next cycle
  }
}

// Constructor
Doppler::Doppler()
{
  this->jsonLine = "";
  this->mmwave.cfgString = "";
}

void Doppler::start()
{
  // Start serial port
  SerialRadar.setRxBufferSize(8192);
  SerialRadar.begin(115200, SERIAL_8N1, RADAR_RX, RADAR_TX);

  // Start the tasks
  this->startTasks();
}

// Function to start the task
void Doppler::startTasks()
{
  // Create the task and pass 'this' pointer to the static wrapper
  xTaskCreate(
      sendTaskFunction,   // Task function
      "sendTaskFunction", // Name of the task
      8192,               // Stack size (in words)
      this,               // Task input parameter (this instance)
      1,                  // Priority of the task
      &sendTaskHandle     // Task handle
  );
}

uint32_t Doppler::getInterval(uint64_t *ticks)
{
  uint64_t now = timer.getTickCount();
  uint32_t interval = timer.timeElapsed_ms(*ticks, now);
  if (interval > 99999)
    interval = 99999;
  *ticks = now;
  return interval;
}

bool Doppler::process()
{
  bool result = false;

  return result;
}

void Doppler::version()
{
  SerialRadar.println("version");
}

void Doppler::flush()
{
  int c;
  // Reception state machine
  while (SerialRadar.available())
  {
    c = SerialRadar.read();
    mmwave.rx.ctr = 0;
  }
}

bool Doppler::parsePointCloudExtTLV(const uint8_t *payload, int length)
{
  if (!payload || length < (int)sizeof(MmwDemo_output_message_point_uint))
    return false;

  const auto *unit = reinterpret_cast<const MmwDemo_output_message_point_uint *>(payload);
  const float xyzUnit = unit->xyzUnit;
  const float dopplerUnit = unit->dopplerUnit;
  const float snrUnit = unit->snrUnit;
  const float noiseUnit = unit->noiseUnit;

  payload += sizeof(MmwDemo_output_message_point_uint);
  length -= sizeof(MmwDemo_output_message_point_uint);
  if (length <= 0)
    return false;

  const bool is10B = (length % sizeof(MmwDemo_output_message_UARTpoint)) == 0;
  const bool is12B = (length % sizeof(MmwDemo_output_message_UARTpoint_int16)) == 0;
  const int ptSize = is10B   ? sizeof(MmwDemo_output_message_UARTpoint)
                     : is12B ? sizeof(MmwDemo_output_message_UARTpoint_int16)
                             : 0;
  if (ptSize == 0)
    return false;

  const int count = length / ptSize;

  for (int i = 0; i < count; ++i)
  {
    float x, y, z, v, snr;
    if (is10B)
    {
      auto *p = reinterpret_cast<const MmwDemo_output_message_UARTpoint *>(payload + ptSize * i);
      x = p->x * xyzUnit;
      y = p->y * xyzUnit;
      z = p->z * xyzUnit;
      v = p->doppler * dopplerUnit;
      snr = ((float)p->snr) * snrUnit;
    }
    else
    {
      auto *p = reinterpret_cast<const MmwDemo_output_message_UARTpoint_int16 *>(payload + ptSize * i);
      x = p->x * xyzUnit;
      y = p->y * xyzUnit;
      z = p->z * xyzUnit;
      v = p->doppler * dopplerUnit;
      snr = ((float)p->snr) * snrUnit;
    }

    if (i > 0)
    {
      this->jsonPoints += ","; // add comma between objects
    }
    this->jsonPoints += "{";
    this->jsonPoints += "\"id\":";
    this->jsonPoints += String(i);
    this->jsonPoints += ",\"x\":";
    this->jsonPoints += String(x, 3);
    this->jsonPoints += ",\"y\":";
    this->jsonPoints += String(y, 3);
    this->jsonPoints += ",\"z\":";
    this->jsonPoints += String(z, 3);
    this->jsonPoints += ",\"v\":";
    this->jsonPoints += String(v, 3);
    this->jsonPoints += ",\"snr\":";
    this->jsonPoints += String(snr, 3);
    this->jsonPoints += "}";
  }
  return true;
}

bool Doppler::parseTargetListTLV_308(const uint8_t *tlvPayload, int length)
{
  if (!tlvPayload || length <= 0)
    return false;
  const size_t entrySize = sizeof(TargetExt308);
  if ((size_t)length % entrySize != 0)
    return false;

  const uint32_t count = (uint32_t)((size_t)length / entrySize);
  const TargetExt308 *tgt = reinterpret_cast<const TargetExt308 *>(tlvPayload);

  this->mmwave.persons = count;

  for (uint32_t i = 0; i < count; ++i)
  {

    Target_t t = {
        tgt[i].tid,
        tgt[i].posX, tgt[i].posY, tgt[i].posZ,
        tgt[i].velX, tgt[i].velY, tgt[i].velZ,
        tgt[i].accX, tgt[i].accY, tgt[i].accZ,
        tgt[i].confidence,
        tgt[i].g};
    this->mmwave.targets.push_back(t);

    if (i > 0)
    {
      this->jsonTargets += ","; // add comma between objects
    }
    this->jsonTargets += "{";
    this->jsonTargets += "\"id\":";
    this->jsonTargets += String(tgt[i].tid);

    this->jsonTargets += ",\"x\":";
    this->jsonTargets += String(tgt[i].posX, 3);

    this->jsonTargets += ",\"y\":";
    this->jsonTargets += String(tgt[i].posY, 3);

    this->jsonTargets += ",\"z\":";
    this->jsonTargets += String(tgt[i].posZ, 3);

    this->jsonTargets += ",\"vx\":";
    this->jsonTargets += String(tgt[i].velX, 3);

    this->jsonTargets += ",\"vy\":";
    this->jsonTargets += String(tgt[i].velY, 3);

    this->jsonTargets += ",\"vz\":";
    this->jsonTargets += String(tgt[i].velZ, 3);

    this->jsonTargets += ",\"ax\":";
    this->jsonTargets += String(tgt[i].accX, 3);

    this->jsonTargets += ",\"ay\":";
    this->jsonTargets += String(tgt[i].accY, 3);

    this->jsonTargets += ",\"az\":";
    this->jsonTargets += String(tgt[i].accZ, 3);

    this->jsonTargets += ",\"cf\":";
    this->jsonTargets += String(tgt[i].confidence, 3);

    this->jsonTargets += ",\"gf\":";
    this->jsonTargets += String(tgt[i].g, 3);

    this->jsonTargets += "}";

    // Print id, position (m), velocity (m/s)
    // Serial.printf("@TGT id=%u pos=(%.2f, %.2f, %.2f) m vel=(%.2f, %.2f, %.2f) m/s\n",
    //               (unsigned)tgt[i].tid,
    //               tgt[i].posX, tgt[i].posY, tgt[i].posZ,
    //               tgt[i].velX, tgt[i].velY, tgt[i].velZ);
  }
  return true;
}

bool Doppler::parseTLVs(uint8_t *buffer)
{
  bool result;
  int i, outCount;

  // Interpret header from buffer
  MmwDemo_output_message_header_t *header = reinterpret_cast<MmwDemo_output_message_header_t *>(buffer);

  // TLVs start immediately after header
  uint8_t *tlvPtr = buffer + sizeof(MmwDemo_output_message_header_t);
  uint8_t *endPtr = buffer + header->totalPacketLen;

  for (i = 0; i < header->numTLVs; ++i)
  {
    // Ensure at least 8 bytes are available for TLV header
    if (tlvPtr + 8 > endPtr)
    {
      // Serial.printf("Error: TLV #%u header out of bounds (offset %ld)\r\n", i + 1, tlvPtr - buffer);
      break;
    }

    // Read TLV header fields safely
    uint32_t tlvType, tlvLength;
    memcpy(&tlvType, tlvPtr, sizeof(uint32_t));
    memcpy(&tlvLength, tlvPtr + 4, sizeof(uint32_t));

    // if (this->jsonLine.length() > 0)
    // {
    //   Serial.printf("TLV #%u\r\n", i + 1);
    //   // // Serial.printf("  Offset : %ld\r\n", tlvPtr - buffer);
    //   Serial.printf("  Type   : %u\r\n", tlvType);
    //   Serial.printf("  Length : %u bytes\r\n", tlvLength);

    //   // Print raw TLV header bytes for debugging
    //   Serial.printf("  Raw header: ");
    //   for (int k = 0; k < 8; ++k) 
    //   {
    //     Serial.printf("%02X ", tlvPtr[k]);
    //   }
    //   Serial.printf("\r\n");
    // }

    // Extract payload and print preview
    uint8_t *payload = tlvPtr + 8;
    size_t payloadLength = tlvLength - 8;

    switch (tlvType)
    {
    // Point Cloud (Detected Points)
    case TLV_POINT_CLOUD:
      parsePointCloudExtTLV(payload, tlvLength);
      break;
    // Target List (Tracks)
    case TLV_TARGET_LIST:
      this->parseTargetListTLV_308(payload, tlvLength);
      break;
    // Other TLV types can be handled here...
    default:
      break;
    }

    // Bounds check for the full TLV (header + payload)
    if (tlvPtr + tlvLength > endPtr)
    {
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

String Doppler::getNextLine()
{
  // Find next newline
  int nl = this->mmwave.cfgString.indexOf('\n');

  String line;
  if (nl == -1)
  {
    // Last (or only) line
    line = this->mmwave.cfgString;
    this->mmwave.cfgString = ""; // consume everything
  }
  else
  {
    // Extract substring up to newline
    line = this->mmwave.cfgString.substring(0, nl);
    this->mmwave.cfgString.remove(0, nl + 1); // remove line + '\n' from the original string
  }
  line.trim(); // remove any trailing \r or spaces
  return line;
}

void Doppler::replyRes(String incoming, String msg)
{
  int pos = incoming.lastIndexOf('}');
  if (pos >= 0)
  {
    incoming.remove(pos);
  }
  incoming += ",\"res\":\"";
  incoming += msg;
  incoming += "\"}";
  Serial.println(incoming);
}

void Doppler::replyRes(String incoming, int value)
{
  int pos = incoming.lastIndexOf('}');
  if (pos >= 0)
  {
    incoming.remove(pos);
  }
  incoming += ",\"res\":";
  incoming += value;
  incoming += "}";
  Serial.println(incoming);
}

void Doppler::handleGetCommand(String incoming)
{
  if (this->mmwave.state == MMWAVE_IDLE)
  {
    this->replyRes(incoming, "error");
  }
  else if (this->mmwave.state != MMWAVE_DONE)
  {
    this->replyRes(incoming, "busy");
  }
  else
  {
    this->setJsonLine(incoming);
  }
}

void Doppler::handleStatusCommand(String incoming)
{
  String resp = "V" + FirmwareVersion + ", (" + FirmwareDate + "), State:" + String((int)this->mmwave.state);
  this->replyRes(incoming, resp);
}

void Doppler::handleUseCase()
{
  // If there are targets, analyze them to determine use cases
  if (this->mmwave.targets.size() > 0)
  {
    // ---- Cross Traffic evaluation ----
    // Condition: target below 2m with |vx| above 0.3m/s and |vy| < 0.2m/s
    for (size_t i = 0; i < this->mmwave.targets.size(); ++i)
    {
      // Skip if too far in y
      if (this->mmwave.targets[i].y > 2.0f)
        continue;
      // Simple heuristic rules to determine use case
      if (fabs(this->mmwave.targets[i].vx) > 0.3 && fabs(this->mmwave.targets[i].vy) < 0.2)
      {
        addUniqueUseCase(this->mmwave.useCases, UseCase_enum::CrossTraffic);
      }
    }

    // ---- Tailgating (pair-based proximity) ----
    // Condition: exists a pair (i, j) such that:
    //    y < 2 m  AND  |x_i-x_j| <= 1 m  AND  |y_i-y_j| <= 1 m
    bool tailgating = false;
    // Check all pairs (i, j)
    for (size_t i = 0; i < this->mmwave.targets.size() && !tailgating; ++i)
    {
      // Skip if too far in y
      if (this->mmwave.targets[i].y > 2.0f)
        continue;
      // Check all j > i
      for (size_t j = i + 1; j < this->mmwave.targets.size(); ++j)
      {
        // Skip if too far in y
        if (this->mmwave.targets[j].y > 2.0f)
          continue; // skip if too far in y

        // Check proximity in x and y
        float dy = fabs(this->mmwave.targets[i].y - this->mmwave.targets[j].y);
        float dx = fabs(this->mmwave.targets[i].x - this->mmwave.targets[j].x);
        if (dx <= 1 && dy <= 1)
        {
          tailgating = true;
          break;
        }
      }
    }
    // If found, add use case
    if (tailgating)
    {
      addUniqueUseCase(this->mmwave.useCases, UseCase_enum::Tailgating);
    }
  }
}

void Doppler::exec()
{
  switch (this->mmwave.state)
  {
  case MMWAVE_IDLE:
    if (this->mmwave.cfgString.length() > 0)
    {
      this->mmwave.state = MMWAVE_READ_LINE;
    }
    break;

  case MMWAVE_READ_LINE:
    this->mmwave.currentLine = this->getNextLine();
    // If it's a non-comment nor empty line, send it
    if (!this->mmwave.currentLine.startsWith("%") && !this->mmwave.currentLine.isEmpty())
    {
      this->mmwave.retries = 0;
      this->mmwave.state = MMWAVE_SEND_LINE;
    }
    break;

  case MMWAVE_SEND_LINE:
    Serial.println(this->mmwave.currentLine);

    SerialRadar.println(this->mmwave.currentLine);
    if (this->mmwave.currentLine.startsWith("baudRate"))
    {
      int spaceIndex = this->mmwave.currentLine.indexOf(' ');
      if (spaceIndex > 0)
      {
        String baudStr = this->mmwave.currentLine.substring(spaceIndex + 1); // "1250000"
        long baud = baudStr.toInt();                                         // convert to integer
        // Serial.printf("Changing baud rate to %ld bps\r\n", baud);
        // ✅ Ensure last message really went out
        SerialRadar.flush();
        // Now it's safe to switch
        SerialRadar.end();
        SerialRadar.begin(baud, SERIAL_8N1, RADAR_RX, RADAR_TX);
        // Don't wait for echo, just proceed to next line
        this->mmwave.state = MMWAVE_READ_LINE;
      }
    }
    else
    {
      this->mmwave.timer = millis();
      this->mmwave.rx.ctr = 0;
      this->mmwave.state = MMWAVE_WAIT_ECHO;
    }
    break;

  case MMWAVE_WAIT_ECHO:
    // After a timeout either retry or finish
    if (millis() - this->mmwave.timer >= 1200)
    {
      if (++this->mmwave.retries <= 5)
      {
        this->mmwave.state = MMWAVE_SEND_LINE;
      }
      else
      {
        this->setCfgString(""); // clear config to stop further processing
        this->mmwave.state = MMWAVE_IDLE;
      }
      break;
    }

    // Read incoming data and look for expected echo
    while (SerialRadar.available())
    {
      char c = SerialRadar.read();
      if (this->mmwave.rx.ctr < RX_SIZE - 1)
      {
        this->mmwave.rx.message.buffer[this->mmwave.rx.ctr++] = c;
      }
      if (c == '\n')
      {
        this->mmwave.rx.message.buffer[this->mmwave.rx.ctr] = '\0';
        this->mmwave.rx.ctr = 0;
        String received((char *)this->mmwave.rx.message.buffer);
        received.trim();

        // Serial.println(received);

        // Check for expected echo
        if (received.equalsIgnoreCase("Done"))
        {
          // Proceed to next line
          if (this->mmwave.cfgString.length() > 0)
          {
            this->mmwave.state = MMWAVE_READ_LINE;
          }
          else
          {
            this->mmwave.state = MMWAVE_DONE;
          }
          break;
        }
      }
    }
    break;

  case MMWAVE_DONE:
    while (SerialRadar.available())
    {
      char c = SerialRadar.read();

      // Receive magic word...
      if (this->mmwave.rx.ctr < sizeof(MAGIC_WORD))
      {
        // Check if the received byte matches the expected magic word
        if (c == MAGIC_WORD[this->mmwave.rx.ctr])
        {
          this->mmwave.rx.message.buffer[this->mmwave.rx.ctr++] = c;
          // If magic word mismatch, reset counter
        }
        else
        {
          this->mmwave.rx.ctr = 0; // Reset if mismatch
        }
        // Receive the header until it's complete
      }
      else if (this->mmwave.rx.ctr < sizeof(MmwDemo_output_message_header_t))
      {
        this->mmwave.rx.message.buffer[this->mmwave.rx.ctr++] = c;
        // Else header is complete...
        if (this->mmwave.rx.ctr == sizeof(MmwDemo_output_message_header_t))
        {
          // If header is not valid, reset counter
          if (isHeaderValid(&this->mmwave.rx.message.header))
          {
            const uint32_t elapsed = doppler.getInterval(&doppler.mmwave.lastTick);
            this->jsonFrame = "{\"frame\":";
            this->jsonFrame += this->mmwave.rx.message.header.frameNumber;
            this->jsonFrame += ",\"dt\":";
            this->jsonFrame += String(elapsed);
            this->jsonPoints = "";
            this->jsonTargets = "";
            this->mmwave.persons = 0;
            this->mmwave.targets.clear();
            this->mmwave.useCases.clear();

            // printHeader(&this->mmwave.rx.message.header);
          }
          else
          {
            // Serial.println("Invalid header received!");
            this->mmwave.rx.ctr = 0; // Reset counter
          }
        }
        // Else receive the TLV data
      }
      else if (this->mmwave.rx.ctr < this->mmwave.rx.message.header.totalPacketLen)
      {
        this->mmwave.rx.message.buffer[this->mmwave.rx.ctr++] = c;
        // If message is complete...
        if (this->mmwave.rx.ctr == this->mmwave.rx.message.header.totalPacketLen)
        {
          // Parse TLVs
          if (parseTLVs(this->mmwave.rx.message.buffer))
          {
            if (this->jsonLine.length() > 0)
            {
              int pos = this->jsonLine.lastIndexOf('}');
              if (pos >= 0)
              {
                this->jsonLine.remove(pos);
              }
              this->jsonLine += ",\"res\":";
              Serial.print(this->jsonLine);

              this->handleUseCase();

              String payload = this->jsonFrame;
              payload += ",\"persons\":";
              payload += String(this->mmwave.persons);
              payload += ",\"use_case\":[";
              for (size_t i = 0; i < this->mmwave.useCases.size(); ++i)
              {
                payload += String(static_cast<uint8_t>(this->mmwave.useCases[i])); // convert enum to number
                if (i < this->mmwave.useCases.size() - 1)
                  payload += ",";
              }
              payload += "]";

              if (this->jsonTargets.length() > 0)
              {
                payload += ",\"tgt\":[";
                payload += this->jsonTargets + "]";
              }

              if (this->jsonLine.indexOf("raw") >= 0)
              {
                payload += ",\"raw\":[";
                payload += this->jsonPoints + "]";
              }

              payload += "}";
              uint32_t crc = crc32_le(0, (const uint8_t *)payload.c_str(), payload.length());
              payload += ",\"crc\":" + String(crc);
              Serial.print(payload);
              Serial.println("}");
              this->jsonLine = "";
            }
          }
          this->mmwave.rx.ctr = 0;
        }
      }
    }
    break;
  }
}