#ifndef DOPPLER_H
#define DOPPLER_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <FS.h>     // Required for fs::File
#include <FFat.h>   // Or SPIFFS.h or SD.h, depending on the FS used

#define RX_SIZE               8192
#define TX_SIZE               2048
#define MAX_TRACKED_OBJECTS   32

#define PLATFORM_TYPE         0x000A6432 // Platform type (IWRL6432)

enum : uint32_t {
  TLV_POINT_CLOUD            = 301,
  TLV_TARGET_LIST            = 308,
  TLV_POINT_SIDE_INFO        = 310,
  TLV_TRACK_EXTENT_HEIGHT    = 311,
};

typedef enum {
  MMWAVE_IDLE = 0,
  MMWAVE_READ_LINE,
  MMWAVE_SEND_LINE,
  MMWAVE_WAIT_ECHO,
  MMWAVE_DONE,
  MMWAVE_TLV,
} mmwave_state_e;

typedef struct OutputMessageHeader {
  uint16_t magicWord[4];      /* Sync word: {0x0102,0x0304,0x0506,0x0708} */
  uint32_t version;           /* MajorNum*2^24+MinorNum*2^16+BugfixNum*2^8+BuildNum */
  uint32_t totalPacketLen;    /* Total packet length including header in Bytes */
  uint32_t platform;          /* platform type */
  uint32_t frameNumber;       /* Frame number */
  uint32_t timeCpuCycles;     /* Time in CPU cycles when the message was created */
  uint32_t numDetectedObj;    /* Number of detected objects */
  uint32_t numTLVs;           /* Number of TLVs */
  uint32_t subFrameNumber;    /* Subframe number */
} MmwDemo_output_message_header_t;

class Doppler {
public:
  Doppler();
  void start();
  void startTasks();
  void exec();
  void version();
  uint32_t getInterval(uint64_t *ticks);

  bool parseTLVs(uint8_t *buffer);
  bool parsePointCloudExtTLV(const uint8_t* payload, int length);
  bool parseTargetListTLV_308(const uint8_t* tlvPayload, int length);

  void handleGetCommand(String incoming);
  void handleStatusCommand(String incoming);
  void setJsonLine(const String &line) { this->jsonLine = line; }
  String getJsonLine() { return this->jsonLine; }

  void setCfgString(const String &cfg) { this->mmwave.cfgString = cfg; }

  mmwave_state_e getState() { return this->mmwave.state; }

  struct {
    mmwave_state_e state;
    uint8_t timeout;
    uint8_t cycle;

    struct {
      uint16_t i, f;
      uint16_t ctr;
    } tx;
    struct {
      union {
        MmwDemo_output_message_header_t header;
        uint8_t buffer[RX_SIZE];
      } message;
      uint16_t ctr;
    } rx;

    // Optional: other runtime metadata
    fs::File cfgFile;
    String currentLine;
    String cfgString;
    uint32_t timer = 0;
    uint64_t lastTick = 0;
    uint8_t retries = 0;
    uint32_t persons = 0;
  } mmwave;

private:
  bool process();
  void flush();
  String getNextLine();
  void replyRes(String incoming, String msg);
  void replyRes(String incoming, int value);

  String jsonLine;
  String jsonFrame;
  String jsonTargets;
  String jsonPoints;

  // Task handle
  TaskHandle_t sendTaskHandle;
};

uint16_t Succ(uint16_t i, uint16_t size);
uint16_t Pred(uint16_t i, uint16_t size);

#endif // DOPPLER_H
