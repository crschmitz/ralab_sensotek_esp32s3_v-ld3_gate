#include <ArduinoJson.h>
#include "Usart.h"
#include "Config.h"
#include "Doppler.h"
#include "defines.h"

extern "C" {
  #include "esp32/rom/crc.h"
}

String FirmwareVersion = "104";
String FirmwareDate    = "08.10.2025";

extern Config config;
extern Doppler doppler;

Usart::Usart() {
  this->bracesCount = 0;
  this->msg = "";
}

void Usart::handleIncomingJson(const String &incoming) {
  // 1) Validate JSON
  StaticJsonDocument<2048> doc;
  DeserializationError err = deserializeJson(doc, incoming);
  if (err) {
    return;
  }

  // 2) Check "id"
  if (!doc.containsKey("id")) {
    return;
  }

  int id = doc["id"].as<int>();
  // Validate id
  if (id == 1) {
    // If "cmd" is present, handle commands
    if (doc.containsKey("cmd")) {
      String cmd = doc["cmd"].as<const char*>();
      // If command is "get", save the JSON line to Doppler
      if (cmd == "get") {
        doppler.handleGetCommand(incoming);
      } else if (cmd == "status") {
        doppler.handleStatusCommand(incoming);
      } else if (cmd == "cfg") {
        // Check for required keys
        if (doc.containsKey("file")) {
          String cfg = doc["file"].as<const char*>();
          // If crc is present, we could validate it here
          if (doc.containsKey("crc")) {
            int crc = doc["crc"].as<int>();
            uint32_t calc_crc = crc32_le(0, (const uint8_t*)cfg.c_str(), cfg.length());
            String result = "";
            if (crc != (int)calc_crc) {
              result = "error";
            } else {
              doppler.setCfgString(cfg);
              result = "done";
            }
            doc.remove("file");
            doc.remove("crc");
            doc["res"] = result;
            serializeJson(doc, Serial);            
            Serial.println();
          }
        } else {
          Serial.println("cfg command missing 'file' or 'crc'");
        }
      }
    }
  }
}

void Usart::exec() {
  while (Serial.available()) {
    char c = char(Serial.read());
    // If valid ASCII character, add to command buffer
    if (isAscii(c)) {
      if (this->bracesCount == 0) {
        if (c == '{') {
          this->msg = "{";    // start of new JSON command
          this->bracesCount = 1;
        }
      } else {
        this->msg += c;
        if (c == '{') {
          this->bracesCount++;
        } else if (c == '}') {
          this->bracesCount--;
          if (this->bracesCount == 0) {
            this->handleIncomingJson(this->msg);
            // Serial.println(this->msg); // echo back the complete JSON command
            this->msg = "";
          }
        }
      }
    } else {
      this->bracesCount = 0;
    }
  }
}
