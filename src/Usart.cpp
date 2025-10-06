#include <ArduinoJson.h>
#include "Usart.h"
#include "Config.h"
#include "Doppler.h"
#include "defines.h"

extern "C" {
  #include "esp32/rom/crc.h"
}

String FirmwareVersion = "102";
String FirmwareDate    = "14.09.2025";

extern Config config;
extern Doppler doppler;

Usart::Usart() {
  this->bracesCount = 0;
  this->msg = "";
  this->argc = 0;
}

void Usart::handleIncomingJson(const String &incoming) {
  // 1) Validate JSON
  StaticJsonDocument<2048> doc;
  DeserializationError err = deserializeJson(doc, incoming);
  if (err) {
    Serial.println("Error deserializing JSON");
    return;
  }

  // 2) Check "id"
  if (!doc.containsKey("id")) {
    Serial.println("No id");
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
              result = "CRC Error";
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

// Split a command line into separate arguments.
void Usart::splitCommandLine() {
  int i, lastIndex = 0;
  this->argc = 0;
  for (i = 0; i < this->msg.length(); i++) {
    // Loop through each character and check if it's a comma
    if (this->msg.substring(i, i+1) == " ") {
      // Grab the piece from the last index up to the current position and store it
      this->argv[this->argc++] = this->msg.substring(lastIndex, i);
      // Update the last position and add 1, so it starts from the next character
      lastIndex = i + 1;
    }

    // If we're at the end of the string (no more commas to stop us)
    if (i == this->msg.length() - 1) {
      // Grab the last part of the string from the lastIndex to the end
      this->argv[this->argc++] = this->msg.substring(lastIndex, i+1);
    }
  }
}  

// Handles byte reception via serial and returns true if a command is received
bool Usart::getCommand() {
  bool result = false;
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
  return result;
}

/**************************************************************************//**
 * @brief Print a help screen.
 *****************************************************************************/
void Usart::printHelp(void) {
  Serial.printf("\r\nmmWave Sensor Sensotek v%s - %s\r\n", FirmwareVersion, FirmwareDate);
  Serial.printf(
    "Available commands:\r\n"
    "   h            : Show this help\r\n"
    "   cfg          : Read the configuration\r\n"
    "\r\n" );
}

void Usart::printDSPMode(void) {
  switch(config.parameters.detection.dsp_mode) {
    case DSP_OFF:   Serial.printf("dsp off\r\n");  break;
    case DSP_ON:    Serial.printf("dsp on\r\n");   break;
    case DSP_FULL:  Serial.printf("dsp full\r\n"); break;
    default: break;
  }
}

void Usart::printParams(uint8_t *parameters) {
  int i;
  Serial.printf("\r\nParameters:\r\n");
  for (i=0 ; i<18 ; i++) {
    Serial.printf("%c", parameters[i]);
  }
  Serial.printf(" ");
  for (i=19 ; i<42 ; i++) {
    Serial.printf("%02X ", parameters[i]);
  }
  Serial.printf("\r\n");
}

void Usart::exec() {
  // ____________________________
  // If a command was received...
  if (this->getCommand()) {
    this->execCommand();
  }
}

bool Usart::updateParam(uint8_t *param, uint8_t min, uint8_t max) {
  bool result = false;
  uint16_t value;
  if (this->argc > 1) {
    value = atoi(this->argv[1].c_str());
    if (value >= min && value <= max) {
      if (*param != value) {
        *param = value;
        config.commit();
        result = true;
      }
    } else {
      Serial.printf("Invalid! Range is [%d:%d]\r\n", min, max);
      return result;
    }
  }
  Serial.printf("%s %d\r\n", this->argv[0].c_str(), *param);
  return result;
}

bool Usart::updateParam(uint16_t *param, uint16_t min, uint16_t max) {
  bool result = false;
  uint16_t value;
  if (this->argc > 1) {
    value = atoi(this->argv[1].c_str());
    if (value >= min && value <= max) {
      if (*param != value) {
        *param = value;
        config.commit();
        result = true;
      }
    } else {
      Serial.printf("Invalid! Range is [%d:%d]\r\n", min, max);
      return result;
    }
  }
  Serial.printf("%s %d\r\n", this->argv[0].c_str(), *param);
  return result;
}

bool Usart::updateParam(float *param, float min, float max) {
  bool result = false;
  float value;
  if (this->argc > 1) {
    value = atof(this->argv[1].c_str());
    if (value >= min && value <= max) {
      if (*param != value) {
        *param = value;
        config.commit();
        result = true;
      }
    } else {
      Serial.printf("Invalid! Range is [%.1f:%.1f]\r\n", min, max);
      return result;
    }
  }
  Serial.printf("%s %.1f\r\n", this->argv[0].c_str(), *param);
  return result;
}

// Executes received command
void Usart::execCommand() {
  if (!this->argv[0].compareTo("h")) {
    this->printHelp();

  } else if (!this->argv[0].compareTo("version")) {
    doppler.version();

  } else if (!this->argv[0].compareTo("reset")) {
    esp_restart();

  } else if (!this->argv[0].compareTo("bypass")) {
    updateParam(&config.parameters.radar.bypass_mode, 0, 1);

  } else if (!this->argv[0].compareTo("cfg")) {
    config.printConfiguration();

  /* If invalid command */
  } else if (argv[0]) {
    Serial.printf("Invalid command\r\n");
    Serial.printf("%s\r\n", argv[0]);
  }
}
