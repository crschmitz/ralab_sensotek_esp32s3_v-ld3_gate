#include <Arduino.h>
#include <Preferences.h>
#include <esp_partition.h>
#include <esp_heap_caps.h>
#include <FFat.h>
#include "esp_task_wdt.h"
#include "soc/efuse_reg.h"
#include "Timer.h"
#include "Usart.h"
#include "Config.h"
#include "Doppler.h"

#include <nvs.h>
#include <nvs_flash.h>

// Define if the watchdog timer should be enabled
#define WATCHDOG_ENABLED false
// Set the watchdog timer timeout (in seconds)
#define WATCHDOG_TIMEOUT 4

// Global instances
Preferences preferences;
Timer timer;
Usart usart;
Config config;
Doppler doppler;

// Function to initialize the pins
void setupPins() {
  pinMode(DOUT_FAULT, INPUT);   // Fault pin as input
  pinMode(DOUT_IN, OUTPUT);     // Output pin
  pinMode(DOUT_EN, OUTPUT);     // Enable pin
  pinMode(RELAY_OUT, OUTPUT);   // Relay pin
  digitalWrite(DOUT_EN, LOW);   // Ensure TIOS is disabled at startup
  digitalWrite(RELAY_OUT, LOW); // Ensure relay is disabled at startup
}

// Function to set the digital output
void setDigitalOutput(bool active) {
  // If selected output is relay only, keep digital IO disabled
  if (config.parameters.select_out == SELECT_OUT_RELAY) {
    digitalWrite(DOUT_EN, LOW);  // Ensure TIOS is disabled at startup

  // Else digital I/O is used...
  } else {
    // Check the output mode and set the output accordingly
    switch (config.parameters.output_mode) {
      // NPN: Activate (N-Switch), Deactivate (Hi-Z)
      case OUTPUT_MODE_NPN:
        if (active) {
          digitalWrite(DOUT_EN, HIGH);  // Enable the output
          digitalWrite(DOUT_IN, HIGH);  // NPN: Activate (N-Switch)
        } else {
          digitalWrite(DOUT_EN, LOW);   // Disable the output
          digitalWrite(DOUT_IN, HIGH);  // NPN: Deactivate (Hi-Z)
        }
        break;

      // PNP: Activate (P-Switch), Deactivate (Hi-Z)        
      case OUTPUT_MODE_PNP:
        if (active) {
          digitalWrite(DOUT_EN, HIGH);  // Enable the output
          digitalWrite(DOUT_IN, LOW);   // PNP: Activate (P-Switch)
        } else {
          digitalWrite(DOUT_EN, LOW);   // Disable the outpSerial.printut
          digitalWrite(DOUT_IN, LOW);   // PNP: Deactivate (Hi-Z)
        }
        break;

      // PP: Activate (P-Switch), Deactivate (N-Switch)
      case OUTPUT_MODE_PP:
        if (active) {
          digitalWrite(DOUT_EN, HIGH);  // Enable the output
          digitalWrite(DOUT_IN, LOW);   // PP: Activate (P-Switch)
        } else {
          digitalWrite(DOUT_EN, HIGH);  // Enable the output
          digitalWrite(DOUT_IN, HIGH);  // PP: Activate (N-Switch)
        }
        break;

      default:
        break;
    }
  }
}

// Function to set the relay output
void setRelayOutput(bool active) {
  // If selected output is digital IO only, keep relay off
  if (config.parameters.select_out == SELECT_OUT_DIGITAL) {
    digitalWrite(RELAY_OUT, LOW);

  // Else if relay type is NC, activate relay when output is active
  } else if (config.parameters.relay_type == RELAY_TYPE_NC) {
    if (active) {
      digitalWrite(RELAY_OUT, HIGH);
    } else {
      digitalWrite(RELAY_OUT, LOW);
    }
  
  // Else if relay type is NO, activate relay when output is inactive
  } else if (config.parameters.relay_type == RELAY_TYPE_NO) {
    if (active) {
      digitalWrite(RELAY_OUT, LOW);
    } else {
      digitalWrite(RELAY_OUT, HIGH);
    }
  }
}

// Function to initialize NVS and preferences
void initializeNVS() {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Initialize preferences
  preferences.begin("sensotek", false);
}

// Setup function
void setup() {
  // Start serial communication for debugging
  Serial.begin(921600);
  unsigned long startWait = millis();
  while (!Serial && millis() - startWait < 1000) {
    delay(10); // espera no máximo 2 segundos
  }

  // Initialize the pins
  setupPins();

  #ifdef WATCHDOG_ENABLED
  // Initialize the watchdog timer with a timeout of 4 seconds
  esp_task_wdt_init(WATCHDOG_TIMEOUT, true);  // Enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); // Add the current thread to be watched
  Serial.printf("Watchdog timer initialized with %d seconds timeout\r\n", WATCHDOG_TIMEOUT);
  #endif

  // Start the configuration and restore the parameters
  initializeNVS();
  config.restore();

  // Set the output pins inactive
  setRelayOutput(false);
  setDigitalOutput(false);

  // List all partitions
  esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, NULL);
  if (it != NULL) {
    do {
      const esp_partition_t* partition = esp_partition_get(it);
      if (partition != NULL) {
        Serial.printf("Partition: %s, Type: %d, SubType: %d, Address: 0x%x, Size: 0x%x\r\n",
            partition->label, partition->type, partition->subtype, partition->address, partition->size);
      }
    } while (it = esp_partition_next(it));
    esp_partition_iterator_release(it);
  }

  // Check flash encryption mode
  uint32_t flash_crypt_cnt = (REG_READ(EFUSE_RD_REPEAT_DATA1_REG) >> 19) & 0x7F;
  Serial.printf("Flash crypt count: %d\r\n", __builtin_popcount(flash_crypt_cnt));

  if (__builtin_popcount(flash_crypt_cnt) % 2 == 1) {
      Serial.println("Flash encryption is enabled");
  } else {
      Serial.println("Flash encryption is disabled");
  }

  // Checking the coding scheme
  uint32_t coding_scheme = (REG_READ(EFUSE_RD_REPEAT_DATA3_REG) >> 21) & 0x3;
  if (coding_scheme == 0) {
      Serial.println("Flash encryption is in development mode");
  } else if (coding_scheme == 1) {
      Serial.println("Flash encryption is in release mode");
  } else {
      Serial.println("Unknown flash encryption mode");
  }

  // Check if PSRAM is available
  if (psramInit()) {
    Serial.println("PSRAM initialized successfully");
    Serial.printf("Total PSRAM: %d bytes\r\n", ESP.getPsramSize());
    Serial.printf("Free PSRAM: %d bytes\r\n", ESP.getFreePsram());
  } else {
    Serial.println("PSRAM initialization failed");
  }

  Serial.printf("Total heap: %d\r\n", ESP.getHeapSize());
  Serial.printf("Free heap: %d\r\n", ESP.getFreeHeap());
  Serial.printf("Total PSRAM: %d\r\n", ESP.getPsramSize());
  Serial.printf("Free PSRAM: %d\r\n", ESP.getFreePsram());

  if (!FFat.begin(true)) {
    Serial.println("Failed to mount FFat");
  } else {
    Serial.println("FFat mounted successfully");
  }

  // Start the radar
  doppler.start();
}

// Empty loop since everything is done in the tasks
void loop() {
  #ifdef WATCHDOG_ENABLED
    // Feed the watchdog timer regularly to prevent a reset (TODO - find better place to feed wdt)
    esp_task_wdt_reset();
  #endif
}

