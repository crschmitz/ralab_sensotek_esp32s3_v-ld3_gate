#ifndef __Config_h
#define __Config_h

#include <Preferences.h>
#include "defines.h"

#define COD_VALIDATION    20220126
#define CONFIG_RELEASE    20220126

typedef enum {
  DSP_OFF  = 0,
  DSP_ON   = 1,
  DSP_FULL = 2,
} ENUM_DSP_MODE;

/* Configuration struct size: 1024 bytes */
/* Size must be multiple of 4 guarantee cks at the end */
typedef struct {
  uint32_t cod_validation;  
	struct {
    uint8_t bypass_mode;
    uint8_t reserved[63];
    uint32_t reserved32b[14];
	} radar;

	struct {
		uint32_t dsp_mode;						// dsp mode (use the same protocol as DSP radar)
		uint16_t ampl_filter;         // amplitude filter (validate only amplitudes above this limit)
    uint16_t dummy;
    float offset;                 // offset to computed opening range (default is 0.5m)
    uint16_t led_slow;
    uint16_t led_fast;
    float door_range;             // distance to the door (only process detections above this range)
    uint16_t door_close;          // detections with constant speed to validate door closing
    uint16_t door_clear;          // detections to clear door closing flag
    struct {
      float width;                // user field width  (m): -----> used in dip 11 = position 3 (case 4)
      float height;               // user field height (m): --/
    } user_field;
    uint32_t reserved[8];
	} detection;
	
  struct {
	  uint16_t varvel;              // max variation in velocity to use a raw detection [km/h]
    uint16_t vardst;              // max variation in distance to use a raw detection [m]
    uint16_t varang;              // max variation in angle    to use a raw detection [Â°]
    uint16_t blocks;              // mean blocks to output a result
    uint16_t spread1;             // spread1 (limit for 1 person)
    uint16_t spread2;             // spread2 (limit for 2 persons)
    uint16_t reserved[58];
  } filter;

  struct {
    float min;                    // minimal range to evaluate cross traffic (in meters)
    float max;                    // maximum range to evaluate cross traffic (in meters)
    float dy;                     // delta Y to evaluate cross traffic
    float dx;                     // delta X to evaluate cross traffic
    float xy;                     // ratio xy to set cross traffic
    float yx;                     // ratio yx to reset cross traffic
    uint8_t start;                // true if cross traffic can be set at start (depending on starting angle)
    uint8_t angle;                // starting angle to set cross traffic from the start
    uint8_t integ;                // cross traffic integration (cycles to validate falling edge)
    uint8_t dummy;
    float channel;                // cross traffic channel (used to reset cross traffic when it starts as 1)
    float speed;                  // minimal speed to evaluate cross traffic
  } cross;

  uint8_t brightness;
  uint8_t door_speed;
  uint8_t dip_switch;
  uint8_t reserved8b;

  uint8_t relay_type;             // relay type  (0 = NO, 1 = NC)
  uint8_t output_mode;            // output mode (0 = NPN ; 1 = PNP ; 2 = PP)
  uint8_t select_out;             // select out  (0 = relay ; 1 = digital IO ; 2 = both)
  uint8_t dummy;

  uint32_t reserved[36];

	// Release number
  uint32_t release;
  uint32_t crc;                     // CRC-32 (must be last 32-bits of config structure)
} ST_PARAMETERS;

// Configuration settings
class Config {
public:
  Config();
  void loop();
  void commit();
  void commit(uint32_t timeout);
  bool check();
  bool restore();
  void restoreToFactory();
  void print();
  void printConfiguration();
  void onTimer();
  uint32_t getCRC();
  uint32_t timer;
  uint32_t timeout;

  ST_PARAMETERS parameters;
};

extern Preferences preferences;

#endif