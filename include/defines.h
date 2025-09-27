#ifndef __DEFINES_H
#define __DEFINES_H

//LED
#define LED_R 2 //CHANGE NEEDED V3.1
#define LED_G 4 //CHANGE NEEDED V3.1
#define LED_B 1 //CHANGE NEEDED V3.1

#define PWM_CHANNEL_R 0
#define PWM_CHANNEL_G 1
#define PWM_CHANNEL_B 2

#define PWM_FREQUENCY 2000
#define PWM_RESOLUTION 8

//OUTPUTS
#define RELAY_OUT 15
#define DOUT_FAULT 12
#define DOUT_IN 13
#define DOUT_EN 14

//SWITCH
#define SWA 5
#define SWB 6
#define SWC 7
#define SWD 8
#define SWCEN 9

//RS485
#define RS485_DIR 17
#define RS485_TX 18
#define RS485_RX 16

//RADAR
#define RADAR_TX 10
#define RADAR_RX 11

//UART0
#define UART_TX0 39
#define UART_RX0 40

//I2C
#define SDA 34
#define SCL 33

//CAN
#define CANRX 21
#define CANTX 26
#define CANSILENT 47
#define CAMTERM 48

//I2C Address
#define EEPROM_ADDRESS 0x50
#define IMU_ADDRESS 0x6A
#define MAGNET_ADDRESS 0x1C
#define OLED_ADDRESS 0x3C

// Output mode definitions
#define OUTPUT_MODE_NPN 0
#define OUTPUT_MODE_PNP 1
#define OUTPUT_MODE_PP  2

// Relay type definitions
#define RELAY_TYPE_NC  0
#define RELAY_TYPE_NO  1

// Select output definitions
#define SELECT_OUT_BOTH    0
#define SELECT_OUT_RELAY   1
#define SELECT_OUT_DIGITAL 2

void setRelayOutput(bool active);
void setDigitalOutput(bool active);
void setLedRGBColor(uint8_t red, uint8_t green, uint8_t blue);

#endif
