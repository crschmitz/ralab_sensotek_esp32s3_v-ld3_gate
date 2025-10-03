#ifndef __Usart_h
#define __Usart_h

#include <Arduino.h>

#define MAXARGC 21

class Usart {
public:
  Usart();
  void exec();
  void printParams(uint8_t *parameters);
  String getCmd() { return this->cmd; }
  void setCmd(String cmd) { this->cmd = cmd; }

private:
  void printHelp();
  void splitCommandLine();
  bool getCommand();
  void execCommand();
  void printDSPMode();
  bool updateParam(uint8_t *param, uint8_t min, uint8_t max);
  bool updateParam(uint16_t *param, uint16_t min, uint16_t max);
  bool updateParam(float *param, float min, float max);
  String processCommand(const String &cmd);
  void handleIncomingJson(const String &incoming);

  String msg;
  String cmd;
  int argc;
  String argv[MAXARGC];
};

#endif
