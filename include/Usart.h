#ifndef __Usart_h
#define __Usart_h

#include <Arduino.h>

#define MAXARGC 21

class Usart {
public:
  Usart();
  void exec();

private:
  void handleIncomingJson(const String &incoming);
  String msg;
  int bracesCount;
};

#endif
