#ifndef NATIVE
#include "unittest_transport.h"
#include <HardwareSerial.h>

void unittest_uart_begin()
{
  Serial.begin(115200);
}

void unittest_uart_putchar(char c)
{
  Serial.print(c);
}

void unittest_uart_flush(){
  Serial.flush();
}

void unittest_uart_end() {
}
#endif