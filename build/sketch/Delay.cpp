#line 1 "D:\\Project\\ProjectArduino\\Cyberhourglass\\Delay.cpp"
#include "Delay.h"
void NonBlockDelay::Delay (unsigned long t)
{
  iTimeout = millis() + t;
  return;
};
bool NonBlockDelay::Timeout (void)
{
  return (iTimeout < millis());
}
unsigned long NonBlockDelay::Time(void)
 {
   return iTimeout;
 }
