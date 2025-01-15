#line 1 "D:\\Project\\ProjectArduino\\Cyberhourglass\\Delay.h"
#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
class NonBlockDelay {
    unsigned long iTimeout;
  public:
    void Delay (unsigned long);
    bool Timeout (void);
    unsigned long Time(void);
};
