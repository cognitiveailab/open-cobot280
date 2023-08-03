// CobotAtom.h

#if !defined(COBOT_ATOM_h) 
#define COBOT_ATOM_h

#include <stdint.h>

#include "SerialBusServo.h"


class CobotAtom {

  public:
    CobotAtom();
    ~CobotAtom();

    // Set the color of the display
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    void displayRainbow(int delayMillis, int cycles);

};


#endif