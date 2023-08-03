// The scaffold of a class representing the Cobot280 atom controller, on the end effector. 
// The atom appears to be accessed on the serial servo bus, using a different header (0xFE 0xFE, instead of 0xFF 0xFF)
// Peter Jansen, 2023

#include "CobotAtom.h"

/*
 *  Constructor/Destructor
 */ 

// Constructor
CobotAtom::CobotAtom() {

}

// Destructor
CobotAtom::~CobotAtom() {

}


/*
 * Display
 */

// Set the color of the display
void CobotAtom::setColor(uint8_t r, uint8_t g, uint8_t b) {
  //Serial.printf("%i %i %i\n", r, g, b);

  // First, send the read request
  uint8_t parameters[] = {r, g, b};
  const uint8_t INST_COLOR  = 0x6A;       // Best guess

  sendAtomPacket(INST_COLOR, (uint8_t*)parameters, 3);
}

// Display a rainbow pattern on the ATOM display
void CobotAtom::displayRainbow(int delayMillis, int cycles) {
    for (int cycle = 0; cycle < cycles; cycle++) {
        for (int hue = 0; hue < 360; hue++) {
            uint8_t r, g, b;
            uint8_t h = hue % 60; // We break the hue into 60 degree segments
            uint8_t x = 255 * (60 - abs(h - 30)) / 30; // Triangle wave function, gives us smooth transition

            if (hue < 60) {
                r = 255; g = x; b = 0;
            } else if (hue < 120) {
                r = x; g = 255; b = 0;
            } else if (hue < 180) {
                r = 0; g = 255; b = x;
            } else if (hue < 240) {
                r = 0; g = x; b = 255;
            } else if (hue < 300) {
                r = x; g = 0; b = 255;
            } else { // hue < 360
                r = 255; g = 0; b = x;
            }

            // Set color and delay
            this->setColor(r, g, b);
            delay(delayMillis);
        }
    }
}

