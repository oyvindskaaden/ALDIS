#include <SoftwareSerial.h>

enum class BaudRate : uint32_t {
  BAUD_9600   = 9600,
  BAUD_14400  = 14400,
  BAUD_19200  = 19200, 
  BAUD_38400  = 38400, 
  BAUD_57600  = 57600, 
  BAUD_115200 = 115200
};

enum class State{
  IDLE,
  STAGE_BUSY,
  TIMEOUT,
  STAGE_RDY
} state;

struct RGB {
  bool red;
  bool green;
  bool blue;
};

SoftwareSerial uart(2, 3); // RX -> Pin 2, TX -> Pin 3

uint8_t rgbPins[3] = {9, 8, 7};

uint8_t button = 5;

RGB stateColors[4] {
  {0, 0, 1},  //! IDLE        -> Blue
  {1, 0, 0},  //! STAGE_BUSY  -> Red
  {1, 1, 0},  //! TIMEOUT     -> Yellow
  {0, 1, 0}   //! STAGE_RDY   -> Green
}

void WriteRGB (uint8_t rgb_pins[3], RGB rgb) {
  digitalWrite(rgb_pins[0], rgb.red);
  digitalWrite(rgb_pins[1], rgb.green);
  digitalWrite(rgb_pins[2], rgb.blue);
}

void setup() {
  // put your setup code here, to run once:
  for (uint8_t p : rgbPins) {
    pinMode(p, OUTPUT);
  }
  pinMode(button, INPUT_PULLUP);

  uart.begin((long)BaudRate::BAUD_115200);
}

void loop() {
  // put your main code here, to run repeatedly:

}


