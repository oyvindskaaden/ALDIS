#include <SoftwareSerial.h>

#define   BUSY_BLINK_PERIOD_MS    250
#define   TIMEOUT_BLINK_PERIOD_MS 500
#define   RDY_BLINK_PERIOD_MS     250
#define   IDLE_PULSE_PERIOD_MS    8
#define   TIMEOUT_PULSE_PERIOD_MS 1
#define   TIMEOUT_DELAY_MS        10000
#define   TIME_TO_IDLE_MS         3000   

#define   SERIAL_SOF              0x88
#define   SERIAL_MAX_LEN          5

enum class SerialCMD : uint8_t {
  STATE       = 0x10,
  BUTTON      = 0x20
};

enum class BaudRate : uint32_t {
  BAUD_9600   = 9600,
  BAUD_14400  = 14400,
  BAUD_19200  = 19200, 
  BAUD_38400  = 38400, 
  BAUD_57600  = 57600, 
  BAUD_115200 = 115200
};

enum class State : uint8_t{
  IDLE,
  STAGE_BUSY,
  TIMEOUT,
  STAGE_RDY
};

struct RGB {
  bool red;
  bool green;
  bool blue;
};


enum class PacketState {
  GET_SOF,
  GET_DATA
};

struct PacketInfo {
  PacketState state;
  uint8_t     len;
  uint8_t     data[3];
};

PacketInfo packet {
  .state  = PacketState::GET_SOF,
  .len    = 0
};



State state = State::IDLE;
State nextState = State::STAGE_BUSY;
uint32_t lastChange = 0;
uint32_t stateNow = 0;


uint8_t nextIntensity  = 255;
bool isReducingIntensity  = true;
uint32_t now = 0;
uint32_t then = 0;
bool isLedOn = false;

SoftwareSerial uart(5, 6); // RX -> Pin 5, TX -> Pin 6

// Red: Pin 11, Green: Pin 10, Blue: Pin 9
uint8_t rgbPins[3] = {11, 10, 9};
uint8_t button = 2;


RGB stateColors[4] {
  {0, 0, 1},  //! IDLE        -> Blue
  {1, 0, 0},  //! STAGE_BUSY  -> Red
  {1, 1, 0},  //! TIMEOUT     -> Yellow
  {0, 1, 0}   //! STAGE_RDY   -> Green
};

// Global ISR
void ChangeStateISR() {
  lastChange = millis();
  state = nextState;
}


// Table used for CRC calculation
uint8_t crcLookup[256] = {
  0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75, 0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B, 
  0x1C, 0x8D, 0xFF, 0x6E, 0x1B, 0x8A, 0xF8, 0x69, 0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67, 
  0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE, 0xDC, 0x4D, 0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43, 
  0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51, 0x2A, 0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F, 
  0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05, 0x7E, 0xEF, 0x9D, 0x0C, 0x79, 0xE8, 0x9A, 0x0B, 
  0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19, 0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86, 0x17, 
  0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D, 0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33, 
  0x54, 0xC5, 0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21, 0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F, 
  0xE0, 0x71, 0x03, 0x92, 0xE7, 0x76, 0x04, 0x95, 0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B, 
  0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89, 0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87, 
  0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD, 0xD6, 0x47, 0x35, 0xA4, 0xD1, 0x40, 0x32, 0xA3, 
  0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1, 0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C, 0x2E, 0xBF, 
  0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5, 0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB, 
  0x8C, 0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9, 0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7, 
  0xA8, 0x39, 0x4B, 0xDA, 0xAF, 0x3E, 0x4C, 0xDD, 0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3, 
  0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50, 0xC1, 0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF
};


uint8_t CRC8(uint8_t *pData, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    uint8_t data = crc ^ *pData++;
    crc = crcLookup[data];
  }
  return crc;
}

void DecodePacket(uint8_t data[3]) {
  switch ((SerialCMD)data[0])
  {
  case SerialCMD::STATE:
    if (data[1] <= (uint8_t)State::STAGE_RDY) {
      lastChange = millis();
      state = (State)data[1];
    }
    break;
  case SerialCMD::BUTTON:
    ChangeStateISR();
    break;
  
  default:
    break;
  }
}


void SendState() {
  uint8_t data[5] = {SERIAL_SOF, SERIAL_SOF, (uint8_t)SerialCMD::STATE, (uint8_t)state, 0x00};

  data[4] = CRC8(&data[2], 2);

  //uart.write(data, sizeof(data));
  Serial.write(data, sizeof(data));
  
  /// DEBUG
  /*for (int i = 0; i<sizeof(data); i++) {
    Serial.print(data[i], HEX);
  }*/
  ///
}

void ReceiveData() {
  
  //if (uart.available() >= (packet.len != 0) ? packet.len : 1) {
  if (Serial.available() >= (packet.len != 0) ? packet.len : 1) {
    //uint8_t numBytes = uart.readBytes(packet.data, (packet.len != 0) ? packet.len : 1);
    uint8_t numBytes = Serial.readBytes(packet.data, (packet.len != 0) ? packet.len : 1);
    
    /// DEBUG
    /*for(int i = 0; i < numBytes; i++) {
      Serial.print(packet.data[i], HEX);
    }*/
    ///
    
    switch (packet.state)
    {
    case PacketState::GET_SOF:
      if (packet.data[0] == SERIAL_SOF)
        packet.len++;
      if (packet.len == 2) {
        packet.len = 3;
        packet.state = PacketState::GET_DATA;
      }

      break;
    case PacketState::GET_DATA:
      if (!CRC8(packet.data, sizeof(packet.data)))
        DecodePacket(packet.data);
      break;
    default:
      break;
    }
  }
}


void WriteRGB (uint8_t rgb_pins[3], RGB rgb, uint8_t intensity) {
  analogWrite(rgb_pins[0], rgb.red * intensity);
  analogWrite(rgb_pins[1], rgb.green * intensity);
  analogWrite(rgb_pins[2], rgb.blue * intensity);
}


void WriteRGB (uint8_t rgb_pins[3], RGB rgb) {
  WriteRGB(rgb_pins, rgb, 255);
}


void PulseLeds(State state, uint8_t delay) {
  if ((now = millis()) - then > delay) {
    then = now;

    WriteRGB(rgbPins, stateColors[(uint8_t)state], nextIntensity);

    if (isReducingIntensity) {
      if (nextIntensity > 0)
        nextIntensity--;
      else
        isReducingIntensity = false;
    }
    else {
      if (nextIntensity < 255)
        nextIntensity++;
      else
        isReducingIntensity = true;
    }
  }
}


void BlinkLeds(State state, uint16_t delay) {
  if ((now = millis()) - then > delay) {
    then = now;

    isLedOn = !isLedOn;
    nextIntensity = isLedOn * 255;

    WriteRGB(rgbPins, stateColors[(uint8_t)state], nextIntensity);
  }
}


void DoIdleState() {
  nextState = State::STAGE_BUSY;

  PulseLeds(State::IDLE, IDLE_PULSE_PERIOD_MS);
}


void DoStageBusyState(bool isPaused) {
  if (millis() - lastChange > TIMEOUT_DELAY_MS && !isPaused) {
    lastChange = millis();
    state = State::TIMEOUT;
  }

  nextState = State::STAGE_RDY;
  BlinkLeds(State::STAGE_BUSY, BUSY_BLINK_PERIOD_MS);
}


void DoTimeoutState() {
  nextState = State::STAGE_RDY;
  
  PulseLeds(State::TIMEOUT, TIMEOUT_PULSE_PERIOD_MS);
}


void DoStageReadyState() {

  if (millis() - lastChange > TIME_TO_IDLE_MS) {
    lastChange = millis();
    state = State::IDLE;
    nextState = State::STAGE_BUSY;
  }

  nextState = State::IDLE;
  BlinkLeds(State::STAGE_RDY, RDY_BLINK_PERIOD_MS);
}





void setup() {
  for (uint8_t p : rgbPins) {
    pinMode(p, OUTPUT);
  }
  pinMode(button, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(button), ChangeStateISR, RISING);
  Serial.begin((long)BaudRate::BAUD_9600);

  
  uart.begin((long)BaudRate::BAUD_9600);
}

State lastState = State::IDLE;
void loop() {

  if(lastState != state) {
    SendState();
    lastState = state;
  }
  switch (state) {
  case State::IDLE:
    DoIdleState();
    break;
  case State::STAGE_BUSY:
    DoStageBusyState(!digitalRead(button));
    break;
  case State::TIMEOUT:
    DoTimeoutState();
    break;
  case State::STAGE_RDY:
    DoStageReadyState();
    break;
  }

  ReceiveData();

}
