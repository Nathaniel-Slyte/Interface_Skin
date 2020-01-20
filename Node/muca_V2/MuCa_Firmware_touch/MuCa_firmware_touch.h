//https://www.newhavendisplay.com/appnotes/datasheets/touchpanel/FT5x16_registers.pdf
//https://www.buydisplay.com/download/ic/FT5206.pdf
#include "Wire.h"

#define I2C_ADDRESS       0x38

#define MODE_NORMAL       0x00

// NORMAL
#define TOUCH_REGISTERS   31
#define STATUS            0x02

volatile bool newTouch = false;


void interruptmuca() {
  newTouch = true;
}

//// ============================== CLASS ==============================

class TouchPoint {
  public: unsigned int flag;
    unsigned int x;
    unsigned int y;
    unsigned int weight;
    unsigned int area;
    unsigned int id;
};

class MuCa {
  public:
    MuCa();
    void init();

    bool poll();

    // TOUCH
    bool updated();
    int getNumberOfTouches();
    TouchPoint getTouch(int i);


  private:
    bool isInit = false;

    // TOUCH
    TouchPoint touchpoints[5];
    byte touchRegisters[TOUCH_REGISTERS];
    void getTouchData();
    void setTouchPoints();
    byte numTouches = 0;

};

////  ============================== INITIALIZATION ==============================

MuCa::MuCa() {}

void MuCa::init() {

  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);

  Wire.begin();
  Wire.setClock(400000); // 400000 https://www.arduino.cc/en/Reference/WireSetClock

  // Initialization

  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(byte(0x00));
  Wire.write(byte(MODE_NORMAL));
  Wire.endTransmission(I2C_ADDRESS);
  delay(100);
  Serial.println("MuCa initialized");
  delay(100);
  isInit = true;
}


//// ============================== TOUCH ==============================


 bool MuCa::updated() {
   if (!isInit) return false;
   //poll();


   int reading = 0;
   while (1) {
     Wire.requestFrom(I2C_ADDRESS, 3);
     if (Wire.available()) {
       reading = Wire.read();
       int high_bit = (reading & (1 << 7));
       Serial.println(byte(reading));
       // if (high_bit == 0) {
       //   break;
       //}
     }
   }


  if (newTouch == true) {
    newTouch = false;
    return true;
  } else {
    return false;
  }
 }

TouchPoint MuCa::getTouch(int i) {
  return touchpoints[i];
}

 bool MuCa::poll() {
  getTouchData();
  setTouchPoints();
  return true;
 }

void MuCa::getTouchData() {
  Wire.requestFrom(I2C_ADDRESS, TOUCH_REGISTERS);
  int register_number = 0;
  // get all register bytes when available
  while (Wire.available())
  {
    touchRegisters[register_number++] = Wire.read();
  }
}

void MuCa::setTouchPoints() {
  numTouches = touchRegisters[STATUS] & 0xF;
  unsigned int registerIndex = 0;
  for (int i = 0; i < numTouches; i++) {
    // 0 1 0 1 0 0 1 1 0
    // HIGH          LOW
    // var high = b >> 4; var low = b & 0x0F;

    registerIndex = (i * 6) + 3;
    touchpoints[i].flag    = touchRegisters[registerIndex] >> 6; // 0 = down, 1 = lift up, // 2 = contact // 3 = no event
    // touchpoints[i].flag = touchRegisters[registerIndex] les deux premiers bits
    touchpoints[i].x       = word(touchRegisters[registerIndex] & 0x0f, touchRegisters[registerIndex + 1]);
    touchpoints[i].y       = word(touchRegisters[registerIndex + 2] & 0x0f, touchRegisters[registerIndex + 3]);
    touchpoints[i].id      = touchRegisters[registerIndex + 2] >> 4;
    touchpoints[i].weight  = touchRegisters[registerIndex + 4];
    touchpoints[i].area    = touchRegisters[registerIndex + 5] >> 4;
  }
}

int MuCa::getNumberOfTouches() {
  return numTouches;
}


//https://www.buydisplay.com/download/ic/FT5206.pdf + https://github.com/optisimon/ft5406-capacitive-touch/blob/master/CapacitanceVisualizer/FT5406.hpp
// https://github.com/hyvapetteri/touchscreen-cardiography + http://optisimon.com/raspberrypi/touch/ft5406/2016/07/13/raspberry-pi-7-inch-touchscreen-hacking/
//https://www.newhavendisplay.com/app_notes/FT5x16.pdf + https://www.newhavendisplay.com/appnotes/datasheets/touchpanel/FT5x16_registers.pdf
//https://github.com/azzazza/patch_kernel_q415/blob/master/drivers/input/touchscreen/ft5x06_ts.c
