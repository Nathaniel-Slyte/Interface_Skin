//https://www.newhavendisplay.com/appnotes/datasheets/touchpanel/FT5x16_registers.pdf
//https://www.buydisplay.com/download/ic/FT5206.pdf
#include "Wire.h"

#define I2C_ADDRESS       0x38

#define MODE_NORMAL       0x00
#define MODE_TEST         0x40

// RAW
#define NUM_ROWS          21 // 21
#define ROW_GAP           9
#define NUM_COLUMNS       12

#define ROWS_USE          12

#define CALIBRATION_MAX   3
#define CALIB_THRESHOLD   0
#define CTP_INT           2

//// ============================== CLASS ==============================

class MuCa {
  public:
    MuCa();
    String messages_ble = "void";
    
    void init(bool interupt = true);
    //void skipLine(MucaLine line, const short lineNumber[], size_t size );

    //CHORE
    //void setGain(int val);
    //void printInfo();
    //void autocal();
    //void printAllRegisters();
    //void setNumTouchPoints();
    //void setReportRate(unsigned short rate);

    //RAW
    unsigned int grid[NUM_ROWS * NUM_COLUMNS];
    //void useRawData(bool useRaw);
    //void getRawData();

    // I2C
    byte readRegister(byte reg,short numberBytes);
    byte setRegister(byte reg, byte val);

  private:
    
    //CHORE
    //bool poll();
    bool isInit = false;
    bool useInterrupt = true;
    bool skippedLines[NUM_ROWS + NUM_COLUMNS]; // Maximum SkippedLines
   

    //RAW
    bool rawData = false;
};


////////////////////
//////////////////////////////////////// SETUP ////////////////////////////////////////
////////////////////

MuCa::MuCa() {}

byte MuCa::readRegister(byte reg, short numberBytes) {

  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(I2C_ADDRESS, numberBytes, false);
  byte readedValue = Wire.read();
  return readedValue;
}

byte MuCa::setRegister(byte reg, byte val) {

  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(val); 
  return Wire.endTransmission(false);
}


void MuCa::init(bool interupt) {

  useInterrupt = interupt;
  //Setup I2C
  digitalWrite(25, LOW);
  digitalWrite(26, LOW);
  digitalWrite(27, LOW);

  Wire.begin();
  Wire.setClock(100000); // 400000 https://www.arduino.cc/en/Reference/WireSetClock
  //Wire.setClock(400000); // 400000 https://www.arduino.cc/en/Reference/WireSetClock

  Wire.setTimeout(200);

  byte initDone = -1;
  initDone = setRegister(0x00,MODE_NORMAL);
  messages_ble = initDone;
  //Serial.println("[Muca] Set NORMAL mode");


  if (initDone == 0) {
    //Serial.println("[Muca] Initialized");
    messages_ble = "[Muca] Initialized";
    delay(100);
    isInit = true;
    delay(100);
  } else {
    //Serial.println("[Muca] Error while setting up Muca. Are you sure the SDA/SCL are connected?");
  }
  /*
    // Interrupt
  if(useInterrupt) {
      pinMode(CTP_INT ,INPUT);
    #ifdef digitalPinToInterrupt
    // Serial.println("[Muca] Attachinterrupt");
     attachInterrupt(digitalPinToInterrupt(CTP_INT),interruptmuca,FALLING);
    #else
      attachInterrupt(0,touch_interrupt,FALLING);
    #endif   
  }
  */

  setRegister(0xA7,0x04); // Set autocalibration
}

//https://www.buydisplay.com/download/ic/FT5206.pdf + https://github.com/optisimon/ft5406-capacitive-touch/blob/master/CapacitanceVisualizer/FT5406.hpp
// https://github.com/hyvapetteri/touchscreen-cardiography + http://optisimon.com/raspberrypi/touch/ft5406/2016/07/13/raspberry-pi-7-inch-touchscreen-hacking/
//https://www.newhavendisplay.com/app_notes/FT5x16.pdf + https://www.newhavendisplay.com/appnotes/datasheets/touchpanel/FT5x16_registers.pdf
//https://github.com/azzazza/patch_kernel_q415/blob/master/drivers/input/touchscreen/ft5x06_ts.c
