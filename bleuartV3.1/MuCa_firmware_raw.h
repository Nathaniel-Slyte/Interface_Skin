//https://www.newhavendisplay.com/appnotes/datasheets/touchpanel/FT5x16_registers.pdf
//https://www.buydisplay.com/download/ic/FT5206.pdf
#include "Wire.h"

#define I2C_ADDRESS       0x38

#define MODE_NORMAL       0x00
#define MODE_TEST         0x40

// RAW
#define NUM_ROWS          12 // 21
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
    bool updated();

    //RAW
    unsigned int grid[NUM_ROWS * NUM_COLUMNS];
    void useRawData(bool useRaw);
    void getRawData();

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
  //Serial.println("[Muca] Set NORMAL mode");


  if (initDone == 0) {
    messages_ble = "[Muca] Initialized";
    delay(100);
    isInit = true;
    delay(100);
  } else {
    messages_ble = initDone;
  }

  setRegister(0xA7,0x04); // Set autocalibration
}


bool MuCa::updated() {
  if (!isInit)
    return false;

  if(rawData) {
    getRawData();
  } 
  return true;
}

////////////////////
//////////////////////////////////////// RAW ////////////////////////////////////////
////////////////////


void MuCa::useRawData(bool useRaw) {
    rawData = useRaw;
    useInterrupt = false;
    if(isInit && useRaw) {
      Wire.beginTransmission(I2C_ADDRESS);
      Wire.write(byte(MODE_TEST));
      Wire.write(byte(0x00));
      Wire.endTransmission(I2C_ADDRESS);
      //Serial.println("[Muca] Set TEST mode");
      messages_ble = "[Muca] Set TEST mode";
  }
}

void MuCa::getRawData() {
  messages_ble = "reading data...";
  rawData = true;

  // Start scan //TODO : pas sur qu'on en a besoin
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(byte(0x00));
  Wire.write(byte(0xc0));
  Wire.endTransmission();


  //Wait for scan complete
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(byte(0x00));
  Wire.endTransmission();
  int reading = 0;
  while (1) {
    Wire.requestFrom(I2C_ADDRESS, 1);
    if (Wire.available()) {
      reading = Wire.read();
      int high_bit = (reading & (1 << 7));
      if (high_bit == 0) {
        break;
      }
    }
  }
// Read Data
  for (unsigned int rowAddr = 0; rowAddr < NUM_ROWS; rowAddr++) {

    byte result[2  * NUM_COLUMNS];

    //Start transmission
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(byte(0x01));
    Wire.write(rowAddr);
    unsigned int st = Wire.endTransmission();
    if (st != 0) messages_ble = "I2C write failed";

    delayMicroseconds(50);
    //  delayMicroseconds(50); // Wait at least 100us
    //delay(10);


    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(0x10); // The address of the first column is 0x10 (16 in decimal).
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_ADDRESS, 2 * NUM_COLUMNS, false); // TODO : false was added IDK why
    unsigned int g = 0;
    while (Wire.available()) {
      result[g++] = Wire.read();
    }


    for (unsigned int col = 0; col < NUM_COLUMNS; col++) {
      unsigned  int output = (result[2 * col] << 8) | (result[2 * col + 1]);
      grid[(rowAddr * NUM_COLUMNS) +  NUM_COLUMNS - col - 1] = output;
    }

  } // End foreachrow
  Wire.endTransmission();
}

//https://www.buydisplay.com/download/ic/FT5206.pdf + https://github.com/optisimon/ft5406-capacitive-touch/blob/master/CapacitanceVisualizer/FT5406.hpp
// https://github.com/hyvapetteri/touchscreen-cardiography + http://optisimon.com/raspberrypi/touch/ft5406/2016/07/13/raspberry-pi-7-inch-touchscreen-hacking/
//https://www.newhavendisplay.com/app_notes/FT5x16.pdf + https://www.newhavendisplay.com/appnotes/datasheets/touchpanel/FT5x16_registers.pdf
//https://github.com/azzazza/patch_kernel_q415/blob/master/drivers/input/touchscreen/ft5x06_ts.c
