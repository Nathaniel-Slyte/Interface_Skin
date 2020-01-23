//https://www.newhavendisplay.com/appnotes/datasheets/touchpanel/FT5x16_registers.pdf
//https://www.buydisplay.com/download/ic/FT5206.pdf
#include "Wire.h"

#define I2C_ADDRESS       0x38

#define MODE_TEST         0x40

// RAW
#define NUM_ROWS          21 // 21
#define NUM_COLUMNS       12

#define CALIBRATION_MAX   3
#define CALIB_THRESHOLD   0

//// ============================== CLASS ==============================

class MuCa {
  public:
    MuCa();
    void init(bool raw = false);

    bool poll();

    bool updated();

    //RAW
    void pollRaw();
    bool useRaw = false;
    short grid[NUM_ROWS * NUM_COLUMNS];

    // manual callibration
    void calibrate();
    void setGain(int val);


  private:
    bool isInit = false;

    //RAW
    void getRawData();
    short calibrateGrid[NUM_ROWS * NUM_COLUMNS];
    int calibrationSteps = 0;
};

////  ============================== INITIALIZATION ==============================

MuCa::MuCa() {}

void MuCa::init(bool raw = false) {
  useRaw = raw;
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);

  Wire.begin();
  Wire.setClock(400000); // 400000 https://www.arduino.cc/en/Reference/WireSetClock

  // Initialization
  if (useRaw) {
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(byte(0x00));
    Wire.write(byte(MODE_TEST));
    Wire.endTransmission(I2C_ADDRESS);
   }

  delay(100);
  Serial.println("MuCa initialized");
  delay(100);
  isInit = true;
}


//// ============================== UPDATE ==============================


bool MuCa::updated() {
 if (!isInit) return false;
 poll();
 if (useRaw) return true;
}

bool MuCa::poll() {
 if (useRaw) {
   getRawData();
 }
}

//// ============================== RAW ==============================
//void MuCa::unsureTestMode() { }


void MuCa::getRawData() {

  int startTime = millis();

  // Start scan
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(byte(0x00));
  Wire.write(byte(0xc0));
  Wire.endTransmission();


  ////////////////////////////// Serial.print("startread:");  int tt = millis();  Serial.print(tt);
  // Read Data
  for (unsigned int rowAddr = 0; rowAddr < NUM_ROWS; rowAddr++) {

    byte result[2  * NUM_COLUMNS];

    //Start transmission
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(byte(0x01));
    Wire.write(rowAddr);
    unsigned int st = Wire.endTransmission();
    if (st < 0) Serial.print("i2c write failed");

    delayMicroseconds(50);
    //  delayMicroseconds(50); // Wait at least 100us
    //delay(10);



    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(byte(16)); // The address of the first column is 0x10 (16 in decimal).
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_ADDRESS, 2 * NUM_COLUMNS, false); // TODO : falst was added IDK why
    unsigned int g = 0;
    while (Wire.available()) {
      result[g++] = Wire.read();
    }


    for (unsigned int col = 0; col < NUM_COLUMNS; col++) {
      unsigned  int output = (result[2 * col] << 8) | (result[2 * col + 1]);
      if (calibrationSteps == CALIBRATION_MAX) {
        grid[(rowAddr * NUM_COLUMNS) +  NUM_COLUMNS - col - 1] = CALIB_THRESHOLD + output - calibrateGrid[(rowAddr * NUM_COLUMNS) +  NUM_COLUMNS - col - 1];
      } else {
        calibrateGrid[(rowAddr * NUM_COLUMNS) +  NUM_COLUMNS - col - 1] = output;
        grid[(rowAddr * NUM_COLUMNS) +  NUM_COLUMNS - col - 1] = output;
      }
    }


  } // End foreachrow
  ////////////////////////////// Serial.print("end:"); Serial.println(millis() - tt);

  // Calibration
  if (calibrationSteps != CALIBRATION_MAX) {
    if (grid[0] < 5000) return;
    if (calibrationSteps == 0) {
      memcpy(calibrateGrid, grid, sizeof(grid));
    } else {
      for (int i = 0; i < (NUM_ROWS * NUM_COLUMNS); i++) {
        // calibrateGrid[i] = (calibrateGrid[i] & grid[i]) + ((calibrateGrid[i] ^ grid[i]) >> 1);
        calibrateGrid[i] = (calibrateGrid[i] + grid[i]) / 2;
      }
    }
    Serial.println("Calibrate");
    calibrationSteps++;
  }

}


//// ============================== CALIBRATION ==============================




void MuCa::calibrate() {
  calibrationSteps = 0;
}

void MuCa::setGain(int gain) {
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(byte(0x07));
  Wire.write(byte(gain));
  Wire.endTransmission();
}


//https://www.buydisplay.com/download/ic/FT5206.pdf + https://github.com/optisimon/ft5406-capacitive-touch/blob/master/CapacitanceVisualizer/FT5406.hpp
// https://github.com/hyvapetteri/touchscreen-cardiography + http://optisimon.com/raspberrypi/touch/ft5406/2016/07/13/raspberry-pi-7-inch-touchscreen-hacking/
//https://www.newhavendisplay.com/app_notes/FT5x16.pdf + https://www.newhavendisplay.com/appnotes/datasheets/touchpanel/FT5x16_registers.pdf
//https://github.com/azzazza/patch_kernel_q415/blob/master/drivers/input/touchscreen/ft5x06_ts.c
