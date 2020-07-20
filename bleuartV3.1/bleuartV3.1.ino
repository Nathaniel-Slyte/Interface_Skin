
#if (RAMEND < 1000)
  #define SERIAL_BUFFER_SIZE 16
#else
  #define SERIAL_BUFFER_SIZE 64
#endif



#include <string.h>

#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

#include <Wire.h>
#include "MuCa_firmware_raw.h"

// MuCa
MuCa muca;
#define CALIBRATION_STEPS 5
short currentCalibrationStep = 0;
unsigned int calibrationGrid[NUM_ROWS * NUM_COLUMNS];

// Gyro
const int MPU_addr = 0x68;  // I2C address of the MPU-6050
//int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float Tmp;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;


// BLE Service
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

void setup()
{

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("BBB");
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();

/////////////////////////

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  /*
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  */
  // Call this function if you need to get the IMU error values for your module
  //calculate_IMU_error();
  //delay(20);

/////////////////////////

  // MUCA init 
  muca.init(false);
  muca.useRawData(true);
}

////////////////////
//////////////////////////////////////// BLE ////////////////////////////////////////
////////////////////

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  //Serial.print("Connected to ");
  //Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  //Serial.println();
  //Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}

////////////////////
//////////////////////////////////////// COMMUNICATION ////////////////////////////////////////
////////////////////

void ButcherStr(String str){
  String separator = ",";
  int counter = 0;
  
  int pos = 0;
  String token = "";
  while ((pos = str.indexOf(',')) != -1) {
    token += str.substring(0, pos) + ",";
    str.remove(0, pos + separator.length());
    counter++;
    
    if (counter == 4  || (pos == -1 && token != "" )){
      bleuart.print(token);
      token = "";
      counter = 0;
    } 
  }
}

////////////////////
//////////////////////////////////////// MUCA ////////////////////////////////////////
////////////////////

void GetRaw() {
   if (muca.updated()) {
    if (currentCalibrationStep >= CALIBRATION_STEPS) {
      String str = "";
      for (int i = 0; i < NUM_ROWS; i++) {
        for (int j = 0; j < NUM_COLUMNS; j++) {
          if (muca.grid[i*NUM_ROWS + j] > 0) str += (muca.grid[i*NUM_ROWS + j] /*- calibrationGrid[i*NUM_ROWS + j]*/); // la calibration est a revoir
          if (i*j != NUM_ROWS * NUM_COLUMNS - 1) str += ",";
          else str += ";";
        }
        ButcherStr(str);
        bleuart.print("\n");
        str = "";
      }
    }
    else { // Once the calibration is done
    //Save the grid value to the calibration array
     for (int i = 0; i < NUM_ROWS * NUM_COLUMNS; i++) {
      if (currentCalibrationStep == 0) calibrationGrid[i] = muca.grid[i]; // Copy array
      else calibrationGrid[i] = (calibrationGrid[i] + muca.grid[i]) / 2 ; // Get average
     }
       currentCalibrationStep++;
       bleuart.print("Calibration performed "); bleuart.print(currentCalibrationStep); bleuart.print("/"); bleuart.print(CALIBRATION_STEPS);
    }
  }
}


////////////////////
//////////////////////////////////////// Gyroscope ////////////////////////////////////////
////////////////////

void GetDataGyro(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AccX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AccY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AccZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyroX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyroY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyroZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Wire.endTransmission();
}



////////////////////
//////////////////////////////////////// LOOP ////////////////////////////////////////
////////////////////


void loop(){
  
  while ( bleuart.available() ){
    char ch;
    ch =  bleuart.read();
    bleuart.println("Start !");
    //bleuart.println(ch);
    
    muca.getRawData();
    GetRaw();
    bleuart.println(muca.messages_ble);
    
    GetDataGyro();

    bleuart.print("AccX = "); bleuart.print(AccX);
    bleuart.print(" | AccY = "); bleuart.print(AccY);
    bleuart.print(" | AccZ = "); bleuart.print(AccZ);
    bleuart.print(" | Tmp = "); bleuart.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
    bleuart.print(" | GyroX = "); bleuart.print(GyroX);
    bleuart.print(" | GyroY = "); bleuart.print(GyroY);
    bleuart.print(" | GyroZ = "); bleuart.println(GyroZ);

    delay(333);
    
  }
}
