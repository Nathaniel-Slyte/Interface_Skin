
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

MuCa muca;

#define CALIBRATION_STEPS 5
short currentCalibrationStep = 0;
unsigned int calibrationGrid[NUM_ROWS * NUM_COLUMNS];

// BLE Service
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

void setup()
{

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("AAA");
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
    //bleuart.println(muca.grid[0]);
    //bleuart.println(muca.grid[42]);

  }
}
