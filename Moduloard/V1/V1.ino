#if (RAMEND < 1000)
  #define SERIAL_BUFFER_SIZE 16
#else
  #define SERIAL_BUFFER_SIZE 64
#endif

#include <Muca.h>
#include <string.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "Wire.h"

// gain 0 -> 31

Muca muca;

BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

//int calibrate_count = 0;

String ID = "V1";

void setup() {
  
  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("V1");
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  
  bledfu.begin();

  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  bleuart.begin();

  blebas.begin();
  blebas.write(100);

  startAdv();

  muca.init();
  muca.useRawData(true); // If you use the raw data, the interrupt is not working

  delay(50);
  muca.setGain(2);
}









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

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
}










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



void loop() {
  if (muca.updated()) {
    SendRawByte(); // Faster
  }
  delay(16); // waiting 16ms for 60fps

  delay(300);

}

void SendRawByte() {
  // The array is composed of 254 bytes. The two first for the minimum, the 252 others for the values.
  // HIGH byte minimum | LOW byte minimum  | value 1

  unsigned int minimum = 80000;
  uint8_t rawByteValues[254];

  for (int i = 0; i < NUM_ROWS * NUM_COLUMNS; i++) {
  if (muca.grid[i] > 0 && minimum > muca.grid[i])  {
      minimum = muca.grid[i]; // The +30 is to be sure it's positive
    }
  }
  rawByteValues[0] = highByte(minimum);
  rawByteValues[1] = lowByte(minimum);


  for (int i = 0; i < NUM_ROWS * NUM_COLUMNS; i++) {
    rawByteValues[i + 2] = muca.grid[i] - minimum;
    // Serial.print(rawByteValues[i+2]);
    //  Serial.print(",");

  }
  // Serial.println();
  //GetFPS();
   bleuart.write(rawByteValues, 254);
   //Serial.flush();
  //Serial.println();
}
