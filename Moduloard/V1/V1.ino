#if (RAMEND < 1000)
  #define SERIAL_BUFFER_SIZE 16
#else
  #define SERIAL_BUFFER_SIZE 64
#endif

#include "MuCa_firmware_raw.h"
#include <string.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "Wire.h"
int test = 0;

MuCa muca;

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
  
  
  //Serial.begin(2000000);

  muca.init();
  muca.calibrate();
  // muca.setGain(100);
  
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
  int total = 0;
  bleuart.println("Here");
  bleuart.println(muca.message_ble);
  muca.getRawData();
  String str = "";
  for (int i = 0; i < ROWS_USE * NUM_COLUMNS; i++) {
      if (abs(muca.grid[i]) > 0) {
        str += muca.grid[i];
        total += abs(muca.grid[i]);
      }
      if (i != ROWS_USE * NUM_COLUMNS - 1)
        str += ",";
    }
   ButcherStr(str);
   bleuart.println();
   bleuart.println(total);
   bleuart.println();
  //GetRaw();
  
  /*
  calibrate_count = 0;
  //muca.calibrate(0.1);

  while ( calibrate_count < 10){
    GetRaw();
    calibrate_count ++;
  }
  
  while ( bleuart.available() ){
    char ch;
    ch =  bleuart.read();
    delay(500);
    if (ch == 'p')
    {
      bleuart.println("\n Data ! \n");
      GetRaw();
    }
  }
  */
  delay(3000);

}


void GetRaw() {
  if (muca.updated()) {
  
   for (int i = 0; i < ROWS_USE * NUM_COLUMNS; i++) {
      if (muca.grid[i] > 0) bleuart.print(muca.grid[i]);
      if (i != ROWS_USE * NUM_COLUMNS - 1)
        bleuart.print(",");
    }
   bleuart.println();

  }

  delay(1);
}



/*

int frameCount = 0;
float fps = 0.0F;
float t = 0.0F;
float prevtt = 0.0F;

void GetFPS()
{
  frameCount++;
  t += millis() - prevtt;
  if (t > 1000.0f)
  {
    fps = frameCount;
    frameCount = 0;
    t = 0;
  }
  prevtt = millis();
  //Serial.println(fps);
}
*/
