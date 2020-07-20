

#if (RAMEND < 1000)
  #define SERIAL_BUFFER_SIZE 16
#else
  #define SERIAL_BUFFER_SIZE 64
#endif

#include "MuCa_firmware_raw.h"
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#define nord 24
#define sud 7
#define est 14
#define ouest 17
String  ID  = "V3";
//MuCa muca;
// BLE Service
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

void setup()
{
  
  //digitalWrite(nord,HIGH);
  Serial.begin(115200);
//  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  
  Serial.println("Bluefruit52 BLEUART Example");
  Serial.println("---------------------------\n");

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  //Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("V3");
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

  Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
  Serial.println("Once connected, enter character(s) that you wish to send");
 /* muca.init(false); // useInterrupt ne fonctionne pas bien
  muca.useRaw = true;
  muca.calibrate(1.0);*/
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

void loop()
{

  while ( bleuart.available() )
  {
    char ch;
    ch = bleuart.read();
    if (ch == 'n')
    {
      AllumeNord();
      bleuart.println("n");
    }
    if (ch == 's')
    {
      AllumeSud();
      bleuart.println("s");
    }
    if (ch == 'e')
    {
      AllumeEst();
      bleuart.println("e");
    }
    if (ch == 'o')
    {
      AllumeOuest();
      bleuart.println("o");
    }
    if (ch == 'r')
    {
      String s = AllInputRead();
      bleuart.println(s);
    }
    if (ch == 't')
    {
      Eteindre();
      bleuart.println("stop "+ ID + " ");
    }
  /*  if (ch == 'p')
    {
      String u ="";// ID+ ";";
      //u += "valGyro;";
      u+= GetRaw()+";";
      bleuart.println(u);
    }*/
  }
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
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

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}
void AllumeNord(){
  Eteindre();
  pinMode(nord,OUTPUT);
  digitalWrite(nord, HIGH);
  }
void AllumeSud(){
  Eteindre();
  pinMode(sud,OUTPUT);
  digitalWrite(sud, HIGH);
  }
void AllumeEst(){
  Eteindre();
  pinMode(est,OUTPUT);
  digitalWrite(est, HIGH);
  }
void AllumeOuest(){
  Eteindre();
  pinMode(ouest,OUTPUT);
  digitalWrite(ouest, HIGH);
  }

void Eteindre(){
  pinMode(nord,OUTPUT);
  pinMode(sud,OUTPUT);
  pinMode(est,OUTPUT);
  pinMode(ouest,OUTPUT);
  digitalWrite(nord, LOW);
  digitalWrite(sud, LOW);
  digitalWrite(est, LOW);
  digitalWrite(ouest, LOW);
  }
String AllInputRead(){
  pinMode(nord,INPUT);
  pinMode(sud,INPUT);
  pinMode(est,INPUT);
  pinMode(ouest,INPUT);
  String ret = "";
  ret = ID+",";
  ret += digitalRead(nord);
  ret +=",";
  ret += digitalRead(est);
  ret +=",";
  ret += digitalRead(sud);
  ret +=",";
  ret += digitalRead(ouest);
  ret += ";";
  return ret;
  }

/*
String GetRaw() {
  String sFt="";
  if (muca.updated()) {
   for (int i = 0; i < ROWS_USE * NUM_COLUMNS; i++) {
      if (muca.grid[i] > 0)sFt+=muca.grid[i];
      if (i != ROWS_USE * NUM_COLUMNS - 1)
        sFt+=",";//Serial.print(",");
    }
  }
  return sFt;
}*/
