#include <bluefruit.h>
#include "LSM6DS3.h"
#include "Wire.h"


//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);  //I2C device address 0x6A

#define CONVERT_G_TO_MS2 9.80665f
float FREQUENCY_HZ = 1;
float INTERVAL_MS = (1000 / (FREQUENCY_HZ + 1));

static unsigned long last_interval_ms = 0;
float prev_z = 0 ;

// max concurrent connections supported by this example
#define MAX_PRPH_CONNECTION   1
uint8_t connection_count = 1;

typedef union
{
  float f_num;
  uint8_t f_bytes[4];
  char f_char[4];
} FLOATUNION;

FLOATUNION u_accelx,u_accely,u_accelz;
char byte_array[12];

 // ACCELEROMETERSERVICE_SERVICE_UUID =      "E95D0753251D470AA062FA1922DFA9A8";
 // ACCELEROMETERDATA_CHARACTERISTIC_UUID =  "E95DCA4B251D470AA062FA1922DFA9A8";
// ACCELEROMETERPERIOD_CHARACTERISTIC_UUID = "E95DFB24251D470AA062FA1922DFA9A8";


const uint8_t ACCEL_UUID_SERVICE[] =
{
  0xA8, 0xA9, 0xDF, 0x22, 0x19 ,0xFA ,        // FA1922DFA9A8
  0x62, 0xA0,             // A062
  0x0A, 0x47,             // 470A
  0x1D, 0x25,             // 251D
  0x53, 0x07, 0x5D, 0xE9  // E95D0753
};


const uint8_t ACC_UUID_CHR[] =
{
   0xA8, 0xA9, 0xDF, 0x22, 0x19 ,0xFA,         // FA1922DFA9A8
  0x62, 0xA0,             // A062
  0x0A, 0x47,             // 470A
  0x1D, 0x25,             // 251D
  0x4B, 0xCA, 0x5D, 0xE9 // E95DCA4B
 
};

const uint8_t FRQ_UUID_CHR[] =
{
  0xA8, 0xA9, 0xDF, 0x22, 0x19 ,0xFA,         // FA1922DFA9A8
  0x62, 0xA0,             // A062
  0x0A, 0x47,             // 470A
  0x1D, 0x25,             // 251D
  0x24, 0xFB, 0x5D, 0xE9, // E95DFB24
};

BLEService        accel_service(ACCEL_UUID_SERVICE);
BLECharacteristic accel_chr(ACC_UUID_CHR);
BLECharacteristic freq_chr(FRQ_UUID_CHR);

void setup()
{

    Serial.begin(115200);
  while (!Serial)
    ;

  if (myIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
    Serial.println("Device OK!");
  }

  Serial.println("Amiga");
  Serial.println("------------------------------\n");

  // Initialize Bluefruit with max concurrent connections as Peripheral = MAX_PRPH_CONNECTION, Central = 0
  Serial.println("Initialising the module");
  Bluefruit.begin(MAX_PRPH_CONNECTION, 0);
  Bluefruit.setName("Amiga");
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!
  accel_service.begin();


  accel_chr.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  accel_chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  accel_chr.setFixedLen(12);
  accel_chr.begin();
 
  freq_chr.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  freq_chr.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  freq_chr.setFixedLen(1);
  freq_chr.begin();
  freq_chr.write8(0x01); // led = on when connected

  freq_chr.setWriteCallback(freq_write_callback);

  // Setup the advertising packet(s)
  Serial.println("Setting up the advertising");
  startAdv();
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include HRM Service UUID
  Bluefruit.Advertising.addService(accel_service);

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

void setFrequency(int frq)
{
 
FREQUENCY_HZ = frq;
INTERVAL_MS = (1000 / (FREQUENCY_HZ + 1));
last_interval_ms = 0;
    Serial.println(" Frequency Changed !"); 
}

void freq_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  (void) conn_hdl;
  (void) chr;
  (void) len; // len should be 1
       
    Serial.println("   Data Recived !"); 
    Serial.println(len); 
    Serial.println(String(data[0])); 
    setFrequency(data[0]);
}

float formatFloat(float myFloat){
   return roundf((myFloat*CONVERT_G_TO_MS2) * 1000)  / 1000;
}

void loop()
{
    if (millis() > last_interval_ms + INTERVAL_MS) {
    last_interval_ms = millis();

     u_accelx.f_num = formatFloat(myIMU.readFloatAccelX()) ;
    u_accely.f_num =  formatFloat(myIMU.readFloatAccelY()); 
    u_accelz.f_num =  formatFloat(myIMU.readFloatAccelZ());

    Serial.print(u_accelx.f_num);
    Serial.print('\t');
    Serial.print(u_accely.f_num);
    Serial.print('\t');
    Serial.println(u_accelz.f_num);
    

    //String acc_data = String(accelx)+","+String(accely)+","+String(accelz);
    // if(accelx > 5 && accely < 2){
    //  float mod = accelz - prev_z;
    //   prev_z = accelz; 
    //   if(mod > 0){
    //     Serial.print(" Breath In ");
    //   }else{
    //     Serial.print(" Breath Out ");
    //   };
    // }

        // notify all connected clients
    for (uint16_t conn_hdl=0; conn_hdl < MAX_PRPH_CONNECTION; conn_hdl++)
    {
      if ( Bluefruit.connected(conn_hdl) && accel_chr.notifyEnabled(conn_hdl) )
      {
          strcpy(byte_array, u_accelx.f_char);
          strcat(byte_array, u_accely.f_char);
          strcat(byte_array, u_accelz.f_char);
          accel_chr.notify(conn_hdl,byte_array);
      }      
    }
  }   
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  (void) conn_handle;

  connection_count++;
  Serial.print("Connection count: ");
  Serial.println(connection_count);

  // Keep advertising if not reaching max
  if (connection_count < MAX_PRPH_CONNECTION)
  {
    Serial.println("Keep advertising");
    Bluefruit.Advertising.start(0);
  }
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

  connection_count--;
}