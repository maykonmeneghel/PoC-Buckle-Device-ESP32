/*
Wi-Fi Fall Detector Buckle for the elderly people

   This code was written by Maykon Meneghel. This is a buckle device that detects Wi-Fi drops.
   You can freely use this code, provided you mention it.

   Suggested citation:
   Meneghel, M.C., 2018.

For questions: maykon.meneghel@pucpr.br
*/

//---LIBRARYS---------------------------------------------------------------------------------------------------------------------------------------
#include <ESP8266WiFi.h>      // https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/src/ESP8266WiFi.h
#include <ESP8266mDNS.h>      // https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266mDNS
#include <WiFiUdp.h>          // https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/src/WiFiUdp.h
#include <Wire.h>             // https://github.com/esp8266/Arduino/blob/master/libraries/Wire/Wire.h
#include <MySQL_Connection.h> // https://github.com/ChuckBell/MySQL_Connector_Arduino/blob/master/src/MySQL_Connection.h
#include <MySQL_Cursor.h>     // https://github.com/ChuckBell/MySQL_Connector_Arduino/blob/master/src/MySQL_Cursor.h
#include <ArduinoOTA.h>       // https://github.com/esp8266/Arduino/tree/master/libraries/ArduinoOTA

//---GLOBAL VARIABLES--------------------------------------------------------------------------------------------------------------------------------
const int MPU_ADDR =      0x68; // MPU6050 sensor address setting (0x68)
const int WHO_AM_I =      0x75; // registro de identificação do dispositivo
const int PWR_MGMT_1 =    0x6B; // registro de configuração do gerenciamento de energia
const int GYRO_CONFIG =   0x1B; // register of the gyro configuration
const int ACCEL_CONFIG =  0x1C; // register of the accelerometer configuration
const int ACCEL_XOUT =    0x3B; // register of the x-axis reader of the accelerometer
const int sda_pin = 4;  // SDA pin
const int scl_pin = 5;  // SCL pin
const int PIN_INT = 2;  // movment detection interrupt pin
const int PIN_AD0 = 12; // MPU address pin (0 -> 0x68 | 1 -> 0x69)
const int LED     = 14; // LED pin
const int PIN_RST = 16; // Reset pin
const char* SSID = "your_ssid";           // Known SSID   
const char* PASSWORD = "your_password";   // Known Password
char user[] = "your_username_mysql_database";         // MySQL user login username
char pass_DB[] = "your_password_mysql_database";      // MySQL user login password
IPAddress server_addr(XXX,XXX,XXX,XX);                // Change XXX for the database server address
WiFiClient client;
MySQL_Connection conn((Client *)&client);
MySQL_Cursor* cursor;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, Tmp2;        // variables to store the accelerometer data

//---I2C COMUNNICATION----------------------------------------------------------------------------------------------------------------------------------
void initI2C() 
{
  Wire.begin(sda_pin, scl_pin);
  initMPU();
}
void findMPU(int mpu_addr)
{
  Wire.beginTransmission(MPU_ADDR);
  int data = Wire.endTransmission(true);

  if(data == 0)
  {
    Serial.print("Dispositivo encontrado no endereco: 0x");
    Serial.println(MPU_ADDR, HEX);
  }
  else 
  {
    Serial.println("Dispositivo nao encontrado!");
  }
}
void checkMPU(int mpu_addr)
{
  findMPU(MPU_ADDR);
    
  int data = readRegMPU(WHO_AM_I); // Register 117 – Who Am I - 0x75
  
  if(data == 104) 
  {
    Serial.println("MPU6050 Dispositivo respondeu OK! (104)");

    data = readRegMPU(PWR_MGMT_1); // Register 107 – Power Management 1-0x6B

    if(data == 64) Serial.println("MPU6050 em modo SLEEP! (64)");
    else Serial.println("MPU6050 em modo ACTIVE!"); 
  }
  else Serial.println("Verifique dispositivo - MPU6050 NAO disponivel!");
}
void initMPU()
{
  digitalWrite(PIN_AD0,LOW);
  setSleepOff();
  setGyroScale();
  setAccelScale();
  checkMPU(MPU_ADDR);
}
void setSleepOff()
{
  writeRegMPU(PWR_MGMT_1, 0); // escreve 0 no registro de gerenciamento de energia(0x68), colocando o sensor em o modo ACTIVE
}
//------------------------------------------------------------------------------------------------------------------
/* function to configure the gyro scales
    gyroscope scale log: 0x1B[4:3]
   0 é 250°/s
   
    FS_SEL  Full Scale Range
      0        ± 250 °/s      0b00000000
      1        ± 500 °/s      0b00001000
      2        ± 1000 °/s     0b00010000
      3        ± 2000 °/s     0b00011000
*/
void setGyroScale()
{
  writeRegMPU(GYRO_CONFIG, 0);
}
//------------------------------------------------------------------------------------------------------------------
/* function to configure the accelerometer scales
    accelerometer scale log: 0x1C[4:3]
   0 é 250°/s

    AFS_SEL   Full Scale Range
      0           ± 2g            0b00000000
      1           ± 4g            0b00001000
      2           ± 8g            0b00010000
      3           ± 16g           0b00011000
*/
void setAccelScale()
{
  writeRegMPU(ACCEL_CONFIG, 0);
}
//------------------------------------------------------------------------------------------------------------------
// Function that writes a given value to a given record
void writeRegMPU(int reg, int val)      //accepts a record and a value as a parameter
{
  Wire.beginTransmission(MPU_ADDR);     // initiates communication with the MPU6050 address
  Wire.write(reg);                      // sends the record with which you want to work
  Wire.write(val);                      // write the value in the registry
  Wire.endTransmission(true);           // the transmission ends
}
//------------------------------------------------------------------------------------------------------------------
// Function that reads from a given record
uint8_t readRegMPU(uint8_t reg)        // accepts a parameter record
{
  uint8_t data;
  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                      // sends the record with which you want to work
  Wire.endTransmission(false);          // terminates transmission but continues with I2C open (sends STOP and START)
  Wire.requestFrom(MPU_ADDR, 1);        // configures to receive 1 byte from the record chosen above.
  data = Wire.read();                   // reads the byte and saves
  return data;                          // data returns 
}
//------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------
/* function that reads the sensor data (raw data)
    are 14 bytes in total, they are 2 bytes for each axis and 2 bytes for temperature:

  0x3B 59 ACCEL_XOUT[15:8]
  0x3C 60 ACCEL_XOUT[7:0]
  0x3D 61 ACCEL_YOUT[15:8]
  0x3E 62 ACCEL_YOUT[7:0]
  0x3F 63 ACCEL_ZOUT[15:8]
  0x40 64 ACCEL_ZOUT[7:0]

  0x41 65 TEMP_OUT[15:8]
  0x42 66 TEMP_OUT[7:0]

  0x43 67 GYRO_XOUT[15:8]
  0x44 68 GYRO_XOUT[7:0]
  0x45 69 GYRO_YOUT[15:8]
  0x46 70 GYRO_YOUT[7:0]
  0x47 71 GYRO_ZOUT[15:8]
  0x48 72 GYRO_ZOUT[7:0]
   
*/
void readRawMPU()
{  
  Wire.beginTransmission(MPU_ADDR);       // initiates communication with the MPU6050 address
  Wire.write(ACCEL_XOUT);                 // sends the record with which you want to work, starting with record 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);            // terminates transmission but continues with I2C open (sends STOP and START)
  Wire.requestFrom(MPU_ADDR, 14);         // configures to receive 14 bytes starting from the register chosen above (0x3B)

  AcX = Wire.read() << 8;                 // reads the most significant byte first
  AcX |= Wire.read();                     // then reads the least significant bit
  AcY = Wire.read() << 8;
  AcY |= Wire.read();
  AcZ = Wire.read() << 8;
  AcZ |= Wire.read();

  Tmp = Wire.read() << 8;
  Tmp |= Wire.read();

  GyX = Wire.read() << 8;
  GyX |= Wire.read();
  GyY = Wire.read() << 8;
  GyY |= Wire.read();
  GyZ = Wire.read() << 8;
  GyZ |= Wire.read(); 

  AcX = (AcX/16384);            
  AcY = (AcY/16384);
  AcZ = (AcZ/16384);
  Tmp = (Tmp/340.00+36.53);
  GyX = (GyX/131);
  GyY = (GyY/131);
  GyZ = (GyZ/131);
  
  Tmp2 = (Tmp/340.00+36.53);
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
//---COMUNICAÇÃO WiFi---------------------------------------------------------------------------------------------------------------------------------
/*
  function that connects the NodeMCU to the Wifi network
  SSID and PASSWORD must be indicated in variables
 */
void reconnectWiFi() 
{
  if(WiFi.status() == WL_CONNECTED)
    return;

  WiFi.begin(SSID, PASSWORD);

  while(WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Successfully connected to the network: ");
  Serial.println(SSID);
  Serial.print("IP obtido: ");
  Serial.println(WiFi.localIP());  
}
//------------------------------------------------------------------------------------------------------------------
void initWiFi()
{
  delay(10);
  Serial.print("Connecting to the network: ");
  Serial.println(SSID);
  Serial.println("Aguarde");
  reconnectWiFi();
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
//---MySQL-----------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------ 
void connSQL() {
 while (conn.connect(server_addr, 3306, user, pass_DB) != true) { //3306 is the default MySQL port
        delay(100);
        Serial.print(".");
    }
    Serial.println("Connected to MySQL server.");
}
//------------------------------------------------------------------------------------------------------------------ 
void recordData() {
  //Serial.println("Gravando Dados.");
  // Initiate the query class instance
  cursor = new MySQL_Cursor(&conn);
  // Execute the query
  char ID[] = "202001";
  char Mssg[] = "01";
  char INSERT_SQL[250];
  sprintf(INSERT_SQL,"INSERT INTO database_table_name (timeStamp, ID, Message) VALUES (current_time, %s,%s)", ID, Mssg);
  //Serial.println(INSERT_SQL);
  cursor->execute(INSERT_SQL);
  // Note: since there are no results, we do not need to read any data
  // Deleting the cursor also frees up memory used
  delete cursor;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
//---Detection Algorithm------------------------------------------------------------------------------------------------------------------------------
bool detect(){
  float mod = sqrt(pow(AcX,2)+pow(AcY,2)+pow(AcZ,2));
  Serial.print("Modulo = ");
  Serial.println(mod);
  if (mod >= 2.3){
    //float roll = atan2(AcY,AcZ) * (180/3.14);
    //float pitch = atan2(AcX, sqrt(pow(AcY,2) + pow(AcZ,2))) * (180/3.14);
    float ang = atan2(sqrt(pow(AcY,2)+pow(AcZ,2)),AcX)*(180/3.14);
    Serial.print("Angulo = ");
    Serial.println(ang);
    if (ang >= 60){
      return true;
    }else{
      return false;
    }
  } else {
    return false;
  }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  pinMode(PIN_INT, INPUT);
  pinMode(PIN_AD0, OUTPUT);
  pinMode(PIN_RST,OUTPUT);
  Serial.begin(115200);
  Serial.println("Configuring the MPU6050");
  initI2C();
  Serial.println("Connecting to the internet");
  initWiFi(); 
  Serial.println("Connecting to the SQL database");
  connSQL();
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Finished settings!");
}
//------------------------------------------------------------------------------------------------------------------
void loop() {
  ArduinoOTA.handle();
  readRawMPU();
  if(detect()){
    recordData();
  }
  yield();  
}
//------------------------------------------------------------------------------------------------------------------
