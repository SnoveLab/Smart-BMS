#define VERSION 1
#define TTS 600000 //10 minutes time to sleep if not moving
#define SPEEDMIN 5 //5kmph is considered moving vehicle

esp_sleep_wakeup_cause_t wakeup_reason;
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  120        /* Time ESP32 will go to sleep (in seconds) */
#define TIME_ALIVE 25

RTC_DATA_ATTR unsigned long millisOffset=0;
RTC_DATA_ATTR unsigned long lastMoveSpeed=0;
RTC_DATA_ATTR unsigned long lastUpload=0;
RTC_DATA_ATTR bool bFirstTurnON=1;

bool isReadyToSleep=0;
bool isReadyToSleepResponse=1;
bool isReadyToSleepOTA=1;


#include "pinout.h"
#include <SSLClient.h>
#include "trustAnchor.h"
#include <ModbusMaster.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// instantiate ModbusMaster object
ModbusMaster node;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial gpsSerial(GPS_TX, GPS_RX);


//GPRS Module
// #define TINY_GSM_MODEM_SIM800
#define TINY_GSM_MODEM_A76XXSSL //Support A7670X/A7608X/SIM7670G
#define TINY_GSM_RX_BUFFER          1024 // Set RX buffer to 1Kb

// #define DUMP_AT_COMMANDS
#include <TinyGsmClient.h>
#define SerialAT Serial2
#define SerialMon Serial
#define TINY_GSM_DEBUG Serial

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif

#ifdef TINY_GSM_MODEM_A76XXSSL
TinyGsmClient client(modem);
#endif

#ifdef TINY_GSM_MODEM_SIM800

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

TinyGsmClient client(modem);
SSLClient clientSecure(client, TAs, (size_t)TAs_NUM, RANDOMPIN);
#include <ArduinoHttpClient.h>
const char serverOTA[]   = "firebasestorage.googleapis.com";
const char resourceOTA[] = "/v0/b/snovelab.firebasestorage.app/o/SmartBMS_V0_4_2G.ino.bin?alt=media";
const int  portOTA       = 443;
HttpClient          httpOTA(clientSecure, serverOTA, portOTA);
#endif

uint32_t lastReconnectAttempt = 0;
// Your GPRS credentials, if any
const char apn[]      = "internet";

#include <PubSubClient.h>
PubSubClient  pubsubClient;
const char *broker = "202.10.42.81";
char *client_pass ="nouvelab";

#include <ArduinoJson.h>

JsonDocument docReceived;

#include "EEPROM.h"
#define EEPROM_SIZE 128

// MQTT details
char clientId[128];
char topicRealTimeStatus[64];
char topicReadParameterSetting[64];
char topicSendParameterSetting[64];
char topicUpdateParameterSetting[64];
char topicUpdateSettingDone[64];
char topicUpdatePower[64];
char topicUpdatePowerDone[64];

//Data from MQTT
int MOSChargePower=0;
int MOSDischargePower=0;
int RequestDoneFlagUpdatePower=0;
unsigned long PeriodicMQTT=30000;
int newVersion=VERSION;
int RequestDoneFlagUpdateParameter=0;

//Flag for MQTT
int publishUpdatePower=0;
int publishUpdateParameter=0;
int publishReadParameter=0;


//Device ID
uint64_t chipID = 0;

time_t unixTime=0;

//Data from BMS
uint16_t totalVolt=0;
uint16_t SoC=0;
int32_t Ampere=0;
uint16_t StatusOnOff=0;
uint16_t Alarm1=0;
uint16_t Alarm2=0;
uint16_t Alarm3=0;
uint16_t Alarm4=0;
uint16_t Alarm5=0;
uint16_t Alarm6=0;
uint16_t BuzzerStatus=0;
uint16_t SensorQty=0;
int32_t Temperature[8];
uint16_t MOSCharge=0;
uint16_t MOSDischarge=0;
uint16_t Balance=0;
uint16_t VoltPerSeries[8];
uint16_t MaxMinVoltAndPos[4];
uint16_t VoltDifference=0;

//Data from GPS
double Lat=0;
double Long=0;
char chipIDHex[32];

void setup()
{
  #ifdef TINY_GSM_MODEM_SIM800
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  #endif
  // GPS turn on pin
  pinMode(GPS_EN,OUTPUT);
  digitalWrite(GPS_EN,HIGH);

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.begin(115200);
  
  chipID=ESP.getEfuseMac();
  sprintf(chipIDHex,"%llX",chipID);
  Serial.println(chipIDHex);

  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("Failed to initialize EEPROM");
    ESP.restart();
  }

  EEPROM.get(0, PeriodicMQTT);


  xTaskCreatePinnedToCore(
    TaskBMS
    ,  "BMS"
    ,  4096  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  1  // Priority
    ,  NULL // With task handle we will be able to manipulate with this task.
    ,  0 // Core on which the task will run
    );

  xTaskCreatePinnedToCore(
    TaskGPS
    ,  "GPS"
    ,  4096  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  2  // Priority
    ,  NULL // With task handle we will be able to manipulate with this task.
    ,  0 // Core on which the task will run
    );

  xTaskCreatePinnedToCore(
    TaskMQTT
    ,  "MQTT"
    ,  10240  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  3  // Priority
    ,  NULL // With task handle we will be able to manipulate with this task.
    ,  1 // Core on which the task will run
    );

  
}

void TaskMQTT(void *pvParameters){  // This is a task.
  JsonDocument doc;
  
  setupGPRS();
  turnOnGPRS();
  vMQTTSetup();
  bMQTTConnect(client_pass);
  for (;;){ // A Task shall never return or exit.
    // Make sure we're still registered on the network
    if (!modem.isNetworkConnected()) {
      SerialMon.println("Network disconnected");
      if (!modem.waitForNetwork(30000L, true)) {
        SerialMon.println(" fail");
        vTaskDelay(10000);
        return;
      }
      if (modem.isNetworkConnected()) {
        SerialMon.println("Network re-connected");
      }
      // and make sure GPRS/EPS is still connected
      if (!modem.isGprsConnected()) {
        SerialMon.println("GPRS disconnected!");
        SerialMon.print(F("Connecting to "));
        SerialMon.print(apn);
        if (!modem.gprsConnect(apn)) {
            SerialMon.println(" fail");
            vTaskDelay(10000);
            return;
        }
        if (modem.isGprsConnected()) {
            SerialMon.println("GPRS reconnected");
        }
      }
    }

    

    if (!pubsubClient.connected()) {
      Serial.print(pubsubClient.state());
      SerialMon.println("=== MQTT NOT CONNECTED ===");
      // Reconnect every 10 seconds
      uint32_t t = millis();
      if (t - lastReconnectAttempt > 10000L) {
          lastReconnectAttempt = t;
          if (bMQTTReconnect(client_pass)) {
              lastReconnectAttempt = 0;
          }
      }
      vTaskDelay(100);
    } else {
      static char buffer[1024];

      //Put to sleep if we don't have to send data on this wake up cycle
      if (((long)(PeriodicMQTT-(millis()+millisOffset-lastUpload))>TIME_TO_SLEEP*1000) && !bFirstTurnON) {
        if (!isReadyToSleep) {
          Serial.printf("Calculated: %lu \t Periodic: %lu \t Elapsed: %lu \t isReadyToSleep:%d \n",PeriodicMQTT-(millis()+millisOffset-lastUpload),PeriodicMQTT,(millis()+millisOffset-lastUpload),isReadyToSleep);
          Serial.println("==READY TO SLEEP WHEN PERIODIC IS STILL MORE THAN TIME TO SLEEP==");
          isReadyToSleep=1;
        }
      } else {
        isReadyToSleep=0;
      }

      if (millis()+millisOffset-lastUpload>PeriodicMQTT || bFirstTurnON) {
        getGSMTIME(&unixTime);

        doc.clear();
        //Insert data to json
        doc["DeviceID"] = chipIDHex;
        doc["CurrentVersion"] = VERSION;
        doc["SignalQuality"] = modem.getSignalQuality();
        doc["PeriodicMQTT"] = PeriodicMQTT/1000;
        doc["Lat"] = Lat;
        doc["Long"] = Long;
        doc["Timestamp"] = unixTime;
        doc["Volt"] = totalVolt/10.0;
        doc["SoC%"] = SoC/10.0;
        doc["Ampere"] = Ampere;

        JsonArray VoltPerSeriesJSON = doc["VoltPerSeries"].to<JsonArray>();
        for (int i=0; i<8 ; i++) {
          VoltPerSeriesJSON.add(VoltPerSeries[i]);
        }

        JsonArray MaxMinVoltJSON = doc["MaxMinVolt"].to<JsonArray>();
        for (int i=0; i<2 ; i++) {
          MaxMinVoltJSON.add(MaxMinVoltAndPos[i*2]);
        }

        JsonArray MaxMinVoltPosJSON = doc["MaxMinVoltPos"].to<JsonArray>();
        for (int i=0; i<2 ; i++) {
          MaxMinVoltPosJSON.add(MaxMinVoltAndPos[(i*2)+1]);
        }

        doc["VoltDifference"] = VoltDifference;
        doc["MOSCharge"] = MOSCharge;
        doc["MOSDischarge"] = MOSDischarge;
        doc["SensorQty"] = SensorQty;

        JsonArray TemperatureJSON = doc["Temperature"].to<JsonArray>();
        for (int i=0; i<SensorQty ; i++) {
          TemperatureJSON.add(Temperature[i]);
        }

        if (Balance==0) {
          doc["Balance"] = "Off";
        } else if (Balance==1) {
          doc["Balance"] = "Passive";
        } else {
          doc["Balance"] = "Active";
        }
        
        

        // Serial.println(Alarm1,BIN);
        // Serial.println(Alarm2,BIN);
        // Serial.println(Alarm3,BIN);
        // Serial.println(Alarm4,BIN);
        // Serial.println(Alarm5,BIN);
        // Serial.println(Alarm6,BIN);

        char tempbuff[17];
        intToArray(tempbuff,Alarm1); 
        doc["Alarm1"] = tempbuff;
        intToArray(tempbuff,Alarm2); 
        doc["Alarm2"] = tempbuff;
        intToArray(tempbuff,Alarm3); 
        doc["Alarm3"] = tempbuff; 
        intToArray(tempbuff,Alarm4); 
        doc["Alarm4"] = tempbuff; 
        intToArray(tempbuff,Alarm5); 
        doc["Alarm5"] = tempbuff; 
        intToArray(tempbuff,Alarm6); 
        doc["Alarm6"] = tempbuff; 

        doc["StatusOnOff"] = (int) (MOSCharge || MOSDischarge);

        doc["BuzzerStatus"] =0;


        serializeJson(doc, buffer);

        Serial.println(buffer);
        pubsubClient.publish(topicRealTimeStatus,buffer);
        lastUpload=millis()+millisOffset;
        bFirstTurnON=0;
        isReadyToSleep=1;
      }
      if (publishUpdatePower) {
        doc.clear();
        doc["DoneFlag"]=(int)!RequestDoneFlagUpdatePower;
        doc["MOSCharge"] = MOSCharge;
        doc["MOSDischarge"] = MOSDischarge;
        serializeJson(doc, buffer);
        pubsubClient.publish(topicUpdatePowerDone,buffer);
        publishUpdatePower=0;
        isReadyToSleepResponse=1;
      }
      //Update Parameter
      if (publishUpdateParameter) {
        doc.clear();
        doc["DoneFlag"]=(int)RequestDoneFlagUpdateParameter;
        doc["PeriodicMQTT"] = PeriodicMQTT/1000;
        doc["newVersion"] = newVersion;
        serializeJson(doc, buffer);
        pubsubClient.publish(topicUpdateSettingDone,buffer);
        publishUpdateParameter=0;
        isReadyToSleepResponse=1;
      }
      //Send Parameter
      if (publishReadParameter) {
        doc.clear();
        doc["PeriodicMQTT"] = PeriodicMQTT/1000;
        doc["newVersion"] = newVersion;
        doc["MOSCharge"] = MOSCharge;
        doc["MOSDischarge"] = MOSDischarge;
        serializeJson(doc, buffer);
        pubsubClient.publish(topicSendParameterSetting,buffer);
        publishReadParameter=0;
        isReadyToSleepResponse=1;
      }
      //Check OTA after publishing feedback
      if (newVersion>VERSION) {
        isReadyToSleepOTA=0;
        vUpdateOTA();
        isReadyToSleepOTA=1;
      } 

      
      pubsubClient.loop();
    }
  }
}

void intToArray (char* output,uint16_t value) {
  for (int i=0; i<16; i++) {
    output[15-i]=((value>>i) & 1) +48;
  }
  output[16]='\0';
  return;
}


void TaskBMS(void *pvParameters){  // This is a task.

  // BMS turn on pin
  pinMode(BMS_ON,OUTPUT);
  // Modbus communication runs at 9600 baud
  Serial1.begin(9600,SERIAL_8N1,BMS_RX,BMS_TX);
  // Modbus slave ID
  node.begin(0x81, Serial1,true,0x51);

  uint16_t data[10];

  for (;;){ // A Task shall never return or exit.
    int retry=0;
    vTurnOnBMS();
    //Get Voltage of each cells
    while((iReadDataBMS (VoltPerSeries, 0x00, 8)!=0) && (retry++<3));
    retry=0;
    //Get current positive for discharging and negative for charging
    while((iReadDataBMS (data, 0x39, 1)!=0) && (retry++<3));
    retry=0;
    Ampere=(data[0]-30000)*0.1;
    //Get total voltage of the battery
    while((iReadDataBMS (data, 0x38, 1)!=0) && (retry++<3));
    retry=0;
    totalVolt=data[0];
    //Get SOC
    while((iReadDataBMS (data, 0x3A, 1)!=0) && (retry++<3));
    retry=0;
    SoC=data[0];
    //Get MaxMin Battery Voltage and position
    while((iReadDataBMS (MaxMinVoltAndPos,0x3E,4)!=0) && (retry++<3));
    retry=0;
    
    //Get Voltage Difference
    while((iReadDataBMS (data,0x42,1)!=0) && (retry++<3));
    retry=0;
    VoltDifference=data[0];

    //Get MOS Charge and Discharge  Status
    while((iReadDataBMS (data,0x52,2)!=0) && (retry++<3));
    retry=0;
    MOSCharge=data[0];
    MOSDischarge=data[1];

    //Get Sensor Qty Temperature
    while((iReadDataBMS (data,0x3D,1)!=0) && (retry++<3));
    retry=0;
    SensorQty=data[0];

    //Get Temperature
    while((iReadDataBMS (data,0x30,SensorQty)!=0) && (retry++<3));
    retry=0;
    for (int i=0;i<SensorQty;i++) {
      Temperature[i]=data[i]-40;
    }


    //Get Balance  Status 0 Off, 1 Passive, 2 Active 
    while((iReadDataBMS (data,0x4D,1)!=0) && (retry++<3));
    retry=0;
    Balance=data[0];

    //Get Alarm
    while((iReadDataBMS (data,0x6D,1)!=0) && (retry++<3));
    retry=0;
    Alarm1=data[0];

    //Get Alarm
    while((iReadDataBMS (data,0x6E,1)!=0) && (retry++<3));
    retry=0;
    Alarm2=data[0];
    
    //Get Alarm
    while((iReadDataBMS (data,0x6F,1)!=0) && (retry++<3));
    retry=0;
    Alarm3=data[0];

    //Get Alarm
    while((iReadDataBMS (data,0x70,1)!=0) && (retry++<3));
    retry=0;
    Alarm4=data[0];

    //Get Alarm
    while((iReadDataBMS (data,0x72,1)!=0) && (retry++<3));
    retry=0;
    Alarm5=data[0];

    //Get Alarm
    while((iReadDataBMS (data,0x73,1)!=0) && (retry++<3));
    retry=0;
    Alarm6=data[0];

    if (RequestDoneFlagUpdatePower) {
      //Charge MOS
      int err=iWriteDataBMS(0x121,MOSChargePower);
      //Discharge MOS
      err=err||iWriteDataBMS(0x122,MOSDischargePower);

      //Get MOS Charge and Discharge  Status
      while((iReadDataBMS (data,0x52,2)!=0) && (retry++<3));
      retry=0;
      MOSCharge=data[0];
      MOSDischarge=data[1];

      if (!err) {
        RequestDoneFlagUpdatePower=0;
      }
      publishUpdatePower=1;
    }
    
    vTaskDelay(5000);
  }
}

void TaskGPS(void *pvParameters){  // This is a task.

  bool isMovingSpeed=1;
  
  // GPS communication runs at 9600 baud
  gpsSerial.begin(9600);

  for (;;){ // A Task shall never return or exit.
    if (gps.location.isValid()) {
      Lat=gps.location.lat();
      Long=gps.location.lng();
    }
    //Update time of moving if speed is more than specified value
    if (gps.speed.kmph()>SPEEDMIN) {
      lastMoveSpeed=millis()+millisOffset;
    }
    Serial.printf("Time:%ld ms \t speed: %.2f km/h \t lastMoveSpeed:%ld ms \t isReadyToSleep: %d\n",millis()+millisOffset, gps.speed.kmph(), lastMoveSpeed, isReadyToSleep);
    //gives flag if vehicle not moving for TTS time
    if (millis()+millisOffset -lastMoveSpeed> TTS) {
      isMovingSpeed=0;
    } else {
      isMovingSpeed=1;
    }
    
    // printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
    // printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
    // printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    // printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
    // printInt(gps.location.age(), gps.location.isValid(), 5);
    // printDateTime(gps.date, gps.time);
    // printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
    // printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
    // printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
    // printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);
    // Serial.println();
    smartDelay(1000);
    vTaskDelay(1);
    if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
    
    //Sleep if device not moving for TTS time
    if (!isMovingSpeed && isReadyToSleep && isReadyToSleepResponse && isReadyToSleepOTA && millis()>TIME_ALIVE*1000) {
      Serial.println("SLEEP!");
      gpio_hold_en((gpio_num_t)GPS_EN);
      gpio_deep_sleep_hold_en();
      millisOffset+=millis();
      millisOffset+=TIME_TO_SLEEP*1000;
      esp_deep_sleep_start();
    }

  }
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);

}


static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);

}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
      vTaskDelay(1);
    }
  } while (millis() - start < ms);
}

void print_wakeup_reason(){
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO");  break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void loop()
{
}

