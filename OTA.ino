#include <Update.h>

#ifdef TINY_GSM_MODEM_A76XXSSL
const char *server_OTA =  "https://firebasestorage.googleapis.com/v0/b/snovelab.firebasestorage.app/o/SmartBMS_V0_4_4G.ino.bin?alt=media";
#endif

#ifdef TINY_GSM_MODEM_SIM800
const char *server_OTA =  "https://firebasestorage.googleapis.com/v0/b/snovelab.firebasestorage.app/o/SmartBMS_V0_4_2G.ino.bin?alt=media";
#endif

void setupGPRS() {
  #ifdef TINY_GSM_MODEM_SIM800
    SerialAT.begin(115200, SERIAL_8N1, GSM_TX, GSM_RX);
  #else
    SerialAT.begin(115200, SERIAL_8N1, GSM_TX, GSM_RX);
  #endif
  pinMode(GSM_EN, OUTPUT);
  pinMode(GSM_POW, OUTPUT);
}

void turnOnGPRS() {
  Serial.println("Restart GPRS/4G");
  turnOffGPRS();
  vTaskDelay(5);
  Serial.println("TURNING ON GPRS/4G");
  digitalWrite(GSM_EN, HIGH);
  digitalWrite(GSM_POW, HIGH);
  int retry = 0;
  
  while (!modem.init()&& (retry++ < 10)) {
    Serial.println("FAILED TO CONNECT TO 4G MODULE");
    digitalWrite(GSM_EN, LOW);
    vTaskDelay(1000);
    digitalWrite(GSM_EN, HIGH);
    vTaskDelay(1000);
  }

  String name = modem.getModemName();
  DBG("Modem Name:", name);

  String modemInfo = modem.getModemInfo();
  DBG("Modem Info:", modemInfo);

  // Check if SIM card is online
  SimStatus sim = SIM_ERROR;
  sim = modem.getSimStatus();
  switch (sim) {
    case SIM_READY:
      Serial.println("SIM card online");
      break;
    case SIM_LOCKED:
      Serial.println("The SIM card is locked. Please unlock the SIM card first.");
      // const char *SIMCARD_PIN_CODE = "123456";
      // modem.simUnlock(SIMCARD_PIN_CODE);
      break;
    default:
      break;
  }

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
      SerialMon.println(" fail");
      vTaskDelay(3000);
      return;
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }

  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn)) {
      SerialMon.println(" fail");
      vTaskDelay(3000);
      return;
  }
  SerialMon.println(" success");

  if (modem.isGprsConnected()) {
      SerialMon.println("GPRS connected");
  }

  // Check network registration status and network signal status
  int16_t sq;
  Serial.print("Wait for the modem to register with the network.");
  RegStatus status = REG_NO_RESULT;
  retry = 0;
  while ((status == REG_NO_RESULT || status == REG_SEARCHING || status == REG_UNREGISTERED) && (retry < 10)) {
    status = modem.getRegistrationStatus();
    retry++;
    switch (status) {
      case REG_UNREGISTERED:
      case REG_SEARCHING:
        sq = modem.getSignalQuality();
        Serial.printf("[%lu] Signal Quality:%d\n", millis() / 1000, sq);
        vTaskDelay(1000);
        break;
      case REG_DENIED:
        Serial.println("Network registration was rejected, please check if the APN is correct");
        return;
      case REG_OK_HOME:
        Serial.println("Online registration successful");
        break;
      case REG_OK_ROAMING:
        Serial.println("Network registration successful, currently in roaming mode");
        break;
      default:
        Serial.printf("Registration Status:%d\n", status);
        vTaskDelay(1000);
        break;
    }
  }
  Serial.println();


  Serial.printf("Registration Status:%d\n", status);
  vTaskDelay(1000);

  String ueInfo;
  if (modem.getSystemInformation(ueInfo)) {
    Serial.print("Inquiring UE system information:");
    Serial.println(ueInfo);
  }

  vTaskDelay(5000);

  String ipAddress = modem.getLocalIP();
  Serial.print("Network IP:");
  Serial.println(ipAddress);

  vTaskDelay(1000);
}

void turnOffGPRS() {
  digitalWrite(GSM_EN, LOW);
  digitalWrite(GSM_POW, LOW);
}

void vUpdateOTA() {
  Serial.println("=====OTA=====");
  #ifdef TINY_GSM_MODEM_A76XXSSL
    // Initialize HTTPS
    Serial.println("Initialize HTTPS");
    modem.https_begin();

    // Set OTA Server URL
    if (!modem.https_set_url(server_OTA)) {
      Serial.println("Failed to set the URL. Please check the validity of the URL!");
      return;
    }

    // Send GET request
    int httpCode = 0;
    Serial.println("Get firmware form HTTPS");
    httpCode = modem.https_get();
    if (httpCode != 200) {
      Serial.print("HTTP get failed ! error code = ");
      Serial.println(httpCode);
      return;
    }

    

    // Get firmware size
    size_t firmware_size = modem.https_get_size();
    Serial.print("Firmware size : "); Serial.print(firmware_size); Serial.println("Kb");

    // Begin Update firmware
    Serial.println("Start upgrade firmware ...");
    if (!Update.begin(firmware_size)) {
        Serial.println("Not enough space to begin OTA");
        return;
    }

    char headerMD5[50];
    //Get only the hash
    strncpy(headerMD5,(strstr(modem.https_header().c_str(),"x-goog-meta-X-MD5: ")+19),32);
    headerMD5[32]='\0';
    Serial.println(headerMD5);
    Update.setMD5(headerMD5);

    uint8_t buffer[1024];
    int written = 0;
    int progress = 0;
    int total = 0;
    int len =0;
    do {
      // Read firmware form modem buffer
      len = modem.https_body(buffer, 1024);
      if (len <= 0)break;

      written = Update.write(buffer, len);
      if (written != len) {
          Serial.println("Written only : " + String(written) + "/" + String(len) + ". Retry?");
      }
      total += written;
      int newProgress = (total * 100) / firmware_size;
      if (newProgress - progress >= 5 || newProgress == 100) {
          progress = newProgress;
          Serial.print(String("\r ") + progress + "%\n");
      }
      //delay for stability
      vTaskDelay(200);
    } while (len>0);
    if (!Update.end()) {
        Serial.printf("Written %d from %dkB\n",total,firmware_size);
        Serial.println("Update not finished? Something went wrong!");
        Serial.println("Error Occurred. Error #: " + String(Update.getError()));
        return;
    }
    Serial.print("MD5: ");
    Serial.println(Update.md5String());

    Serial.println();

    if (!Update.isFinished()) {
        Serial.println("Update successfully completed.");
    }

    Serial.println("=== Update successfully completed. Rebooting.");

    vTaskDelay(1500);

    modem.https_end();
  #endif
  #ifdef TINY_GSM_MODEM_SIM800
    
    // Wait for network
    int i=0;
    for (i = 0; i < 3 && !modem.waitForNetwork(30000L, false); i++) {
      Serial.println(F("Restarting modem..."));
      modem.restart();
    }
    if (i >= 3) {
      Serial.println(F("Network failed and cancel update OTA"));
      return;
    } else {
      if (modem.isGprsConnected()) { Serial.println(" GPRS connected"); }
      httpOTA.connectionKeepAlive();  // Currently, this is needed for HTTPS
      int err = httpOTA.get(resourceOTA);
      if (err != 0) {
        Serial.println(F("failed to connect"));
        return;
      } else {
        int status = httpOTA.responseStatusCode();
        Serial.print(F("Response status code: "));
        Serial.println(status);
        if (!status) {
          return;
        }

        char headerMD5[50];
        //Get only the hash
        headerMD5[0]='\0';
        Serial.println("READING HEADER");
        while(httpOTA.headerAvailable())
        {
          if (httpOTA.readHeaderName() == "x-goog-meta-X-MD5") {
            strncpy(headerMD5,httpOTA.readHeaderValue().c_str(),32);
            break;
          }
        }
        headerMD5[32]='\0';


        unsigned long timeout = millis();
        uint32_t length = httpOTA.contentLength();
        if (length >= 0) {
          Serial.print(F("Content length is: "));
          Serial.println(length);
          Serial.println(F("Reading response data"));
          uint32_t readLength = 0;

          // Begin Update firmware
          Serial.println("Start upgrade firmware ...");
          if (!Update.begin(length)) {
              Serial.println("Not enough space to begin OTA");
              return;
          }

          
          Update.setMD5(headerMD5);
          Serial.println(headerMD5);

          uint8_t buffer[1024];
          int written = 0;
          int progress = 0;
          int total = 0;
          int len =0;
          do {
            // Read firmware form modem buffer
            len = clientSecure.read(buffer, 1024);
            if (len <= 0)break;
            
            written = Update.write(buffer, len);

            if (written != len) {
                Serial.println("Written only : " + String(written) + "/" + String(len) + ". Retry?");
            }
            total += written;
            int newProgress = (total * 100) / length;
            if (newProgress - progress >= 5 || newProgress == 100) {
                progress = newProgress;
                Serial.print(String("\r ") + progress + "%\n");
            }
            //delay for stability
            vTaskDelay(200);
          } while (len>0);
          if (!Update.end()) {
              Serial.printf("Written %d from %dkB\n",total,length);
              Serial.println("Update not finished? Something went wrong!");
              Serial.println("Error Occurred. Error #: " + String(Update.getError()));
              return;
          }
          Serial.print("MD5: ");
          Serial.println(Update.md5String());

          Serial.println();

          if (!Update.isFinished()) {
              Serial.println("Update successfully completed.");
          }

          Serial.println("=== Update successfully completed. Rebooting.");

          vTaskDelay(1500);
          
        }
      }
      httpOTA.stop();
    }
  #endif

  esp_restart();
}

int getGSMTIME(time_t *unixUTC)
{
    int     GSMyear = 0;
    int     GSMmonth = 0;
    int     GSMdate = 0;
    int     GSMhours = 0;
    int     GSMminutes = 0;
    int     GSMseconds = 0;
    float   GSMtimezone = 0;
    time_t  GSMUTCtime = 0;
    modem.NTPServerSync("pool.ntp.org",0);

    if (modem.getNetworkTime(&GSMyear, &GSMmonth, &GSMdate, &GSMhours, &GSMminutes, &GSMseconds, &GSMtimezone))
    {
      struct tm s;
      s.tm_sec  = (GSMseconds); 
      s.tm_min  = (GSMminutes);
      s.tm_hour = (GSMhours);
      s.tm_mday = (GSMdate);
      s.tm_mon  = (GSMmonth-1); // Month (0-11)
      s.tm_year = (GSMyear-1900); // Year since 1900
      GSMUTCtime   = mktime(&s);
      *unixUTC=GSMUTCtime;
      Serial.println(GSMUTCtime);
      Serial.printf("GSM Time:    %s",  ctime(&GSMUTCtime));
      if (GSMUTCtime > 1615155060) {      //  check for valid time, not 1939!
        return 1;
      }
    }
    return 0;
}