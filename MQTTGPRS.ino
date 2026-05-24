

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, length);
  SerialMon.println();

  //check if topic is update Power
  if (!strcmp(topic,topicUpdatePower)) {
    deserializeJson(docReceived,payload);
    MOSChargePower=docReceived["MOSCharge"];
    MOSDischargePower=docReceived["MOSDischarge"];
    RequestDoneFlagUpdatePower=docReceived["RequestDoneFlag"];
    isReadyToSleepResponse=0;
    Serial.println("Change MOSFET!");
  }

  //Check if topic is update Parameter Setting
  if (!strcmp(topic,topicUpdateParameterSetting)) {
    deserializeJson(docReceived,payload);
    PeriodicMQTT=docReceived["PeriodicMQTT"];
    PeriodicMQTT*=1000;
    newVersion=docReceived["newVersion"];
    RequestDoneFlagUpdateParameter=docReceived["RequestDoneFlag"];
    publishUpdateParameter=1;
    //Save device setting on EEPROM
    EEPROM.put(0, PeriodicMQTT);
    EEPROM.commit();
    isReadyToSleepResponse=0;
    Serial.println("Update Parameter!");
  }

  //Check if topic is update Parameter Setting
  if (!strcmp(topic,topicReadParameterSetting)) {
    publishReadParameter=1;
    isReadyToSleepResponse=0;
    Serial.println("Read Parameter!");
  }
}


void vMQTTSetup () {
  sprintf(clientId,"BMS%llX",chipID);
  sprintf(topicRealTimeStatus, "RealTimeStatus/%llX",chipID);
  sprintf(topicReadParameterSetting, "ReadParameterSetting/%llX",chipID);
  sprintf(topicSendParameterSetting, "SendParameterSetting/%llX",chipID);
  sprintf(topicUpdateParameterSetting, "UpdateParameterSetting/%llX",chipID);
  sprintf(topicUpdateSettingDone, "UpdateSettingDone/%llX",chipID);
  sprintf(topicUpdatePower, "UpdatePower/%llX",chipID);
  sprintf(topicUpdatePowerDone, "UpdatePowerDone/%llX",chipID);
  pubsubClient.setBufferSize(1024);
  pubsubClient.setKeepAlive(180); //Adjust pingreq time to 3 minutes
  
  pubsubClient.setClient(client);
  pubsubClient.setServer(broker, 1883);
  pubsubClient.setCallback(callback);
}

bool bMQTTConnect(char *clientPass) {
  if (!pubsubClient.connected()) {
    Serial.println(clientId);
    Serial.println(clientPass);
    if (pubsubClient.connect(clientId, "nouvelab", clientPass,0,0,0,0,false)) {
      Serial.println("connected");
      // pubsubClient.subscribe(topicRealTimeStatus,1);
      pubsubClient.subscribe(topicReadParameterSetting,1);
      // pubsubClient.subscribe(topicSendParameterSetting,1);
      pubsubClient.subscribe(topicUpdateParameterSetting,1);
      // pubsubClient.subscribe(topicUpdateSettingDone,1);
      pubsubClient.subscribe(topicUpdatePower,1);
      // pubsubClient.subscribe(topicUpdatePowerDone,1);
    } else {
      Serial.print("failed, rc=");
      Serial.print(pubsubClient.state());
    }
  }
  return pubsubClient.connected();
}

bool bMQTTReconnect(char *clientPass) {
  static int reconnectRetry=0;
  Serial.print("Reconnect: ");
  bool rc=bMQTTConnect(clientPass);
  Serial.println(rc);
  //Handle failed reconnection multiple times
  if (rc != 0) {
    reconnectRetry++;
  } else {
    reconnectRetry=0;
  }
  if (reconnectRetry>10) {
    Serial.println("==TOO MANY RECONNECTION ATTEMPTS FAILED, RESTARTING==");
    esp_restart();
  }
  return rc;
}



