void vTurnOnBMS () {
  digitalWrite(BMS_ON,HIGH);
  vTaskDelay(1000);
  digitalWrite(BMS_ON,LOW);
}

int iReadDataBMS (uint16_t* data, uint16_t address, uint16_t length ) {
  uint8_t result = node.readHoldingRegisters(address,length);
  // Serial.print("Result: ");
  // Serial.println(result);
  if (result == node.ku8MBSuccess)
  {
    for (int j = 0; j < length; j++)
    {
      data[j] = node.getResponseBuffer(j);
      // Serial.println(data[j],DEC);
    }
  }
  if (result!=0) {
    Serial.println("FAILED TO READ FROM BMS");
  }
  //Delay for stability
  vTaskDelay(1);
  return result;
}

int iWriteDataBMS (uint16_t address, uint16_t value ) {
  uint8_t result = node.writeSingleRegister(address,value);
  // Serial.print("Write Result: ");
  // Serial.println(result);
  vTaskDelay(5);
  return result;
}