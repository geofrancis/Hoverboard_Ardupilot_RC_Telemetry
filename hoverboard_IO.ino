void Send(int16_t uSteer, int16_t uSpeed) {
  // Create command
  Command.start = (uint16_t)START_FRAME;
  Command.steer = (int16_t)uSteer;
  Command.speed = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial1.write((uint8_t *)&Command, sizeof(Command));
  HoverSerial2.write((uint8_t *)&Command, sizeof(Command));
  HoverSerial3.write((uint8_t *) &Command, sizeof(Command));
}





void Receive1() {
  // Check for new data availability in the Serial buffer
  if (HoverSerial1.available()) {
    incomingByte = HoverSerial1.read();                                  // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;  // Construct the start frame
  } else {
    return;
  }

// If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  Serial.print(incomingByte);
  return;
#endif

  // Copy received data
  if (bufStartFrame == START_FRAME) {  // Initialize if new data is detected
    p = (byte *)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
    *p++ = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                          ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
      THRR1 = (Feedback.cmd1);
      THRL1 = (Feedback.cmd2);
      RPMR1 = (Feedback.speedR_meas);
      RPML1 = (Feedback.speedL_meas);
      VOLT1 = (Feedback.batVoltage);
      TEMP1 = (Feedback.boardTemp);

      // Print data to built-in Serial
      Serial.print("1: ");
      Serial.print(Feedback.cmd1);
      Serial.print(" 2: ");
      Serial.print(Feedback.cmd2);
      Serial.print(" 3: ");
      Serial.print(Feedback.speedR_meas);
      Serial.print(" 4: ");
      Serial.print(Feedback.speedL_meas);
      Serial.print(" 5: ");
      Serial.print(Feedback.batVoltage);
      Serial.print(" 6: ");
      Serial.print(Feedback.boardTemp);
      Serial.print(" 7: ");
      Serial.println(Feedback.cmdLed);
    } else {
      Serial.println("Non-valid data skipped");
    }
    idx = 0;  // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}

void Receive2() {
  // Check for new data availability in the Serial buffer
  if (HoverSerial2.available()) {
    incomingByte = HoverSerial2.read();                                  // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;  // Construct the start frame
  } else {
    return;
  }

// If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  Serial.print(incomingByte);
  return;
#endif

  // Copy received data
  if (bufStartFrame == START_FRAME) {  // Initialize if new data is detected
    p = (byte *)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
    *p++ = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                          ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
      THRR2 = (Feedback.cmd1);
      THRL2 = (Feedback.cmd2);
      RPMR2 = (Feedback.speedR_meas);
      RPML2 = (Feedback.speedL_meas);
      VOLT2 = (Feedback.batVoltage);
      TEMP2 = (Feedback.boardTemp);

      // Print data to built-in Serial
      Serial.print("1: ");
      Serial.print(Feedback.cmd1);
      Serial.print(" 2: ");
      Serial.print(Feedback.cmd2);
      Serial.print(" 3: ");
      Serial.print(Feedback.speedR_meas);
      Serial.print(" 4: ");
      Serial.print(Feedback.speedL_meas);
      Serial.print(" 5: ");
      Serial.print(Feedback.batVoltage);
      Serial.print(" 6: ");
      Serial.print(Feedback.boardTemp);
      Serial.print(" 7: ");
      Serial.println(Feedback.cmdLed);
    } else {
      Serial.println("Non-valid data skipped");
    }
    idx = 0;  // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}

void Receive3() {
  // Check for new data availability in the Serial buffer
  if (HoverSerial3.available()) {
    incomingByte = HoverSerial3.read();                                  // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;  // Construct the start frame
  } else {
    return;
  }

// If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  Serial.print(incomingByte);
  return;
#endif

  // Copy received data
  if (bufStartFrame == START_FRAME) {  // Initialize if new data is detected
    p = (byte *)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
    *p++ = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                          ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
      THRR3 = (Feedback.cmd1);
      THRL3 = (Feedback.cmd2);
      RPMR3 = (Feedback.speedR_meas);
      RPML3 = (Feedback.speedL_meas);
      VOLT3 = (Feedback.batVoltage);
      TEMP3 = (Feedback.boardTemp);

      // Print data to built-in Serial
      Serial.print("1: ");
      Serial.print(Feedback.cmd1);
      Serial.print(" 2: ");
      Serial.print(Feedback.cmd2);
      Serial.print(" 3: ");
      Serial.print(Feedback.speedR_meas);
      Serial.print(" 4: ");
      Serial.print(Feedback.speedL_meas);
      Serial.print(" 5: ");
      Serial.print(Feedback.batVoltage);
      Serial.print(" 6: ");
      Serial.print(Feedback.boardTemp);
      Serial.print(" 7: ");
      Serial.println(Feedback.cmdLed);
    } else {
      Serial.println("Non-valid data skipped");
    }
    idx = 0;  // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}

