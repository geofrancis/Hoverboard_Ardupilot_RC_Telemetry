

void power() {

  if (powerchannel > 1800) {
    brdall = 1;
  }

  if (powerchannel < 1300) {
    brdall = 0;
  }

  Serial.println(brdallc);
  int brd1 = digitalRead(8);
  int brd2 = digitalRead(9);
  int brd3 = digitalRead(10);
  Serial.print(" brd1 ");
  Serial.print(brd1);
  Serial.print(" brd1 ");
  Serial.print(brd2);
  Serial.print(" brd1 ");
  Serial.println(brd3);



  if (brdall == 1) {

    if (brdallc == 1) {
      if (brd1 == 1) {
        digitalWrite(board1switch, LOW);
      } else {
        digitalWrite(board1switch, HIGH);
      }
      if (brd2 == 1) {
        digitalWrite(board2switch, LOW);
      } else {
        digitalWrite(board2switch, HIGH);
      }
      if (brd3 == 1) {
        digitalWrite(board3switch, LOW);
      } else {
        digitalWrite(board3switch, HIGH);
      }
      delay(1000);
      digitalWrite(board1switch, LOW);
      digitalWrite(board2switch, LOW);
      digitalWrite(board3switch, LOW);
      Serial.println("LOW");
      brdallc = (brdallc + 1);
      if (brdallc > 1) {
        brdallc = 10;
        brdalld = 1;
      }
    }
  }


if (brdall == 0) {
      if (brdalld == 1) {
      if (brd1 == 0) {
        digitalWrite(board1switch, LOW);
      } else {
        digitalWrite(board1switch, HIGH);
      }
      if (brd2 == 0) {
        digitalWrite(board2switch, LOW);
      } else {
        digitalWrite(board2switch, HIGH);
      }
      if (brd3 == 0) {
        digitalWrite(board3switch, LOW);
      } else {
        digitalWrite(board3switch, HIGH);
      }
      delay(1000);
      digitalWrite(board1switch, LOW);
      digitalWrite(board2switch, LOW);
      digitalWrite(board3switch, LOW);
      Serial.println("LOW");
      brdalld = (brdalld + 1);
      if (brdalld > 1) {
        brdalld = 10;
        brdallc = 1;
      }
    }
  Serial.println("HIGH");
}
}






void Send1(int16_t uSteer, int16_t uSpeed) {
  // Create command
  Command.start = (uint16_t)START_FRAME;
  Command.steer = (int16_t)uSteer;
  Command.speed = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);
  HoverSerial1.write((uint8_t *)&Command, sizeof(Command));
  // Serial.println("send1");
}

void Send2(int16_t uSteer, int16_t uSpeed) {
  // Create command
  Command.start = (uint16_t)START_FRAME;
  Command.steer = (int16_t)uSteer;
  Command.speed = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);
  HoverSerial2.write((uint8_t *)&Command, sizeof(Command));
  //   Serial.println("send2");
}


void Send3(int16_t uSteer, int16_t uSpeed) {
  // Create command
  Command.start = (uint16_t)START_FRAME;
  Command.steer = (int16_t)uSteer;
  Command.speed = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);
  HoverSerial3.write((uint8_t *)&Command, sizeof(Command));
  //  Serial.println("send3");
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
      //   Serial.println("VOLT1 ");
      //  Serial.println(VOLT1);

    } else {
      // Serial.println("Non-valid data skipped");
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
      //         Serial.println("VOLT2 ");
      //   Serial.println(VOLT2);

      // Print data to built-in Serial

    } else {
      //  Serial.println("Non-valid data skipped");
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
      //    Serial.println("VOLT3 ");
      //   Serial.println(VOLT3);
    } else {
      //Serial.println("Non-valid data skipped");
    }
    idx = 0;  // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}
