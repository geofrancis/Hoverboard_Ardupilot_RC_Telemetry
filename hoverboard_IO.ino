
void power() {

//armed = 1;
 //FCOK = 1;
 //active = 1;


  if (FCOK == 1) {
    if (armed == 1) {
      Serial.println("--------------------------------------------------------------------------------------------------------------ARMED");
      if (digitalRead(board2power) == 0) {
      two_on();
      }
      if (active == 1) {
        Serial.println("-------------------------------------------------------------------------------------------------ACTIVE");
        if (digitalRead(board1power) == 0) {
        one_on();
        }
        if (digitalRead(board3power) == 0) {
        three_on();
        }
      }
    }
  }

  if (active == 0) {
    Serial.println("---------------------------------------------------------------------------------------------------INACTIVE");
    one_off();
    three_off();
  }

  if (armed == 0) {
    Serial.println("------------------------------------------------------------------------------------------------------------------DISARMED");
    one_off();
    two_off();
    three_off();
  }

  if (FCOK == 0) {
Serial.println("---------------------------------------------------------------------------------------------------FCOK ");
Serial.println(FCOK);
    one_off();
    two_off();
    three_off();
  }
}


void one_on() {
  Serial.println("------------------------------------------------------------------one on");
  if (set1 > 1) {
    Serial.println("-----------------------------------------------------------------------------------ESC 1 FAILURE");
  }
  if (set1 < 2) {
    Serial.println(digitalRead(board1power));
    if (digitalRead(board1power) == 0) {
      digitalWrite(board1switch, HIGH);
      delay(2000);
      digitalWrite(board1switch, LOW);
      set1++;
      Serial.println(set1);
      if (set1 > 2) {
        Serial.println("---------------------------------------ESC 2 FAIL TO START");
      }
    }
  }
}


void two_on() {
  Serial.println("--------------------------------------------------------------------two on");
  if (set2 > 1) {
    Serial.println("-----------------------------------------------------------------------------------ESC 2 FAILURE");
  }
  if (set2 < 2) {
    Serial.println(digitalRead(board2power));
    if (digitalRead(board2power) == 0) {
      digitalWrite(board2switch, HIGH);
      delay(2000);
      digitalWrite(board2switch, LOW);
      Serial.println(set2);
      set2++;
      if (set2 > 2) {
        Serial.println("--------------------------------------ESC 2 FAIL TO START");
      }
    }
  }
}


void three_on() {
  Serial.println("----------------------------------------------------------------------three on");
  if (set3 > 1) {
    Serial.println("-----------------------------------------------------------------------------------ESC 3 FAILURE");
  }
  if (set3 < 2) {
    Serial.println(digitalRead(board2power));
    if (digitalRead(board3power) == 0) {
      digitalWrite(board3switch, HIGH);
      delay(2000);
      digitalWrite(board3switch, LOW);
      
      Serial.println(set3);
      set3++;
      if (set3 > 2) {
        Serial.println("------------------------------------ESC 3 FAIL TO START");
      }
    }
  }
}

void one_off() {
  Serial.println("------------------------------------------------------------------------------------one off");
  Serial.println(digitalRead(board1power));
  if (digitalRead(board1power) == 1) {
    digitalWrite(board1switch, HIGH);
    Serial.println("----------------------------------------------------------------------------------one switch");
    delay(2000);
    digitalWrite(board1switch, LOW);
    delay(1000);
  }
}

void two_off() {
  Serial.println("------------------------------------------------------------------------------------two off");
  Serial.println(digitalRead(board2power));
  if (digitalRead(board2power) == 1) {
    digitalWrite(board2switch, HIGH);
    Serial.println("----------------------------------------------------------------------------------two switch");
    delay(2000);
    digitalWrite(board2switch, LOW);
    delay(1000);
  }
}

void three_off() {
  Serial.println("------------------------------------------------------------------------------------three off");
  Serial.println(digitalRead(board3power));
  if (digitalRead(board3power) == 1) {
    digitalWrite(board3switch, HIGH);
    Serial.println("----------------------------------------------------------------------------------three switch");
    delay(2000);
    digitalWrite(board3switch, LOW);
    delay(1000);
  }
}




void SendTHR(int16_t uSteer, int16_t uSpeed) {
  // Create command
  Command.start = (uint16_t)START_FRAME;
  Command.steer = (int16_t)uSteer;
  Command.speed = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);
  HoverSerial1.write((uint8_t *)&Command, sizeof(Command));
  HoverSerial2.write((uint8_t *)&Command, sizeof(Command));
  HoverSerial3.write((uint8_t *)&Command, sizeof(Command));

  //Serial.println("sendTHR");
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
      /// Serial.println("ESC 1 Telem");

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
      //  Serial.println("ESC 2 Telem");
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
      // Serial.println("ESC 3 Telem");
    } else {
      //Serial.println("Non-valid data skipped");
    }
    idx = 0;  // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}
