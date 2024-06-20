void Send(int16_t uSteer, int16_t uSpeed) {
  // Create command
  Command.start = (uint16_t)START_FRAME;
  Command.steer = (int16_t)uSteer;
  Command.speed = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial1.write((uint8_t *)&Command, sizeof(Command));
  HoverSerial2.write((uint8_t *)&Command, sizeof(Command));
 // HoverSerial3.write((uint8_t *)&Command, sizeof(Command));

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
  if (Serial2.available()) {
    incomingByte = Serial2.read();                                  // Read the incoming byte
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

    } else {
      Serial.println("Non-valid data skipped");
    }
    idx = 0;  // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}

void powercycleon1() {

      if (digitalRead(board1power) == HIGH) {
        board1on = 1;
      } else {
        board1on = 0;
      }

  if (boardson = 1) {
    if (startcycle = 1) {

      if (board1error = 0) {
        if (board1on == 0) {
          digitalWrite(board1switch, HIGH);
        } else {
          digitalWrite(board1switch, LOW);
        }
      }


      // if switch is on, turn it off after timer
      unsigned long currentMillisb1 = millis();
      if (currentMillisb1 - buttonMillis >= buttoninterval) {
        buttonMillis = currentMillisb1;
      
        if (digitalRead(board1switch) == HIGH) {
          digitalWrite(board1switch, LOW);
          board1error = 1;
        }
      }



      //wait and check if board has powered on.
      unsigned long currentMillisp1 = millis();
      if (currentMillisp1 - checkMillis >= boardinterval) {
        // save the last time you blinked the LED
        checkMillis = currentMillisp1;

        if (board1on == 1) {
          board1error = 0;
          Serial.print("Board 1 ON");
        } else {
          Serial.print("Board 1 ERROR");
        }
        startcycle = 0;
      }
    }
  }
}


void powercycleoff() {


  
      if (digitalRead(board1power) == HIGH) {
        board1on = 1;
      } else {
        board1on = 0;
      }

      if (digitalRead(board2power) == HIGH) {
        board2on = 1;
      } else {
        board2on = 0;
      }

      if (digitalRead(board3power) == HIGH) {
        board3on = 1;
      } else {
        board3on = 0;
      }

/////////////////////////////

  if (boardson = 0) {
    if (stopcycle = 1) {

      if (board1error = 0) {
        if (board1on == HIGH) {
          digitalWrite(board1switch, HIGH);
        } else {
          digitalWrite(board1switch, LOW);
        }
      }

      if (board2error = 0) {
        if (board2on == HIGH) {
          digitalWrite(board2switch, HIGH);
        } else {
          digitalWrite(board2switch, LOW);
        }
      }

      if (board3error = 0) {
        if (board3on == HIGH) {
          digitalWrite(board3switch, HIGH);
        } else {
          digitalWrite(board3switch, LOW);
        }
      }

////////////////////

      unsigned long currentMillisb0 = millis();
      if (currentMillisb0 - buttonMillis >= buttoninterval) {
        buttonMillis = currentMillisb0;



        //if boards are on, switch power button.
        if (board1error = 0) {
          if (board1on == 1) {
            digitalWrite(board1switch, HIGH);
          } else {
            digitalWrite(board1switch, LOW);
          }
        }

        if (board2error = 0) {
          if (board2on == 1) {
            digitalWrite(board2switch, HIGH);
          } else {
            digitalWrite(board2switch, LOW);
          }
        }

        if (board3error = 0) {
          if (board3on == 1) {
            digitalWrite(board3switch, HIGH);
          } else {
            digitalWrite(board3switch, LOW);
          }
        }

////////////////////////
        // if switch is on, turn it off after timer
        unsigned long currentMillis = millis();

        if (currentMillisb0 - buttonMillis >= buttoninterval) {
          buttonMillis = currentMillisb0;

          if (digitalRead(board1switch) == HIGH) {
            digitalWrite(board1switch, LOW);
          }

          if (digitalRead(board2switch) == HIGH) {
            digitalWrite(board2switch, LOW);
          }

          if (digitalRead(board3switch) == HIGH) {
            digitalWrite(board3switch, LOW);
          }
        }



        //wait and check if board has powered off.
        if (currentMillis - checkMillis >= boardinterval) {
          checkMillis = currentMillis;

          if (board1on == 0) {
            board1error = 0;
            Serial.print("Board 1 OFF");
          } else {
            Serial.print("Board 1 ERROR");
          }

          if (board2on == 0) {
            board2error = 0;
            Serial.print("Board 2 OFF");
          } else {
            Serial.print("Board 2 ERROR");
          }

          if (board3on == 0) {
            board3error = 0;
            Serial.print("Board 3 OFF");
          } else {
            Serial.print("Board 3 ERROR");
          }
          stopcycle = 0;
        }
      }
    }
  }
}
