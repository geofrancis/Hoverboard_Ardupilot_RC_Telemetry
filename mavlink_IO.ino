
void request_Mavlink() {
  //Request Data from Pixhawk
  uint8_t _system_id = 255;       // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 158;    // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1;     // Id # of Pixhawk (should be 1)
  uint8_t _target_component = 0;  // Target component, 0 = all (seems to work with 0 or 1
  uint8_t _req_stream_id = MAV_DATA_STREAM_ALL;
  uint16_t _req_message_rate = 0xA;  //number of times per second to request the data in hex
  uint8_t _start_stop = 1;           //1 = start, 0 = stop
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

  Serial1.write(buf, len);  //Write data to serial port
}




void MavLink_RC() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial1.available()) {
    uint8_t c = Serial1.read();

    //Get new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {

            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            HBWATCH();

            if ((hb.type) == 10) {
              if ((hb.custom_mode) == 4) {
                Serial.println("HOLD MOTOR ON");
                active = 0;
              }
              if ((hb.custom_mode) != 4) {
                Serial.println("HOLD MOTOR OFF");
                active = 1;
              }
            }
            Serial.print("\nFlight Mode: ");
            Serial.println(hb.custom_mode);
            BASEMODE = (hb.base_mode);;
            if (BASEMODE == 193) {
              armed = 1;
              Serial.println("------------------------------------------------------------------------------------ARMED");
            }

            if (BASEMODE == 65) {
              Serial.println("------------------------------------------------------------------------------------DISARMED");
              armed = 0;
            }

            //  Serial.print("Type: ");
            //  Serial.println(hb.type);
            //  Serial.print("Autopilot: ");
            //  Serial.println(hb.autopilot);
            Serial.print("Base Mode: ");
            Serial.println(hb.base_mode);
            //  Serial.print("System Status: ");
            // Serial.println(hb.system_status);
            //   Serial.print("Mavlink Version: ");
            //   Serial.println(hb.mavlink_version);
            //    Serial.println();
          }
          break;
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:  // #35
          {
            mavlink_servo_output_raw_t SERVOCHANNEL;
            mavlink_msg_servo_output_raw_decode(&msg, &SERVOCHANNEL);
            // Serial.print("Chanel 1 (raw): ");
            // Serial.println(SERVOCHANNEL.servo1_raw);
            //Serial.print("Chanel 13 (raw): ");
            //Serial.println(SERVOCHANNEL.servo13_raw);
            // MAXRPM = map(SERVOCHANNEL.servo14_raw, 1000, 2000, 100, 300);
            leftoutput = map(SERVOCHANNEL.servo15_raw, 1000, 2000, -MAXRPM, MAXRPM);
            rightoutput = map(SERVOCHANNEL.servo16_raw, 1000, 2000, -MAXRPM, MAXRPM);
            powerchannel = (SERVOCHANNEL.servo13_raw);
            //Serial.println(powerchannel);

            SendTHR(leftoutput, rightoutput);
            //Send2(leftoutput2, rightoutput2);
            //Send3(leftoutput3, rightoutput3);
          }
          break;

       
      }
    }
  }
}



void FCHBC() {
  Serial.print("FCHB ");
  // Serial.println(FCHB);
  if (FCHB == 0) {
    Serial.println("-------------------------------------------------------------NO FC HEARTBEAT");
    Serial.println(FCOK);
    Serial.println(FCHB);
    FCOK = 0;
  }
  if (FCHB > 1) {
    Serial.print("Rover ");
    Serial.println(FCOK);
    Serial.print(FCHB);
    Serial.println("Beats ");
    Serial.println("----------------------------------------------------------FC HEARTBEAT");
    FCHB = 0;
    FCOK = 1;
  }
}




void HBWATCH() {
  FCHB++;
  Serial.print("tick****************************************************************************");
}


void MAVLINK_HB() {
  uint8_t autopilot_type = MAV_TYPE_ONBOARD_CONTROLLER;
  uint8_t system_mode = MAV_MODE_PREFLIGHT;  ///< Booting up
  uint32_t custom_mode = 0;                  ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY;  ///< System ready for flight
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  int type = MAV_TYPE_SERVO;
  // Pack the message
  //Serial.print("mavhb");
  mavlink_msg_heartbeat_pack(1, 158, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}




void MAVLINK_HB1() {
  if (brd1 == 1) {
    uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
    uint8_t system_mode = MAV_MODE_PREFLIGHT;  ///< Booting up
    uint32_t custom_mode = 1;                  ///< Custom mode, can be defined by user/adopter
    uint8_t system_state = MAV_STATE_STANDBY;  ///< System ready for flight
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int type = MAV_TYPE_SERVO;
    // Pack the message
    // Serial.print("mavhb1");
    mavlink_msg_heartbeat_pack(1, 140, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }
}

void MAVLINK_HB2() {
  if (brd2 == 1) {
    uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
    uint8_t system_mode = MAV_MODE_PREFLIGHT;  ///< Booting up
    uint32_t custom_mode = 2;                  ///< Custom mode, can be defined by user/adopter
    uint8_t system_state = MAV_STATE_STANDBY;  ///< System ready for flight
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int type = MAV_TYPE_SERVO;
    // Pack the message
    // Serial.print("mavhb2");
    mavlink_msg_heartbeat_pack(1, 141, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }
}

void MAVLINK_HB3() {
  if (brd3 == 1) {
    uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
    uint8_t system_mode = MAV_MODE_PREFLIGHT;  ///< Booting up
    uint32_t custom_mode = 3;                  ///< Custom mode, can be defined by user/adopter
    uint8_t system_state = MAV_STATE_STANDBY;  ///< System ready for flight
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int type = MAV_TYPE_SERVO;
    // Pack the message
    // Serial.print("mavhb3");
    mavlink_msg_heartbeat_pack(1, 142, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }
}






void MAVLINK_ESC_1() {
  // Serial.print("ESC1");
  mavlink_message_t msg;
  uint64_t time_usec = 1;
  uint8_t index = 1; /*<  Index of the first ESC in this message. minValue = 0, maxValue = 60, increment = 4.*/

  int32_t rrpm[] = { 200, 400, 800, 300 };
  float Voltage[] = { 20, 40, 80, 30 };
  float current[] = { 20, 40, 80, 30 };




  mavlink_msg_esc_status_pack(1, 143, &msg, 0, time_usec, rrpm, Voltage, current);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}




void Mavlink_Telemetry1() {
  if (VOLT1 > 1) {
    mavlink_message_t msg;
    uint32_t time_boot_ms = millis();
    //Serial.print("mavT1");
    const char* name = "THRR1";
    float value = (THRR1);
    mavlink_msg_named_value_float_pack(1, 140, &msg, time_boot_ms, name, value);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);

    name = "THRL1";
    value = THRL1;
    mavlink_msg_named_value_float_pack(1, 140, &msg, time_boot_ms, name, value);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);


    name = "RPMR1";
    value = RPMR1;
    mavlink_msg_named_value_float_pack(1, 140, &msg, time_boot_ms, name, value);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);

    name = "RPML1";
    value = RPML1;
    mavlink_msg_named_value_float_pack(1, 140, &msg, time_boot_ms, name, value);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);



    name = "VOLT1";
    value = (VOLT1 / 100);
    mavlink_msg_named_value_float_pack(1, 140, &msg, time_boot_ms, name, value);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
    //Serial.print("V1: ");
    //Serial.println(VOLT1);


    name = "TEMP1";
    value = (TEMP1 / 10);
    mavlink_msg_named_value_float_pack(1, 140, &msg, time_boot_ms, name, value);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }
}



void Mavlink_Telemetry2() {
  if (VOLT2 > 1) {
    mavlink_message_t msg;
    uint32_t time_boot_ms = millis();
    //Serial.print("mavT2");
    const char* name = "THRR2";
    float value = (THRR2);
    mavlink_msg_named_value_float_pack(1, 141, &msg, time_boot_ms, name, value);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);

    name = "THRL2";
    value = THRL2;
    mavlink_msg_named_value_float_pack(1, 141, &msg, time_boot_ms, name, value);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);


    name = "RPMR2";
    value = RPMR2;
    mavlink_msg_named_value_float_pack(1, 141, &msg, time_boot_ms, name, value);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);

    name = "RPML2";
    value = RPML2;
    mavlink_msg_named_value_float_pack(1, 141, &msg, time_boot_ms, name, value);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);



    name = "VOLT2";
    value = (VOLT2 / 100);
    mavlink_msg_named_value_float_pack(1, 141, &msg, time_boot_ms, name, value);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
    // Serial.print("V2: ");
    //Serial.println(VOLT2);


    name = "TEMP2";
    value = (TEMP2 / 10);
    mavlink_msg_named_value_float_pack(1, 141, &msg, time_boot_ms, name, value);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }
}


void Mavlink_Telemetry3() {
  if (VOLT3 > 1) {
    mavlink_message_t msg;
    uint32_t time_boot_ms = millis();
    //Serial.print("mavT3");
    const char* name = "THRR3";
    float value = (THRR3);
    mavlink_msg_named_value_float_pack(1, 142, &msg, time_boot_ms, name, value);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);

    name = "THRL3";
    value = THRL3;
    mavlink_msg_named_value_float_pack(1, 142, &msg, time_boot_ms, name, value);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);


    name = "RPMR3";
    value = RPMR3;
    mavlink_msg_named_value_float_pack(1, 142, &msg, time_boot_ms, name, value);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);

    name = "RPML3";
    value = RPML3;
    mavlink_msg_named_value_float_pack(1, 142, &msg, time_boot_ms, name, value);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);



    name = "VOLT3";
    value = (VOLT3 / 100);
    mavlink_msg_named_value_float_pack(1, 142, &msg, time_boot_ms, name, value);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
    // Serial.print("V3: ");
    //Serial.println(VOLT3);


    name = "TEMP3";
    value = (TEMP3 / 10);
    mavlink_msg_named_value_float_pack(1, 142, &msg, time_boot_ms, name, value);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }
}



void MAVLINK_BATTHB() {
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  uint8_t system_mode = MAV_MODE_PREFLIGHT;  ///< Booting up
  uint32_t custom_mode = 0;                  ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY;  ///< System ready for flight
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  int type = MAV_TYPE_GROUND_ROVER;
  // Pack the message

  mavlink_msg_heartbeat_pack(1, 180, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}


void MAVBATTERY() {

  long C1 = myCellVoltages[1];
  long C2 = myCellVoltages[2];
  long C3 = myCellVoltages[3];
  long C4 = myCellVoltages[4];
  long C5 = myCellVoltages[5];
  long C6 = myCellVoltages[6];
  long C7 = myCellVoltages[7];
  long C8 = myCellVoltages[8];
  long C9 = myCellVoltages[9];
  long C10 = myCellVoltages[10];
  long C11 = myCellVoltages[11];
  long C12 = myCellVoltages[12];
  long C13 = myCellVoltages[13];
  long C14 = myCellVoltages[14];
  uint16_t CELLSA[10] = { C1, C2, C3, C4, C5, C6, C7, C8, C9, C10 };
  uint16_t CELLSB[4] = { C11, C12, C13, C14 };

  uint8_t system_id = 1;  // id of computer which is sending the command (ground control software has id of 255)
  uint8_t component_id = 180;
  mavlink_message_t msg;
  uint8_t id = 0;
  uint8_t battery_function = 2;  //propulsioni
  uint8_t type = 3;              //liion
  int16_t temperature = Temp_probe_1f;
  uint16_t* voltages = CELLSA;
  int16_t current_battery = 99;
  int32_t current_consumed = (20000 - RemainCapacityf);
  int32_t energy_consumed = 10000;
  int8_t battery_remaining = RSOC;
  int32_t time_remaining = 0;
  uint8_t charge_state = 1;
  uint16_t* voltages_ext = CELLSB;
  uint8_t mode = 0;
  uint32_t fault_bitmask = 0;


  //Pack battery message
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_battery_status_pack(system_id, component_id, &msg, id, battery_function, type, temperature, voltages, current_battery, current_consumed, energy_consumed, battery_remaining, time_remaining, charge_state, voltages_ext, mode, fault_bitmask);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

  //if (CELLS[1] > 1) {
  Serial1.write(buf, len);  //Write data to serial port
  //}
}
