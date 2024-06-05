
void request_Mavlink() {
  //Request Data from Pixhawk
  uint8_t _system_id = 255;       // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2;      // seems like it can be any # except the number of what Pixhawk sys_id is
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

  Serial2.write(buf, len);  //Write data to serial port
}



void Mavlink_Telemetry() {
  mavlink_message_t msg;
  uint32_t time_boot_ms = millis();

  const char* name = "THRR1";
  float value = THRR1;
  mavlink_msg_named_value_float_pack(255, 2, &msg, time_boot_ms, name, value);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);

  name = "THRL1";
  value = THRL1;
  mavlink_msg_named_value_float_pack(255, 2, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);

  name = "THRL2";
  value = THRL1;
  mavlink_msg_named_value_float_pack(255, 2, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);

  name = "THRL2";
  value = THRL2;
  mavlink_msg_named_value_float_pack(255, 2, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);

  name = "THRL3";
  value = THRL2;
  mavlink_msg_named_value_float_pack(255, 2, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);

  name = "THRL3";
  value = THRL1;
  mavlink_msg_named_value_float_pack(255, 2, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);

  name = "RPMR1";
  value = RPMR1;
  mavlink_msg_named_value_float_pack(255, 2, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);

  name = "RPML1";
  value = RPML1;
  mavlink_msg_named_value_float_pack(255, 2, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);

  name = "RPMR2";
  value = RPMR2;
  mavlink_msg_named_value_float_pack(255, 2, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);

  name = "RPML2";
  value = RPML2;
  mavlink_msg_named_value_float_pack(255, 2, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);

  name = "RPMR3";
  value = RPMR3;
  mavlink_msg_named_value_float_pack(255, 2, &msg, time_boot_ms, name, value);
  buf[MAVLINK_MAX_PACKET_LEN];
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);

  name = "RPML3";
  value = RPML3;
  mavlink_msg_named_value_float_pack(255, 2, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);

  name = "VOLT1";
  value = VOLT1;
  mavlink_msg_named_value_float_pack(255, 2, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);

  name = "VOLT2";
  value = VOLT2;
  mavlink_msg_named_value_float_pack(255, 2, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);

  name = "VOLT3";
  value = VOLT3;
  mavlink_msg_named_value_float_pack(255, 2, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);

  name = "TEMP1";
  value = TEMP1;
  mavlink_msg_named_value_float_pack(255, 2, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);

  name = "TEMP2";
  value = TEMP2;
  mavlink_msg_named_value_float_pack(255, 2, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);

  name = "TEMP3";
  value = TEMP3;
  mavlink_msg_named_value_float_pack(255, 2, &msg, time_boot_ms, name, value);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
}




void MavLink_RC() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial2.available()) {
    uint8_t c = Serial2.read();

    //Get new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      //Handle new message from autopilot
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {

            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);

            //  Serial.print("\nFlight Mode: (10 ");
            // Serial.println(hb.custom_mode);
            //  Serial.print("Type: ");
            //  Serial.println(hb.type);
            //  Serial.print("Autopilot: ");
            //  Serial.println(hb.autopilot);
            //   Serial.print("Base Mode: ");
            //   Serial.println(hb.base_mode);
            //   Serial.print("System Status: ");
            //   Serial.println(hb.system_status);
            //   Serial.print("Mavlink Version: ");
            //   Serial.println(hb.mavlink_version);
            //    Serial.println();
          }
          break;
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:  // #35
          {
            mavlink_servo_output_raw_t SERVOCHANNEL;
            mavlink_msg_servo_output_raw_decode(&msg, &SERVOCHANNEL);
            Serial.println(SERVOCHANNEL.servo1_raw);
            Serial.print("Chanel 1 (raw): ");
            Serial.println(SERVOCHANNEL.servo2_raw);
            Serial.print("Chanel 2 (raw): ");
            rightoutput = map(SERVOCHANNEL.servo1_raw, 1000, 2000, -1000, 1000);
            leftoutput = map(SERVOCHANNEL.servo2_raw, 1000, 2000, -1000, 1000);
            Send(leftoutput, rightoutput);
          }
      }
    }
  }
}
