
void request_datastream() {
  //Request Data from Pixhawk
  uint8_t _system_id = 255; // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2; // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1; // Id # of Pixhawk (should be 1)
  uint8_t _target_component = 0; // Target component, 0 = all (seems to work with 0 or 1
  uint8_t _req_stream_id = MAV_DATA_STREAM_ALL;
  uint16_t _req_message_rate = 0x08; //number of times per second to request the data in hex
  uint8_t _start_stop = 1; //1 = start, 0 = stop
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

  Serial2.write(buf, len); //Write data to serial port
}



void send_telemetry(){
uint8_t _system_id = 255; // id of computer which is sending the command (ground control software has id of 255)
uint8_t _component_id = 2; // seems like it can be any # except the number of what Pixhawk sys_id is
uint32_t time_boot_ms = 0;
float value;
mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];

char name = THRR1;
value = (THRR1);

mavlink_msg_named_value_float_pack(_system_id, _component_id, &msg, time_boot_ms,name,value);
uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

  Serial2.write(buf, len); //Write data to serial port
}



void MavLink_receive() {
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
            rightoutput = (SERVOCHANNEL.servo1_raw);
            leftoutput = (SERVOCHANNEL.servo2_raw);
          }
      }
    }
  }
}


