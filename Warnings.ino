void warning() {
  uint8_t system_id = 1;
  uint8_t component_id = 158;
  uint8_t severity = 2;
  uint16_t id = 0;
  uint8_t chunk_seq = 0;



  if (TEMPMAX > 79) {

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_statustext_pack(system_id, component_id, &msg, 0, "ESC themal shutdown imminent", id, chunk_seq);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }

  if (TEMP1 > 650) {


    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_statustext_pack(system_id, component_id, &msg, 0, "ESC 1 Hot", id, chunk_seq);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
   // Serial.print("TEMP1  ");
   // Serial.println(TEMP1);
  }

  if (TEMP2 > 650) {



    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_statustext_pack(system_id, component_id, &msg, 0, "ESC 2 Hot", id, chunk_seq);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
   // Serial.print("TEMP2  ");
   // Serial.println(TEMP2);
  }

  if (TEMP3 > 650) {

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_statustext_pack(system_id, component_id, &msg, 0, "ESC 3 Hot", id, chunk_seq);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
   // Serial.print("TEMP3  ");
   // Serial.println(TEMP3);
  }

  TEMPMAX = 0;


  RPMRAVG = ((RPMR1 + RPMR2 + RPMR3) / 3);
  RPMLAVG = ((RPML1 + RPML2 + RPML3) / 3);

 // Serial.print("RPMRAVG  ");
 // Serial.print(RPMRAVG);
 // Serial.print("  ");
 // Serial.print(RPMR1);
 // Serial.print("  ");
 // Serial.print(RPMR2);
 // Serial.print("  ");
 // Serial.println(RPMR3);
 // Serial.print("     RAVG  ");
 // Serial.print(RPMR1 / RPMRAVG, 3);
 // Serial.print("  ");
 // Serial.print(RPMR2 / RPMRAVG), 3;
 // Serial.print("  ");
 // Serial.println(RPMR3 / RPMRAVG), 3;


 // Serial.print("RPMLAVG  ");
 // Serial.print(RPMLAVG);
 // Serial.print("  ");
 // Serial.print(RPML1);
 // Serial.print("  ");
 // Serial.print(RPML2);
 // Serial.print("  ");
 // Serial.println(RPML3);
 // Serial.print("     LAVG ");
 // Serial.print(RPML1 / RPMLAVG), 3;
 // Serial.print("  ");
 // Serial.print(RPML2 / RPMLAVG), 3;
 // Serial.print("  ");
 // Serial.println(RPML3 / RPMLAVG), 3;


  if (THRR1 > 1) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    if ((RPMR1 / RPMRAVG) < 0.9) {
      mavlink_msg_statustext_pack(system_id, component_id, &msg, severity, "R1 motor slow", id, chunk_seq);
     // Serial.println("                                                   R1S  ");
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial1.write(buf, len);
      //Serial1.flush();
    }
    if ((RPMR1 / RPMRAVG) == 0) {
      mavlink_msg_statustext_pack(system_id, component_id, &msg, severity, "R1 motor STUCK", id, chunk_seq);
     // Serial.println("                                                   R10  ");
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial1.write(buf, len);
    }
  }


  if (THRR2 > 1) {

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    if ((RPMR2 / RPMRAVG) < 0.9) {
      mavlink_msg_statustext_pack(system_id, component_id, &msg, severity, "R2 motor slow", id, chunk_seq);
     // Serial.println("                                                   R2S  ");
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial1.write(buf, len);
    }
    if ((RPMR2 / RPMRAVG) == 0) {
      mavlink_msg_statustext_pack(system_id, component_id, &msg, severity, "R2 motor STUCK", id, chunk_seq);
     // Serial.println("                                                   R20  ");
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial1.write(buf, len);
    }
  }


  if (THRR3 > 1) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    if ((RPMR3 / RPMRAVG) < 0.9) {
      mavlink_msg_statustext_pack(system_id, component_id, &msg, severity, "R3 motor slow", id, chunk_seq);
     // Serial.println("                                                   R3S  ");
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial1.write(buf, len);
    }
    if ((RPMR3 / RPMRAVG) == 0) {
      mavlink_msg_statustext_pack(system_id, component_id, &msg, severity, "R3 motor STUCK", id, chunk_seq);
     // Serial.println("                                                   R30  ");
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial1.write(buf, len);
    }
  }



  if (THRL1 > 1) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    if ((RPML1 / RPMLAVG) < 0.9) {
      mavlink_msg_statustext_pack(system_id, component_id, &msg, severity, "L1 motor slow", id, chunk_seq);
     // Serial.println("                                                   L1S  ");
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial1.write(buf, len);
    }
    if ((RPML1 / RPMLAVG) == 0) {
      mavlink_msg_statustext_pack(system_id, component_id, &msg, severity, "L1 motor STUCK", id, chunk_seq);
     // Serial.println("                                                    L10  ");
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial1.write(buf, len);
    }
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }


  if (THRL2 > 1) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    if ((RPML2 / RPMLAVG) < 0.9) {
      mavlink_msg_statustext_pack(system_id, component_id, &msg, severity, "L2 motor slow", id, chunk_seq);
     // Serial.println("                                                   L2S  ");
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial1.write(buf, len);
    }
    if ((RPML2 / RPMLAVG) == 0) {
      mavlink_msg_statustext_pack(system_id, component_id, &msg, severity, "L2 motor STUCK", id, chunk_seq);
     // Serial.println("                                                    L20  ");
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial1.write(buf, len);
    }
  }


  if (THRL3 > 1) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    if ((RPML3 / RPMLAVG) < 0.9) {
      mavlink_msg_statustext_pack(system_id, component_id, &msg, severity, "L3 motor slow", id, chunk_seq);
     // Serial.println("                                                   L3S  ");
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial1.write(buf, len);
    }
    if ((RPML3 / RPMLAVG) == 0) {
      mavlink_msg_statustext_pack(system_id, component_id, &msg, severity, "L3 motor STUCK", id, chunk_seq);
     // Serial.println("                                                    L30  ");
      uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial1.write(buf, len);
    }
  }
}
