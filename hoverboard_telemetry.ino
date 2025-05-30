
#include "mavlink/common/mavlink.h"
#include "mavlink/common/mavlink_msg_servo_output_raw.h"
//#include "mavlink/common/mavlink_msg_esc_status.h"


SerialPIO HoverSerial1(2, 3);
SerialPIO HoverSerial2(4, 5);
SerialPIO HoverSerial3(6, 7);
SerialPIO HoverSerial4(10, 11);

int DZ = 20; // dead zone

int brd1 = 0;
int brd2 = 0;
int brd3 = 0;

int FCHB = 0;
int FCOK = 0;

int brd1w = 0;
int brd2w = 0;
int brd3w = 0;
int brd1p = 0;
int brd2p = 0;
int brd3p = 0;
int brdall = 1;
int brdauto = 0;
int brdallc = 1;
int brdalld = 1;
int MAXRPM = 100;

int powerchannel = 1000;

const int board1power = 12;
const int board2power = 13;
const int board3power = 14;

const int board1switch = 15;
const int board2switch = 26;
const int board3switch = 27;

const int mainswitch = 28;

#define START_FRAME 0xABCD  // [-] Start frme definition for reliable serial communication
//#define DEBUG_RX            // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

int leftoutput = 0;
int rightoutput = 0;

//low voltage
//speed limit
//power control

int boardson = 0;
int board1on = 0;
int board2on = 0;
int board3on = 0;

int board1error = 0;
int board2error = 0;
int board3error = 0;

int startupok = 0;
int startcycle1 = 0;
int startcycle2 = 0;
int startcycle3 = 0;
int stopcycle = 0;

unsigned long buttonMillis = 0;
unsigned long current_motor_esc_Millis = 0;

unsigned long previous_motor_esc_Millis = 0;

unsigned long checkMillis = 10000;
unsigned long check2Millis = 10000;
unsigned long check3Millis = 10000;

const long buttoninterval = 2000;  // time to hold power switch
const long boardinterval = 4000;   // time to hold power switch

unsigned long previousMillis = 0;  // will store last time LED was updated
const long telem = 1000;
unsigned long previousMillis2 = 0;  // will store last time LED was updated
const long telem2 = 3000;

unsigned long btnm = 0;  // will store last time LED was updated

int BASEMODE = 0;
int armed;
int active = 0;
int THRR1;
int THRL1;
int THRR2;
int THRL2;
int THRR3;
int THRL3;

//wheel RPM
int RPMR1;
int RPML1;
int RPMR2;
int RPML2;
int RPMR3;
int RPML3;

//voltage and temperature
int VOLT1;
int TEMP1;
int VOLT2;
int TEMP2;
int VOLT3;
int TEMP3;
int TEMPMAX;
float RPMLAVG;
float RPMRAVG;

float roll = 0;
float pitch = 0;

uint8_t idx = 0;         // Index for new data pointer
uint16_t bufStartFrame;  // Buffer Start Frame
byte *p;                 // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct {
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct {
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

int DI1O = 0;
int set;
int set1 = 0;
int set2 = 0;
int set3 = 0;

void setup() {
  delay(5000);
  Serial.begin(115200);   // USB
  Serial1.begin(500000);  // 0,1
  Serial2.begin(115200);  // 8,9

  HoverSerial1.begin(115200);
  HoverSerial2.begin(115200);
  HoverSerial3.begin(115200);
  //BMS.begin(115200);


  pinMode(board1power, INPUT);
  pinMode(board2power, INPUT);
  pinMode(board3power, INPUT);
  pinMode(mainswitch, INPUT);

  pinMode(board1switch, OUTPUT);
  pinMode(board2switch, OUTPUT);
  pinMode(board3switch, OUTPUT);

  //request_Mavlink();

  uint8_t system_id = 1;
  uint8_t component_id = 158;
  uint8_t severity = 1;
  uint16_t id = 0;
  uint8_t chunk_seq = 0;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_statustext_pack(system_id, component_id, &msg, 2, "HOVERBOARD STARTUP", id, chunk_seq);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);

  if (digitalRead(board1power)) digitalWrite(board1switch, HIGH);
  if (digitalRead(board2power)) digitalWrite(board2switch, HIGH);
  if (digitalRead(board3power)) digitalWrite(board3switch, HIGH);
  delay(2000);
  digitalWrite(board1switch, LOW);
  digitalWrite(board2switch, LOW);
  digitalWrite(board3switch, LOW);
  delay(2000);

  request_Mavlink();

  mavlink_msg_statustext_pack(system_id, component_id, &msg, 2, "HOVERBOARD CONTROLLER ONLINE", id, chunk_seq);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}



void setup1() {}


void loop() {

  //Serial.println(" loop------------------------------------------------------------------------------------");

  Receive1();
  Receive2();
  Receive3();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= telem) {
    //Serial.println(" loop111------------------------------------------------------------------------------------");
    previousMillis = currentMillis;
    MAVLINK_HB();
    MAVLINK_HB1();
    MAVLINK_HB2();
    MAVLINK_HB3();
    if (DI1O == 1) { Mavlink_Telemetry1(); }
    if (DI1O == 2) { power(); }
    if (DI1O == 3) { Mavlink_Telemetry2(); }
    if (DI1O == 4) { FCHBC(); }
    if (DI1O == 5) { Mavlink_Telemetry3(); }
    if (DI1O == 6) { warning(); }
    DI1O++;
    if (DI1O > 6) { DI1O = 1; }
    Serial.print("                                   DI1O  ");
    Serial.println(DI1O);


}
}


void loop1() {
  MavLink_RC();
}
