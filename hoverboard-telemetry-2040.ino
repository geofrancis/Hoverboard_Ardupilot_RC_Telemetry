
#include "mavlink/common/mavlink.h"
#include "mavlink/common/mavlink_msg_servo_output_raw.h"


SerialPIO HoverSerial1(2, 3);
SerialPIO HoverSerial2(4, 5);
SerialPIO HoverSerial3(6, 7);

#define START_FRAME 0xABCD  // [-] Start frme definition for reliable serial communication
#define DEBUG_RX            // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

int leftoutput = 0;
int rightoutput = 0;






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




void setup() {
  Serial.begin(115200);   // USB
  Serial1.begin(115200);  // 0,1
  Serial2.begin(115200);  // 8,9

  HoverSerial1.begin(115200);
  HoverSerial2.begin(115200);
  HoverSerial3.begin(115200);

  request_Mavlink();
}






void loop() {

  MavLink_RC();
  Mavlink_Telemetry();  

  Receive1();
  Receive2();
  Receive3();


}
