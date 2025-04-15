



void BMS(){
  Serial.println(".");
  Serial.println(".");
  Serial.println(".");
  write_request_start();// Found this helps timing issue, by saying hello, hello.
  write_request_end() ; // Or maybe it flushes out any rogue data.
  write_request_start();// Any way it works,
  write_request_end() ; // And accomodates long delays if you want them at the end
  
  // CELLS VOLTAGE 04 /////////////////////////////////////////////////////////////////////////////
  call_get_cells_v();      // requests cells voltage
  get_bms_feedback();     // returns with up to date, inString= chars, inInts[]= numbers, chksum in last 2 bytes
  //                       Length (length of data string)
  //  got cell voltages, bytes 0 and 1, its 16 bit, high and low
  //  go through and print them
  // Length = Length - 2;
  // print headings
  for (int i = 2; i < (Length + 1); i = i + 2) {
    Serial.print (" Cell ");
    Serial.print (i / 2);
    Serial.print("  ");
  }
  Serial.print (" CellMax "); // CellMax heading
  Serial.print("  ");
  Serial.print (" CellMin "); // CellMin heading
  Serial.print("  ");
  Serial.print (" Diff "); // diference heading
  Serial.print("  ");
  Serial.print ("  Avg "); // Average heading
  Serial.print("  ");
  myNumCells = Length/2;
  Serial.print (" NumCells:");
  Serial.print (myNumCells);
  Serial.println ("");
  // and the values
  Cellavg = 0;
  CellMax = 0;
  CellMin = 5;
  for (int i = 0; i < Length; i = i + 2) {
    highbyte = (inInts[i]);
    lowbyte = (inInts[i + 1]);
    uint16_t Cellnow = two_ints_into16(highbyte, lowbyte);
    float Cellnowf = Cellnow / 1000.0f; // convert to float
    myCellVoltages[i/2] = Cellnowf; // Update value in array
    Cellavg = Cellavg + Cellnowf;
    if (Cellnowf > CellMax) {   // get high and low
      CellMax = Cellnowf;
    }
    if (Cellnowf < CellMin) {
      CellMin = Cellnowf;
    }
    Serial.print(" ");
    Serial.print(Cellnowf, 3); // 3 decimal places
    Serial.print("   ");
  }
  Serial.print(" ");
  Serial.print(CellMax, 3); // 3 decimal places
  Serial.print("   ");
  Serial.print("   ");
  Serial.print(CellMin, 3); // 3 decimal places
  Serial.print("   ");
  Celldiff = CellMax - CellMin; // difference between highest and lowest
  Serial.print("   ");
  Serial.print(Celldiff, 3); // 3 decimal places
  Serial.print("   ");
  Cellavg = Cellavg / (Length / 2); // Average of Cells
  Serial.print(" ");
  Serial.print(Cellavg, 3); // 3 decimal places
  Serial.print("   ");


  // USING BASIC INFO 03 get //////////////////////////////////////////////////////////////////////
  //  CELL BALANCE... info
  call_Basic_info();      // requests basic info.
  get_bms_feedback();   // get that data, used to get BALANCE STATE byte 17 less 4, decimal=byte 13
  //  Serial.print(" BC= ");
  BalanceCode = inInts[13]; //  the 13th byte
  BalanceCode = Bit_Reverse( BalanceCode ) ; // reverse the bits, so they are in same order as cells
  //  Serial.print(BalanceCode, BIN); // works, but, loses leading zeros and get confusing on screen
  print_binary(BalanceCode, 8);// print balance state as binary, cell 1 on the right, cell 8 on left
  //                                    Reversed this. 1 on left, 8 on right
  Serial.print ("  Balancer States ");
  Serial.println(" ");
  
  // PACK VOLTAGE,, bytes 0 and 1, its 16 bit, high and low
  highbyte = (inInts[0]); // bytes 0 and 1
  lowbyte = (inInts[1]);
  uint16_t PackVoltage = two_ints_into16(highbyte, lowbyte);
  PackVoltagef = PackVoltage / 100.0f; // convert to float and leave at 2 dec places
  Serial.print("Pack Voltage = ");
  Serial.print(PackVoltagef);

  // CURRENT
  highbyte = (inInts[2]); // bytes 2 and 3
  lowbyte = (inInts[3]);
  int PackCurrent = two_ints_into16(highbyte, lowbyte);
  // uint16_t PackCurrent = two_ints_into16(highbyte, lowbyte);
  PackCurrentf = PackCurrent / 100.0f; // convert to float and leave at 2 dec places
  Serial.print("   Current = ");
  Serial.print(PackCurrentf);

  //REMAINING CAPACITY
  highbyte = (inInts[4]);
  lowbyte = (inInts[5]);
  uint16_t RemainCapacity = two_ints_into16(highbyte, lowbyte);
  RemainCapacityf = RemainCapacity / 100.0f; // convert to float and leave at 2 dec places
  Serial.print("   Remaining Capacity = ");
  Serial.print(RemainCapacityf);
  Serial.print("Ah");

  //RSOC
  RSOC = (inInts[19]);
  Serial.print("   RSOC = ");
  Serial.print(RSOC);
  Serial.print("%");

  //Temp probe 1
  highbyte = (inInts[23]);
  lowbyte = (inInts[24]);
  float Temp_probe_1 = two_ints_into16(highbyte, lowbyte);
  Temp_probe_1f = (Temp_probe_1 - 2731) / 10.00f; // convert to float and leave at 2 dec places
  Serial.println("");
  Serial.print("Temp probe 1 = ");
  Serial.print(Temp_probe_1f);
  Serial.print(" ");

  //Temp probe 2
  highbyte = (inInts[25]);
  lowbyte = (inInts[26]);
  float Temp_probe_2 = two_ints_into16(highbyte, lowbyte);
  Temp_probe_2f = (Temp_probe_2 - 2731) / 10.00f; // convert to float and leave at 2 dec places
  Serial.print("   Temp probe 2 = ");
  Serial.print(Temp_probe_2f);
  Serial.println(" ");

  // Mosfets
  chargeFet = inInts[20] & 1; //bit0
  dischargeFet = (inInts[20] >> 1) & 1; //bit1
  Serial.print(F("Mosfet Charge = "));
  Serial.print(chargeFet);
  Serial.print(F("  Mosfet DisCharge = "));
  Serial.println(dischargeFet);

  // Pack Protection states
  cellOver = (inInts[17] >> 0) & 1; 
  cellUnder = (inInts[17] >> 1) & 1; 
  packOver = (inInts[17] >> 2) & 1; 
  PackUnder = (inInts[17] >> 3) & 1; 
  chargeOverTemp = (inInts[17] >> 4) & 1; 
  chargeUnderTemp = (inInts[17] >> 5) & 1; 
  dischargeOverTemp = (inInts[17] >> 6) & 1; 
  dischargeUnderTemp = (inInts[17] >> 7) & 1; 
  chargeOvercurrent = (inInts[16] >> 0) & 1; 
  dischargeOvercurrent = (inInts[16] >> 1) & 1; 
  shortCircuit = (inInts[16] >> 2) & 1; 
  AFEerror = (inInts[16] >> 3) & 1; 

  Serial.print("Protection Status: ");
  Serial.print(cellOver);
  Serial.print(cellUnder);
  Serial.print(packOver);
  Serial.print(PackUnder);
  Serial.print(chargeOverTemp);
  Serial.print(chargeUnderTemp);
  Serial.print(dischargeOverTemp);
  Serial.print(dischargeUnderTemp);
  Serial.print(chargeOvercurrent);
  Serial.print(dischargeOvercurrent);
  Serial.print(shortCircuit);
  Serial.print(AFEerror);
  Serial.println("");
  
  inString = "";
  Length = 0;
}

//------------------------------------------------------------------------------------------
// Do not edit anything below this line
//------------------------------------------------------------------------------------------
//  uint16_t PackCurrent = two_ints_into16(highbyte, lowbyte);
uint16_t two_ints_into16(int highbyte, int lowbyte) // turns two bytes into a single long integer
{
  a16bitvar = (highbyte);
  a16bitvar <<= 8; //Left shift 8 bits,
  a16bitvar = (a16bitvar | lowbyte); //OR operation, merge the two
  return a16bitvar;
}
// ----------------------------------------------------------------------------------------------------
void call_Basic_info()
// total voltage, current, Residual capacity, Balanced state, MOSFET control status
{
  flush(); // flush first

  //  DD  A5 03 00  FF  FD  77
  // 221 165  3  0 255 253 119
  uint8_t data[7] = {221, 165, 3, 0, 255, 253, 119};
  Serial2.write(data, 7);
}
//--------------------------------------------------------------------------
void call_get_cells_v()
{
  flush(); // flush first

  // DD  A5  4 0 FF  FC  77
  // 221 165 4 0 255 252 119
  uint8_t data[7] = {221, 165, 4, 0, 255, 252, 119};
  Serial2.write(data, 7);
}
//--------------------------------------------------------------------------
void call_Hardware_info()
{
  flush(); // flush first

  //  DD  A5 05 00  FF  FB  77
  // 221 165  5  0 255 251 119
  uint8_t data[7] = {221, 165, 5, 0, 255, 251, 119};
  // uint8_t data[7] = {DD, A5, 05, 00, FF, FB, 77};
  Serial2.write(data, 7);
}
//--------------------------------------------------------------------------
void write_request_start()
{
  flush(); // flush first

  //   DD 5A 00  02 56  78  FF 30   77
  uint8_t data[9] = {221, 90, 0, 2, 86, 120, 255, 48, 119};
  Serial2.write(data, 9);
}
//----------------------------------------------------------------------------
void write_request_end()
{
  flush(); // flush first

  //   DD 5A 01  02 00 00   FF  FD 77
  uint8_t data[9] = {221, 90, 1, 2, 0, 0, 255, 253, 119};
  Serial2.write(data, 9);
}
//-------------------------------------------------------------------------
void eprom_read()   //BAR CODE
{
  flush(); // flush first
  //delay(5);
  // SENT CODE depends on WHAT IS REQD???
  //   DD  A5  A2 0  FF 5E  77...BAR CODE
  //  221 165 162 0 255 94 119
  // uint8_t data[7] = {221, 165, 162, 0, 255, 94, 119};
  uint8_t data[7] = {221, 165, 32, 0, 255, 224, 119};
  Serial2.write(data, 7);
}

//-------------------------------------------------------------------------
void eprom_end() // no need at mo
{
  flush(); // flush first
  // delay(5);
  //DD  A5  AA  0 FF  56  77
  //221 165 170 0 255 86  119
  // from eprom read
  uint8_t data[7] = {221, 165, 170, 0, 255, 86, 119};
  Serial2.write(data, 7);
}
//------------------------------------------------------------------------------
void flush()
{ // FLUSH
  delay(100); // give it a mo to settle, seems to miss occasionally without this
  while (Serial2.available() > 0)
  { Serial2.read();
  }
  delay(50); // give it a mo to settle, seems to miss occasionally without this
}
//--------------------------------------------------------------------------
void get_bms_feedback()  // returns with up to date, inString= chars, inInts= numbers, chksum in last 2 bytes
//                          Length
//                          Data only, exclude first 3 bytes
{
  inString = ""; // clear instring for new incoming
  delay(100); // give it a mo to settle, seems to miss occasionally without this
  if (Serial2.available() > 0) {
    {
      for (int i = 0; i < 4; i++)               // just get first 4 bytes
      {
        incomingBytes = Serial2.read();
        if (i == 3)
        { // could look at 3rd byte, it's the ok signal
          Length = (incomingBytes); // The fourth byte holds the length of data, excluding last 3 bytes checksum etc
          // Serial.print(" inc ");
          //Serial.print(incomingBytes);
        }
        if (Length == 0) {
          Length = 1; // in some responses, length=0, dont want that, so, make Length=1
        }
      }
      //  Length = Length + 2; // want to get the checksum too, for writing back, saves calculating it later
      for (int i = 0; i < Length + 2; i++) { // get the checksum in last two bytes, just in case need later
        incomingBytes = Serial2.read(); // get the rest of the data, how long it might be.
        inString += (char)incomingBytes; // convert the incoming byte to a char and add it to the string
        inInts[i] = incomingBytes;       // save incoming byte to array as int
      }
    }
  }
}
//-----------------------------------------------------------------------------------------------------
void print_binary(int v, int num_places) // prints integer in binary format, nibbles, with leading zeros
// altered a bit, but got from here,  https://phanderson.com/arduino/arduino_display.html
{
  Serial.println("");
  Serial.print("  ");
  int mask = 0, n;
  for (n = 1; n <= num_places; n++)
  {
    mask = (mask << 1) | 0x0001;
  }
  v = v & mask;  // truncate v to specified number of places
  int cellNum = 0;
  while (num_places)
  {
    if (v & (0x0001 << num_places - 1))
    {
      Serial.print("1        ");
      balancerStates[cellNum] = 1;
    }
    else
    {
      Serial.print("0        ");
      balancerStates[cellNum] = 0;
    }
    --num_places;
    if (((num_places % 4) == 0) && (num_places != 0))
    {
      Serial.print("");
    }
    cellNum++;
  }
}
//-----------------------------------------------------------------------------------------------------
byte Bit_Reverse( byte x )
// http://www.nrtm.org/index.php/2013/07/25/reverse-bits-in-a-byte/
{
  //          01010101  |         10101010
  x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
  //          00110011  |         11001100
  x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
  //          00001111  |         11110000
  x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
  return x;
}