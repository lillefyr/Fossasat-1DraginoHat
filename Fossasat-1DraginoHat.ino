///// lora part //////

#include <RadioLib.h>
// SX1278 has the following connections:
// NSS pin:   10
// DIO0 pin:  2
// DIO1 pin:  6  // Note: pin6 does not generate interrupt (pin3 does)
//                        So with Dragino it is necessary to poll for data
/* Board Arduino UNO */
SX1278 lora = new Module(10,2,6);

uint8_t keepAliveCounter=0;
uint16_t pwrcnt = 0; // value for now
uint16_t messageno = 0;
uint8_t downlink[85];
char tmpbuf2[65];
int16_t signedinteger;
float floatvalue;
uint8_t uplink[64];

String head;
String RSSI;
String SNR;
String ERR;

uint8_t alreadyTransmitted = 0;
///// lora part //////

#include "SoftwareSerial.h"
SoftwareSerial sw(3,4); // RX, TX
#include "SoftwareSerialStable.h"

const byte numChars = 65;
char receivedChars2[numChars];

//////////////// Satellite part /////////////

void SendCommandToSatellite(){
  for (uint8_t i = 0; i<63; i++) { receivedChars2[i] = 0x00; }
  int8_t datalen = SWSerialReceive(receivedChars2); // get command from lolin
  if ( datalen == 0 ) { Serial.println("no data"); return; }
  
#ifdef DEBUG
  Serial.print("Datalen=");
  Serial.println(datalen);
  for (uint8_t i = 0; i < datalen; i++ ) {
    Serial.print(receivedChars2[i],HEX); Serial.print(" ");
  }
  Serial.println();

  for (uint8_t i = 0; i < datalen; i++ ) {
    Serial.print(char(receivedChars2[i]));
  }
  Serial.println();
#endif

//  <callsign><function ID>(optional data length)(optional data)
  for (uint8_t i = 0; i < 63; i++ ) {
    uplink[i] = 0x00;
  }
  uplink[0]='F';
  uplink[1]='O';
  uplink[2]='S';
  uplink[3]='S';
  uplink[4]='A';
  uplink[5]='S';
  uplink[6]='A';
  uplink[7]='T';
  uplink[8]='-';
  uplink[9]='1';
  uplink[10]= receivedChars2[0] - 0x30;
  uplink[11]= 0x00;

  if ( uplink[10] > 0x04 ) {
    Serial.println("Invalid command");
    sprintf(tmpbuf2,"CMD:Invalid command");
    while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
  }
  else
  {
    if ( uplink[10] == 0x01 ) {
      uplink[11] = datalen;
      uint8_t i;
      // start from 1 and skip the command char
      for (i = 0; i < datalen+1; i++ ) {
        uplink[i+12] = receivedChars2[i+1];
      }
      uplink[i+12] = 0x00;
    }
    
    if ( uplink[10] == 0x02 ) {
      uplink[11]= 0x15; //optionalData[0]
      uplink[12]= 0x07; //optionalData[1]
      uplink[13]= 0x0C; //optionalData[2]
      uplink[14]= 0x06; //optionalData[3]
      uplink[15]= 0x10; //optionalData[4]
      uplink[16]= 0x01; //optionalData[5]
      uplink[17]= 0x0F; //optionalData[6]
      uplink[18] = datalen + 7;
  
      uint8_t i;
      // start from 1 and skip the command char
      for (i = 1; i < datalen+1; i++ ) {
        uplink[i+19] = receivedChars2[i];
      }
      uplink[i+19] = 0x00;
    }
    Serial.println();
    Serial.print("Send command = 0x0");
    Serial.println(uplink[10], HEX);
  
    for (uint8_t i = 0; i<63; i++) {
#ifdef DEBUG
      if (isGraph(uplink[i])) {
        Serial.print(char(uplink[i]));
      } else {
        if (( i > 1 ) && ( uplink[i] > 0x00 )) {
          Serial.print(" 0x");
          Serial.print(uplink[i],HEX);
          Serial.print(" "); 
        }
      }
#endif
    }

    for (uint8_t i = 0; i<63; i++) { Serial.print(uplink[i],HEX); Serial.print(" "); }
    Serial.println();
  
    int state = lora.transmit(uplink, 63);
  
    if (state == ERR_NONE) {
      // the packet was successfully transmitted
      //Serial.println(F(" success!"));
  
      // print measured data rate
      Serial.print(F("[SX1278] Datarate:\t"));
      Serial.print(lora.getDataRate());
      Serial.println(F(" bps"));
      
//      sprintf(tmpbuf2,"CMD:OK");
//      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
  
    } else if (state == ERR_PACKET_TOO_LONG) {
      // the supplied packet was longer than 256 bytes
      Serial.println(F(" too long!"));
      sprintf(tmpbuf2,"CMD:ERR_PACKET_TOO_LONG");
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
  
    } else if (state == ERR_TX_TIMEOUT) {
      // timeout occured while transmitting packet
      Serial.println(F(" timeout!"));
      sprintf(tmpbuf2,"CMD:ERR_TX_TIMEOUT");
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
  
    } else if (state == ERR_SPI_WRITE_FAILED) {
      // Real value in SPI register does not match the expected one.
      Serial.println(F(" SPI write failed!"));
      sprintf(tmpbuf2,"CMD:ERR_SPI_WRITE_FAILED");
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
  
    } else {
      // some other error occurred
      Serial.print(F("failed, code "));
      Serial.println(state);
      sprintf(tmpbuf2,"CMD:ERR=%d\n", state);
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
    }
  } // else invalid command
}

//////////////// Satellite part /////////////

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("Fossasat-1DraginoHat"));
  sw.begin(115200);
  
    // initialize SX1278 with default settings
  Serial.print(F("[SX1278] Initializing ... "));
  float CARRIER_FREQUENCY = 436.7f;
  float DEFAULT_CARRIER_FREQUENCY = 436.7f;
  float BANDWIDTH = 125.0f;
  float CONNECTED_BANDWIDTH = 7.8f;
  int   SPREADING_FACTOR = 11;  // defined by fossasat-1
  int   CODING_RATE = 8; // can be 4 or 8.
  char  SYNC_WORD = 0xff;  //0f0f for SX1262 (effing doesn't work)
  int   OUTPUT_POWER = 17; // dBm
  // current limit:               100 mA
  // preamble length:             8 symbols
  // amplifier gain:              0 (automatic gain control)

  int state = lora.begin(DEFAULT_CARRIER_FREQUENCY, BANDWIDTH, SPREADING_FACTOR, CODING_RATE, SYNC_WORD, OUTPUT_POWER, 100, 8, 0);
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
}


void loop() { 

  // this is the satellite radio
  char messageBuffer[65] = { 0x06, 0x4F, 0x53, 0x53, 0x41, 0x53, 0x41, 0x54, 0x2D, 0x31, 0x13, 0x28, 0x78, 0x1E, 0x21, 0x1C, 0xD2, 0x04, 0x3C, 0x04, 0x7D, 0x2F, 0x00 };
//13 0F C4 F1 03 C4 00 00 00 90 01 56 FA 4C 09 00 16
  
/*  
  for (uint8_t i = 0; i<63; i++) { receivedChars2[i] = 0x00; }
  int8_t len = SWSerialReceive(receivedChars2); // from lolin
  if ( len > 0 ){
    Serial.println(receivedChars2);
  }
*/
  
/*  String startup = "Welcome to the wonderworld\n";
//                           1         2         3         4         5         6
//                  12345678901234567890123456789012345678901234567890123456789012
//  String startup = "abcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXYZ";

 for (int i=0; i<startup.length(); i++) {
    messageBuffer[i] = startup[i];
    messageBuffer[i+1] = 0x00;
  }
  if ( timeout + 15000 < millis()) {
    Serial.print("Dragino hat sends =>");
    Serial.println(messageBuffer);
    timeout = millis();
    while ( SWSerialTransmit(messageBuffer) == false ) { yield(); }
    Serial.print("message time=");
    Serial.println(millis() - timeout);
  }
*/

  for (int i=0; i<64; i++) { downlink[i] = 0x00; }
  int state = lora.receive((uint8_t *)downlink, 63);

  if (state == ERR_NONE) {

    // RSSI (Received Signal Strength Indicator)
    RSSI = lora.getRSSI();

    // SNR (Signal-to-Noise Ratio)
    SNR = lora.getSNR();

    // frequency error
    ERR = lora.getFrequencyError();

    Serial.println("downlink from satellite");
    for (uint8_t i = 0; i<64; i++) {
      Serial.print("%0x"); Serial.print(downlink[i],HEX); Serial.print(" ");
    }
    Serial.println();
   
    // format the downlink in packages
    for ( uint8_t i = 0; i < 64; i++ ) { tmpbuf2[i] = 0x00; }

    if ( downlink[10] == 0x10 ) { // RESP_PONG
      tmpbuf2[0] = 'I';
      tmpbuf2[1] = 'D';
      tmpbuf2[2] = '1';
      tmpbuf2[3] = '0';
      tmpbuf2[4] = ':';
      tmpbuf2[5] = 0x00;
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
      Serial.println(tmpbuf2);
      keepAliveCounter = 0;
    }
    
    if ( downlink[10] == 0x11 ) { // RESP_REPEATED_MESSAGE

      uint8_t datalen = downlink[11];
      uint8_t i;

      tmpbuf2[0] = 'I';
      tmpbuf2[1] = 'D';
      tmpbuf2[2] = '1';
      tmpbuf2[3] = '1';
      tmpbuf2[4] = ':';

      for (i = 0; i < datalen; i++) {
        Serial.print(downlink[i+12], HEX);
        Serial.print(" ");
        tmpbuf2[i+5] = downlink[i+12];
        tmpbuf2[i+6] = 0x00;
      }
      tmpbuf2[i+5] = '\n';
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };  // send the message
      keepAliveCounter = 0;
    }
    
    if ( downlink[10] == 0x12 ) { // RESP_REPEATED_MESSAGE_CUSTOM

      uint8_t datalen = downlink[11];
      uint8_t i;

      tmpbuf2[0] = 'I';
      tmpbuf2[1] = 'D';
      tmpbuf2[2] = '1';
      tmpbuf2[3] = '2';
      tmpbuf2[4] = ':';

      for (i = 0; i < datalen; i++) {
        Serial.print(downlink[i+12], HEX);
        Serial.print(" ");
        tmpbuf2[i+5] = downlink[i+12];
        tmpbuf2[i+6] = 0x00;
      }
      tmpbuf2[i+5] = '\n';
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };  // send the message
      keepAliveCounter = 0;
    }

    if ( downlink[10] == 0x13 ) { // RESP_SYSTEM_INFO

      sprintf(tmpbuf2, "ID13:%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x",
      downlink[11], downlink[12], downlink[13], downlink[14], downlink[15],
      downlink[16], downlink[17], downlink[18], downlink[19], downlink[20],
      downlink[21], downlink[22], downlink[23], downlink[24], downlink[25],
      downlink[26]);
      Serial.println(tmpbuf2);
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
/*
      sprintf(tmpbuf2,"BV:%d\n", downlink[11]); // battery charging voltage 250 * 20 mV   

      // battery charging current * 10uA
      signedinteger = downlink[12] + downlink[13]*0x100;
      sprintf(tmpbuf2,"BC:%d\n", signedinteger);
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };

      sprintf(tmpbuf2,"SA:%d\n", downlink[14]); // solar cell A voltage 200 * 20 mV   
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
      sprintf(tmpbuf2,"SB:%d\n", downlink[15]); // solar cell B voltage 220 * 20 mV   
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
      sprintf(tmpbuf2,"SC:%d\n", downlink[16]); // solar cell C voltage 180 * 20 mV   
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
      
      // board temp * 0.01 C
      signedinteger = downlink[19] + downlink[20]*0x100;
      sprintf(tmpbuf2,"BOT:%d\n", signedinteger);
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
      
      // battery temp * 0.01 C
      signedinteger = downlink[17] + downlink[18]*0x100;
      sprintf(tmpbuf2,"BAT:%d\n", signedinteger);
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };

      sprintf(tmpbuf2,"MCT:%d\n", downlink[21]); // mcu temp *0.1 C   
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };

      // reset keepAliveCounter
      signedinteger = downlink[22] + downlink[23]*0x100;
      sprintf(tmpbuf2,"RC:%d\n", signedinteger);
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
      
      sprintf(tmpbuf2,"PWR:%d\n", downlink[24]); //power control
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
    */

      strcpy(tmpbuf2, "RSSI:");
      dtostrf((RSSI.toFloat()), 2, 2, &tmpbuf2[strlen(tmpbuf2)]);
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
      
      strcpy(tmpbuf2, "SNR:");
      dtostrf((SNR.toFloat()), 2, 2, &tmpbuf2[strlen(tmpbuf2)]);
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
      
      strcpy(tmpbuf2, "ERR:");
      dtostrf((ERR.toFloat()), 2, 2, &tmpbuf2[strlen(tmpbuf2)]);
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
      keepAliveCounter = 0;
    } // if downlink[10] = 0x13

    if ( downlink[10] == 0x14 ) { // RESP_LAST_PACKET_INFO // optionalDataLen = 2
      sprintf(tmpbuf2, "ID14:%02x,%02x", downlink[11], downlink[12]);
      /*
      sprintf(tmpbuf2,"PSNR:%d\n", downlink[11]); // optionalData[0] = SNR * 4 dB
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
      sprintf(tmpbuf2,"PRSSI:%d\n", downlink[12]); // optionalData[1] = RSSI * -2 dBm
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
      head = "GO:14";
      head.toCharArray(tmpbuf2, head.length()+1);
      */
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); }; // all available data handled
      keepAliveCounter = 0;
    }
    // end of transfer

  } else if (state == ERR_CRC_MISMATCH) {
    // packet was received, but is malformed
    Serial.println(F("[SX1278] CRC error!"));

  } else if (state ==  ERR_RX_TIMEOUT) {
    // got an interrupt, but no data
    //Serial.println(F("[SX1278] timeout"));
  } else {
    // some other error occurred
    Serial.print(F("[SX1278] Failed, code "));
    Serial.println(state);
  }

  if (keepAliveCounter > 60) { // every 5 minutes
      String keepalive="KEEPALIVE:OK\n";
      keepalive.toCharArray(tmpbuf2, keepalive.length()+1);
      while ( SWSerialTransmit(tmpbuf2)  == false ) { yield(); };
      keepAliveCounter = 0;
  }
  keepAliveCounter++;

  if (sw.available()) {
    Serial.print("Command from Fossasat-1Lolin");
    SendCommandToSatellite();
  }
  /* no asbjorn */
}
