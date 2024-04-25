//Use 3rd Mode to
//Read stable serial PWM from Rasp Pi for steering in Mode 3.  Speed is manual.

//Drive Rear Wheels and Steering Motors using
//hardware serial to run ibus with FS-i6 radio
//  May 26, 2022 turn, drive forward/backward
//  Add: car remote turning through relays 5&6
//  June 6, 2022 (B) steering Potentiometer
//  Aug 28, 2022 Run robot with Flysky on both robots
//  Jun 8, 2023  added getLong()
//  Jun 19,2023  Lowered steering gain from 30 to 8
//  Add: (A) Pot_InterruptA wheel Miller sensor
//  Add: drive straight, zero turn angle means drive straight
//      How much turn for a given interrupt difference
//  Add: rear wheel speed ratio based on turn angle
//  Dec 11, 2023: dead reconning
// .       Serial.print time, Transmitter turn angle, potentiometer, contrL, cntrR
//  Add: display speed, angle turn, position, battery voltage
//  Add: Drive straight potentiometer printout
//  Change: Steering Gain
//  Add: steering null zone
//  Add: GPS dead rockoning steering
/*
  Using Arduino FS-I6X
    Channel 1  > 1500 right turn, <1500 left turn
    Channel 3 (Speed)
    Channel 5 forward, backward SWA
    Channel 6  (3 position SWC)
  fsi6x-arduino-mega-ibus.ino
  Read iBus output port from FS-IA10B receiver module

  //AT home:Pot  Mod 1   Mod 2
//    LEFT MAX is 300     300
//    Center   is 502     502
//    Right MX is 716     720
*/
int nullCounter = 0;
const int BF_Len = 20;                //Serial3 Buffer Length
const double driveDist = 1000.;    //Mode 3 drive dist (inches)
const double steerKp = 15.;     // turning proportional gain(8 is good value)
//MODE 3 drive straight constants
const double STGain = 50.;
const long GO = 4000000L, DR = 1000000L;   //4 second acceleration time (mode3)
const long ST = 4000000L;   //12 seconds full speed, and 4 seconds stop time
#define Miami

#ifdef Miami
const int revTurn = 0;    //turn angle reversed from transmitter
const int Poten = A13;
const int LIntPin = 0;
const int RIntPin = 1;

const int ST_RLPin = 7;    // Steering Right/Left
const int ST_PWM_Pin = 6;  // Steering PWM
const int FB_R_Pin = 4;    // Forward/Backward Right Pin
const int FB_R_PWM = 5;    // Speed Right PWM Pin
const int FB_L_Pin = 12;   // Forward/Backward Left Pin
const int FB_L_PWM = 10;   // Speed Left PWM Pin

byte channelNoA = 0;    //turn
byte channelNoB = 4;    //3 possition switch
byte channelNoC = 2;    //speed
byte channelNoD = 5;    //forward/ backward switch
byte channelNoE = 1;    //reset counters if == 250
// Note IBusBM library labels channels starting with "0"

// Relay pins
const int RelayPin8 = A6;
const int RelayPin6 = A5;
const int RelayPin5 = A4;
const int RelayPin4 = A3;
const int RelayPin3 = A2;
const int RelayPin2 = A1;
const int RelayPin1 = A0;

//Steering values
const int Ladc = 471;       //Left limit
const int Radc = 551;       //Right limit
const int CenterAdc = 511;  //Stra8ght value
const double steerA = CenterAdc;    //Ladc = steerA - steerB*tan(60)
const double steerB = (Radc-CenterAdc)/1.7321;
//const double steerB = 23.094;     //Radc = steerA + steerB*tan(60)

// Gps PVT
const char UBLOX_INIT[] PROGMEM = {
  // Disable NMEA
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off

  // Disable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC, //NAV-PVT off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9, //NAV-POSLLH off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0, //NAV-STATUS off

  // Enable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on

  // Rate
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)
};

const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

struct NAV_PVT {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;          // GPS time of week of the navigation epoch (ms)
  
  unsigned short year;         // Year (UTC) 
  unsigned char month;         // Month, range 1..12 (UTC)
  unsigned char day;           // Day of month, range 1..31 (UTC)
  unsigned char hour;          // Hour of day, range 0..23 (UTC)
  unsigned char minute;        // Minute of hour, range 0..59 (UTC)
  unsigned char second;        // Seconds of minute, range 0..60 (UTC)
  unsigned char valid;                  // Validity Flags (see graphic below)
  unsigned long tAcc;          // Time accuracy estimate (UTC) (ns)
  long nano;                   // Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
  unsigned char fixType;       // GNSSfix Type, range 0..5
  unsigned char flags;                  // Fix Status Flags
  unsigned char flags2;     // reserved
  unsigned char numSV;         // Number of satellites used in Nav Solution
  
  long lon;                    // Longitude (deg)
  long lat;                    // Latitude (deg)
  long height;                 // Height above Ellipsoid (mm)
  long hMSL;                   // Height above mean sea level (mm)
  unsigned long hAcc;          // Horizontal Accuracy Estimate (mm)
  unsigned long vAcc;          // Vertical Accuracy Estimate (mm)
  
  long velN;                   // NED north velocity (mm/s)
  long velE;                   // NED east velocity (mm/s)
  long velD;                   // NED down velocity (mm/s)
  long gSpeed;                 // Ground Speed (2-D) (mm/s)
  long headMot;                // Heading of motion 2-D (deg)
  unsigned long sAcc;          // Speed Accuracy Estimate
  unsigned long headAcc;    // Heading Accuracy Estimate
  unsigned short pDOP;         // Position dilution of precision
  short flags3;             // Reserved
  unsigned long reserved1;     // Reserved
  // Zack added
  long headVeh;
  short magDec;
  unsigned short magAcc;
};

NAV_PVT pvt;

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_PVT); i++) {
    CK[0] += ((unsigned char*)(&pvt))[i];
    CK[1] += CK[0];
  }
}

bool processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_PVT);

  while ( Serial2.available() ) {
    byte c = Serial2.read();
    if ( fpos < 2 ) {
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0;
    }
    else {      
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&pvt))[fpos-2] = c;

      fpos++;

      if ( fpos == (payloadSize+2) ) {
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize+3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize+4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          return true;
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        fpos = 0;
      }
    }
  }
  return false;
}

long lat;
long lng;
float heading;
int hour;
int minute;
int second;

int X_mid = 320; // Middle of the camera picture where a person is


#elif defined Home
const int revTurn = 0;    //turn angle not reversed from transmitter
const int Poten = A0;
const int LIntPin = 0;
const int RIntPin = 1;
const int ST_RLPin = 4;    // Steering Right/Left
const int ST_PWM_Pin = 5;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            // Steering PWM
const int FB_L_Pin = 8;    // Forward/Backward Right Pin
const int FB_L_PWM = 9;    // Speed Right PWM Pin
const int FB_R_Pin = 12;   // Forward/Backward Left Pin
const int FB_R_PWM = 10;   // Speed Left PWM Pin
const int RelayPin8 = 22;

byte channelNoA = 0;    //turn
byte channelNoB = 5;    //3 possition switch
byte channelNoC = 2;    //speed
byte channelNoD = 4;    //forward/ backward switch
byte channelNoE = 1;    //reset counters if == 250
// Note IBusBM library labels channels starting with "0"

//Steering values
const int Ladc = 710;        //Left Limit at home
const int Radc = 287;        //Right Limit at home
const int CenterAdc = 525;   //Center at home
const double steerA = CenterAdc;
const double steerB = (Radc-CenterAdc)/1.7321;     // -124.7
#endif

namespace Vision
{
  char buff[BF_Len];              //was 15
  int buffIdx;
  bool isConfident = false;
  int steeringPWM = 128;          //straight
  int speedPWM = 0;               //wheel PWM from Vision
  int FBDirection = 1;            //forward-back direction from Vision (0 = forward, 1 = backward)
}

// include iBusBM Library
#include <IBusBM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 20 chars and 4 line display

// Create iBus Object
IBusBM ibus;

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) 
{
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}


// Read the channel and return a boolean value
int readSwitch(byte channelInput, bool defaultValue) 
{
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return ch;
  //  return (ch > 50);
}

/*  Steering Wheel Motor
    D4 for forward backware
    D5 for PWM
    01 Jan 2022
*/

const double whlCir = 42.5;     // Wheel Circumference(inch)
const double FBGain = 0. ;      // drive straight wheel feedback gain NOTUSED
const double ST_PWM_Gain = 10.; // NOT USED
double sumPot;
long cntPot;
//
const double inchPerCnt = 1.417178;
const double distL4 = 4. * inchPerCnt;   //Velocity calc distance
const double distR4 = distL4;
const double whlBase = 24.375;
const double cntsPerDegree = whlBase * PI / (1.417176 * 90.); //turn constant

long L_wheel;
long R_wheel;


volatile long cntrL, cntrR, cntrRD;        //Buffer of interrupt times
long mode3SrtTime;
const byte BL = 30;
volatile byte ptrL, ptrR, ptrL0, ptrR0;
long LIntTm[BL], RIntTm[BL];
long lpTm, lpTm0, lastPrint, delTL, delTR;
int driveFlag;        //used in mode 3 for end of run
int turnAnglePWM;
int oldValueB;


void setup()
{
  oldValueB = 0;;
  // Start serial monitor
  Serial.begin(115200);
  Serial3.begin(38400);           // Raspberry Pi
  Serial.println("Begin");
  lcd.init();                      // initialize the lcd
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0, 3);
  lcd.print("Read Pi Steering");

  attachInterrupt(LIntPin, leftWhlCnt, CHANGE);
  attachInterrupt(RIntPin, rightWhlCnt, CHANGE);

  pinMode(ST_RLPin, OUTPUT);
  pinMode(ST_PWM_Pin, OUTPUT);
  pinMode(FB_R_Pin, OUTPUT);
  pinMode(FB_R_PWM, OUTPUT);
  pinMode(FB_L_Pin, OUTPUT);
  pinMode(FB_R_PWM, OUTPUT);

  pinMode(RelayPin8, OUTPUT);    //Power to PWM amps

  digitalWrite(ST_RLPin, 0);
  analogWrite(ST_PWM_Pin, 0);
  digitalWrite(FB_R_Pin, 0);
  analogWrite(FB_R_PWM, 0);
  digitalWrite(FB_L_Pin, 0);
  analogWrite(FB_R_PWM, 0);

  ibus.begin(Serial1);
  cntrRD = 0;                 //Print status every specified period (10 counts)
  cntrR = 0;
  cntrL = 0;
  turnAnglePWM = 128;
  Vision::buffIdx = 0;

  // send configuration data in UBX protocol
  for(int i = 0; i < sizeof(UBLOX_INIT); i++) {                        
    Serial2.write( pgm_read_byte(UBLOX_INIT+i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
}
// **********************************************************************
void loop()
{
  double pot, desA, delCntr ;
  int ST_PWM, FB_PWM;


  int valueA = readChannel(channelNoA, -250, 250, 0);    //turn
  int valueB = readChannel(channelNoB, -250, 250, -250);    //SWC 3 possition switch
  int valueC = readChannel(channelNoC, 0, 250, 0);       //speed
  int valueD = readChannel(channelNoD, 0, 250, 0);       //forward=0,backward = 250
  int valueE = readChannel(channelNoE, -250, 250, 0);    //reset counters if == 250

  // save start time of mode 3
  if(valueB == 250)
  {
    if(oldValueB < 250)
    {
      mode3SrtTime = micros();    //mode3 start time
      driveFlag = 1;           //Enable mode 3 driving
      lastPrint = mode3SrtTime;
      cntrR = 0;
      cntrL = 0;
      sumPot = 0.;
      cntPot = 0;
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Mode3 Drive Staight");
    }
  }
  oldValueB = valueB;
  if (valueE > 150)     //reset counters if switch 
  {
    cntrR = 0;
    cntrL = 0;
    sumPot = 0.;
    cntPot = 0;
  }
  if (valueB < -150)      //Mode 1 *****************
  {
    digitalWrite(RelayPin8, HIGH);      //Use OEM controller
      if (micros() > lastPrint)
      {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Mode 1");
        lcd.setCursor(0, 1);
        lcd.print("Poten");
        lcd.setCursor(6, 1);
        lcd.print(1023-analogRead(Poten));
        lcd.setCursor(0,2);
        lcd.print("avePot ");
        lcd.setCursor(7,2);
        lcd.print(sumPot/cntPot);
        lcd.setCursor(14,2);
        lcd.print(cntPot);
        lcd.setCursor(0,3);
        lcd.print("sA=");
        lcd.print(steerA,1);
        lcd.print(" sB=");
        lcd.print(steerB,3);
        
        
        lastPrint = micros() + 500000;
      }

  }
  else
  {
    digitalWrite(RelayPin8, LOW);       //Use Use FlySky controller
  }

  if (valueB == 0)        //****************** MODE 2, remote control ********
  {
    pot = 1023 - analogRead(Poten);    //using double gears in ackerman steering
    if(revTurn == 1)
    {
       desA = steerA - steerB * tan(PI * valueA / 750.);   //max is pi/3
    }
    else
    {
      desA = steerA + steerB * tan(PI * valueA / 750.);
    }
    ST_PWM = floor(steerKp * (desA - pot) + .5);
    if (ST_PWM >= 0)                   // turn right
    {
      if (ST_PWM > 255)
      {
        ST_PWM = 255;
      }
      digitalWrite(ST_RLPin, HIGH);    // drive steering forward
      analogWrite(ST_PWM_Pin, ST_PWM);
    }
    else
    {
      if (ST_PWM < -255)
      {
        ST_PWM = -255;
      }
      digitalWrite(ST_RLPin, LOW);   // turn left (drive steering backwards
      analogWrite(ST_PWM_Pin, -ST_PWM);
    }
    if (valueD == 0)
    {
      digitalWrite(FB_R_Pin, LOW);      //Forward
      digitalWrite(FB_L_Pin, LOW);
    }
    else
    {
      digitalWrite(FB_R_Pin, HIGH);     //Backward
      digitalWrite(FB_L_Pin, HIGH);
    }
    analogWrite(FB_R_PWM, valueC);      //Speed
    analogWrite(FB_L_PWM, valueC);

    sumPot += 1023-analogRead(Poten);
    cntPot += 1;

    if (micros() > lastPrint)
    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Mode 2");
      lcd.setCursor(0, 1);
      lcd.print("Poten");
      lcd.setCursor(6, 1);
      lcd.print(1023-analogRead(Poten));
      lcd.setCursor(0,2);
      lcd.print("avePot ");
      lcd.setCursor(7,2);
      lcd.print(sumPot/cntPot);
      lcd.setCursor(14,2);
      lcd.print(cntPot);
      lcd.setCursor(0,3);         //print left counter
      lcd.print(cntrL);  
      lcd.setCursor(10,3);        //print right counter
      lcd.print(cntrR);
      
      Serial.print(valueA);       //print transmitter turn angle
      Serial.print(" ");
      Serial.print(1023-analogRead(Poten));
      Serial.print("  ");
      Serial.print(cntrL);
      Serial.print("  ");
      Serial.print(cntrR);
      Serial.print(" ");
      Serial.print(micros());     //time
      Serial.print(" ");
      Serial.print(valueC);     //speed
      Serial.println(" ");
      lastPrint = micros() + 500000;
    }
  }
  else if (valueB > 150)   // MODE 3 Computer control %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  {
    pot = 1023 - analogRead(Poten);    //using double gears in ackerman steering
    
    String inputString = Serial.readStringUntil('\n'); // Read the serial data until newline character
    inputString.trim(); // Remove any leading/trailing whitespace characters

    // Define the string to compare against
    String expectedString = "Stop";
    String followString = inputString.substring(0, 2);
    String expected = "x1";
    // Compare the received string with the expected string
    if (inputString.equals(expectedString)) {
      analogWrite(FB_R_PWM, 0);      //Speed
      analogWrite(FB_L_PWM, 0);
      lcd.print("2");
    } else if (inputString.equals("null") ) {
      nullCounter = nullCounter + 1;
      if (nullCounter == 7) {
        analogWrite(FB_R_PWM, 0);      //Speed
        analogWrite(FB_L_PWM, 0);
        nullCounter = 0;
      }
    } else if(inputString.length() == 0) {
      analogWrite(FB_R_PWM, 0);      //Speed
      analogWrite(FB_L_PWM, 0);
      if(revTurn == 1)
      {
        desA = steerA - steerB * tan(PI * 0 / 750.);   //max is pi/3
      }
      else
      {
        desA = steerA + steerB * tan(PI * 0 / 750.);
      }
    } else if (followString.equals(expected)) {
      nullCounter = 0;
      String values = inputString;
      // Find the index of the values
      int x1Index = values.indexOf("x1:") + 3; // Add 3 to skip "x1:"
      int y1Index = values.indexOf("y1:") + 3; // Add 3 to skip "y1:"
      int x2Index = values.indexOf("x2:") + 3; // Add 3 to skip "x2:"
      int y2Index = values.indexOf("y2:") + 3; // Add 3 to skip "y2:"
      
      // Extract and convert substrings to integers
      int x1 = values.substring(x1Index, values.indexOf(",", x1Index)).toInt();
      int y1 = values.substring(y1Index, values.indexOf(",", y1Index)).toInt();
      int x2 = values.substring(x2Index, values.indexOf(",", x2Index)).toInt();
      int y2 = values.substring(y2Index, values.indexOf(",", y2Index)).toInt();
      int x_d = (x1 + x2) / 2; // center of the person box from the camera
     
      if (x_d > 462) {
        x_d = 462;
      } else if (x_d < 172) {
        x_d = 172;
      }
      int turnX = map(x_d, 172, 462, -250, 250); // maps points on the camera to the turn values of the car
      if (turnX > 250) {
        turnX = 250;
      } else if (turnX < -250) {
        turnX = -250;
      }
      lcd.setCursor(0,1);
      lcd.print(x2);
      if(revTurn == 1)
      {
        desA = steerA - steerB * tan(PI * turnX / 750.);   //max is pi/3
      }
      else
      {
        desA = steerA + steerB * tan(PI * turnX / 750.); // sets the turning angle of the car
      }
      analogWrite(FB_R_PWM, 50);      //Speed
      analogWrite(FB_L_PWM, 50);
    } else {
      analogWrite(FB_R_PWM, 0);      //Speed
      analogWrite(FB_L_PWM, 0);
      if(revTurn == 1)
      {
        desA = steerA - steerB * tan(PI * 0 / 750.);   //max is pi/3
      }
      else
      {
        desA = steerA + steerB * tan(PI * 0 / 750.);
      }
    }

    // Putting gps data into variables
    lat = pvt.lat;
    lng = pvt.lon;
    heading = pvt.headMot/100000.0f;
    hour = pvt.hour;
    minute = pvt.minute;
    second = pvt.second;
    L_wheel =  getLong(&cntrL);
    R_wheel = getLong(&cntrR);

      /*if (x_d < X_mid) {
        if(revTurn == 1)
        {
          desA = steerA - steerB * tan(PI * 200 / 750.);   //max is pi/3
        }
        else
        {
          desA = steerA + steerB * tan(PI * 200 / 750.);
        }
      } else if (x_d > X_mid) {
        if(revTurn == 1)
        {
          desA = steerA - steerB * tan(PI * -200 / 750.);   //max is pi/3
        }
        else
        {
          desA = steerA + steerB * tan(PI * -200 / 750.);
        }
      } else {
          if(revTurn == 1)
          {
            desA = steerA - steerB * tan(PI * 0 / 750.);   //max is pi/3
          }
          else
          {
            desA = steerA + steerB * tan(PI * 0 / 750.);
          }
      } */
    

    ST_PWM = floor(steerKp * (desA - pot) + .5);
    if (ST_PWM >= 0)                   // turn right
    {
      if (ST_PWM > 255)
      {
        ST_PWM = 255;
      }
      digitalWrite(ST_RLPin, HIGH);    // drive steering forward
      analogWrite(ST_PWM_Pin, ST_PWM);
    }
    else
    {
      if (ST_PWM < -255)
      {
        ST_PWM = -255;
      }
      digitalWrite(ST_RLPin, LOW);   // turn left (drive steering backwards
      analogWrite(ST_PWM_Pin, -ST_PWM);
    }
    if (valueD == 0)
    {
      digitalWrite(FB_R_Pin, LOW);      //Forward
      digitalWrite(FB_L_Pin, LOW);
    }
    else
    {
      digitalWrite(FB_R_Pin, HIGH);     //Backward
      digitalWrite(FB_L_Pin, HIGH);
    }


    sumPot += 1023-analogRead(Poten);
    cntPot += 1;


  } 
               //END of MODE 3 using driveFlag to stop after driveDist  %%%%%%%%%%%%%

  
}
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
void leftWhlCnt()
{
  long intTime = micros();

  if (intTime > LIntTm[ptrL] + 1000L)
  {
    ptrL++;
    if (ptrL >= BL) ptrL = 0;
    LIntTm[ptrL] = intTime;
    cntrL++;
  }
}

void rightWhlCnt()
{
  long intTime = micros();

  if (intTime > RIntTm[ptrR] + 1000L)
  {
    ptrR++;
    if (ptrR >= BL) ptrR = 0;
    RIntTm[ptrR] = intTime;
    cntrR++;
  }
}

int readVisionData()
{
  char inByte;
  int rtnChr;           //return character that serial returned a value
  int  byteCnt;
  char in[BF_Len];
  char *angleBuff;
  char *fw;
  char *speedPWM;

  rtnChr = 0;
  byteCnt = 0;
  while (Serial3.available())
  {
    rtnChr = 1;
    inByte = Serial3.read();
    if (inByte != '/n')
    {
      if (byteCnt++ < BF_Len)
      {
        in[byteCnt - 1] = inByte;
      }
      else
      {
        byteCnt = BF_Len + 1;     //set back to buffer overflow
        in[BF_Len - 1] = 0;
      }
    }
    //lcd.setCursor(0,3);
    Serial.println(in);

    // Full message received
    strcpy(Vision::buff, in);

    // Parse message
    if (Vision::buff[0] == '1')
    { // Trust the angle
      Vision::isConfident = true;
      angleBuff = strtok(Vision::buff, ", ");
      angleBuff = strtok(NULL, ", ");
      Vision::steeringPWM = atof(angleBuff);

      fw = strtok(NULL, ", ");
      Vision::FBDirection = atof(fw) * 250;

      speedPWM = strtok(NULL, ", ");
      Vision::speedPWM = atof(speedPWM);
    }
    else
    {
      Vision::isConfident = false;
      //        PID::V.reset();
    }

    if (Vision::isConfident)
    {
      Serial.print("VIS: Confident! Correction: ");
      Serial.println(Vision::steeringPWM);
    }
    else
    {
      Serial.println("VIS: Not confident.");
    }

    Vision::buffIdx = 0;

  }
  return rtnChr;
}

long getLong(long *var)
{
  noInterrupts();
  long gvar = *var;
  interrupts();
  return gvar;
}


