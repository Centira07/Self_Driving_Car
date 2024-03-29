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
//  Add: dead reconning
//  Add: display speed, angle turn, position, battery voltage
//  Add: Drive straight potentiometer printout
//  Change: Steering Gain
//  Add: steering null zone
//  Add: GPS dead recogning steering
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

const int BF_Len = 20;                //Serial3 Buffer Length
const double driveDist = 1000.;    //Mode 3 drive dist (inches)
const double steerKp = 15.;     // turning proportional gain(8 is good value)

/* Added to test constant steering command */
double constantValueA;
#define Miami

#ifdef Miami
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
const int Ladc = 488;       //Left limit
const int Radc = 539;       //Right limit
const int CenterAdc = 514;  //Stra8ght value
const double steerA = CenterAdc;    //Ladc = steerA - steerB*tan(60)
const double steerB = (Radc-CenterAdc)/1.7321;
//const double steerB = 23.094;     //Radc = steerA + steerB*tan(60)

#elif defined Home
const int Poten = A0;
const int LIntPin = 0;
const int RIntPin = 1;
const int ST_RLPin = 4;    // Steering Right/Left
const int ST_PWM_Pin = 5;      // Steering PWM
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
const int Ladc = 307;        //Left Limit at home
const int Radc = 749;        //Right Limit at home
const int CenterAdc = 503;   //Center at home
const double steerA = 503.;
const double steerB = 127.6;
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


volatile long cntrL, cntrR, cntrRD;        //Buffer of interrupt times
const byte BL = 30;
volatile byte ptrL, ptrR, ptrL0, ptrR0;
long LIntTm[BL], RIntTm[BL];
long lpTm, lpTm0, lastPrint, delTL, delTR;
int driveFlag;        //used in mode 3 for end of run
int turnAnglePWM;


void setup()
{
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
    driveFlag = 1;           //Enable mode 3 driving


      if (micros() > lastPrint)
      {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Mode 1");
        lcd.setCursor(0, 1);
        lcd.print("Poten");
        lcd.setCursor(6, 1);
        lcd.print(analogRead(Poten));
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

  if (valueB == 0)                    //Mode 2, remote control ********
  {
    pot = analogRead(Poten);
    desA = steerA + steerB * tan(PI * valueA / 750.);
    ST_PWM = floor(steerKp * (desA - pot) + .5);
    if (ST_PWM >= 0)                   // turn right
    {
      if (ST_PWM > 255)
      {
        ST_PWM = 255;
      }
      digitalWrite(ST_RLPin, LOW);    // drive steering forward
      analogWrite(ST_PWM_Pin, ST_PWM);
    }
    else
    {
      if (ST_PWM < -255)
      {
        ST_PWM = -255;
      }
      digitalWrite(ST_RLPin, HIGH);   // turn left (drive steering backwards
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

    sumPot += analogRead(Poten);
    cntPot += 1;

    if (micros() > lastPrint)
    {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Mode 2");
      lcd.setCursor(0, 1);
      lcd.print("Poten");
      lcd.setCursor(6, 1);
      lcd.print(analogRead(Poten));
      lcd.setCursor(0,2);
      lcd.print("avePot ");
      lcd.setCursor(7,2);
      lcd.print(sumPot/cntPot);
      lcd.setCursor(14,2);
      lcd.print(cntPot);
      lastPrint = micros() + 500000;
    }
  }
  else if (valueB > 150)   // MODE 3 Computer control %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  {

    pot = analogRead(Poten);


    /* This line should be un-commented to run with the controller command */
    // desA = steerA + steerB * tan(PI * valueA / 750.);

    /* These lines should be un-commented to run with a set command value as defined below. */
    constantValueA = 250.; /* This value MUST be within range of [-250, 250] */
    desA = steerA + steerB * tan(PI * constantValueA / 750.);


    ST_PWM = floor(steerKp * (desA - pot) + .5);
    if (ST_PWM >= 0)                   // turn right
    {
      if (ST_PWM > 255)
      {
        ST_PWM = 255;
      } 

      digitalWrite(ST_RLPin, LOW );    // drive steering forward
      analogWrite(ST_PWM_Pin, ST_PWM);
    }
    else
    {
      if (ST_PWM < -255)
      {
        ST_PWM = -255;
      }
      digitalWrite(ST_RLPin, HIGH);   // turn left (drive steering backwards
      analogWrite(ST_PWM_Pin, -ST_PWM);
    }

    if (valueD == 0)
    {
        digitalWrite(FB_R_Pin, LOW); /* Drive forward*/
        digitalWrite(FB_L_Pin, LOW);
    }
    else
    {
        digitalWrite(FB_R_Pin, HIGH);
        digitalWrite(FB_L_Pin, HIGH);
    }

    analogWrite(FB_R_PWM, valueC);
    analogWrite(FB_L_PWM, valueC);

    //END of MODE 3 using driveFlag to stop after driveDist  %%%%%%%%%%%%%

    if (getLong(&cntrR) + getLong(&cntrL) >= cntrRD)
    {
      Serial.print("Turn angle ");
      Serial.println(analogRead(Poten));

      Serial.print("**** L=");
      Serial.print(getLong(&cntrL));
      Serial.print("   R=");
      Serial.println(getLong(&cntrR));
      cntrRD = getLong(&cntrR) + getLong(&cntrL) + 10;
    }

    if (micros() > lastPrint)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(getLong(&cntrL));
      lcd.setCursor(10, 0);
      lcd.print(getLong(&cntrR));
      lastPrint = micros() + 500000;
      lcd.setCursor(0, 1);
      //    lcd.print(LIntTm[ptrL]);
      lcd.print((getLong(&cntrR) + getLong(&cntrL))*inchPerCnt / 2.);
      lcd.setCursor(10, 1);
      //    lcd.print(RIntTm[ptrR]);
      lcd.print(driveFlag);

      if ( (ptrL0 = ptrL - 4) < 0 )
      {
        ptrL0 += BL;
      }
      delTL = LIntTm[ptrL] - LIntTm[ptrL0];
      if ( (ptrR0 = ptrR - 4) < 0 )
      {
        ptrR0 += BL;
      }
      delTR = RIntTm[ptrR] - RIntTm[ptrR0];

      lcd.setCursor(0, 2);
      lcd.print(Vision::isConfident );
      lcd.setCursor(10, 2);
      lcd.print(Vision::steeringPWM );

      //      lcd.setCursor(0, 3);
      //      lcd.print(analogRead(Poten));
      //      lcd.setCursor(10, 3);
      //      lcd.print(valueA);

      lcd.setCursor(0, 3);
      lcd.print(Vision::FBDirection);
      lcd.setCursor(10, 3);
      lcd.print(Vision::speedPWM);
    }
    else
    {
      delay( 10 );
    }
  }
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
void driveCar() {
}

void turnCar() {
}

void deadReckoning() {
}

void carStayStraight() {
}

void displayGPSInfo() {
}

