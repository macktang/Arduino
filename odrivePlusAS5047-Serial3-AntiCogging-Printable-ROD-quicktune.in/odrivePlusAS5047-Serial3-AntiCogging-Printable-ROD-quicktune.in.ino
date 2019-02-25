//red - 5V = WHITE
//orange - C sn = BLUE
//green - MOSI = GREEN
//black - GND = BLACK
//
//yellow - mISO = PURPLE
//
//brown - clock = GRAY

// ***************Odrive includes
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

// ***************AS5047 includes
#include <SPI.h>

// *************Odrive initialization
// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// Serial to the ODrive
//SoftwareSerial odrive_serial(51, 53); //RX (ODrive TX), TX (ODrive RX)
// Note: you must also connect GND on ODrive to GND on Arduino!

// ODrive object
ODriveArduino odrive(Serial1);

// *************AS5047 initialization
unsigned int reading = 0;

// AS4047D Registers
#define AS5047D_select_pin 10

/** volatile **/
#define NOP 0x0000
#define ERRFL 0x0001
#define PROG   0x0003
#define DIAAGC 0x3FFC
#define CORDICMAG 0x3FFD
#define ANGLEUNC  0x3FFE
#define ANGLECOM  0x3FFF

/** non-volatile **/
//#define ZPOSM 0x0016
//#define ZPOSL 0x0017
//#define SETTINGS1 0x0018
//#define SETTINGS2 0x0019

#define RD  0x40    // bit 14 "1" is Read + parity even
#define WR  0x3F    //bit 14 ="0" is Write

//Op Arduino: D10 CS, D11 MOSI, D12 MISO, D13 SCK

// ******* PID CONSTANTS **************************************** PID CONSTANTS ******************
float Kp = .002853;
float Kd = .015;
//float Ki;
float qq = 0.1; //value to increment Kp and Kd by

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(115200);
  Odrivesetup();
  AS5047setup();
}

void loop() {
  // put your main code here, to run repeatedly:
  Odriveloop();
  
}

void Odrivesetup() {
  // ODrive uses 115200 baud
//  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

//  Serial.println("ODriveArduino");
//  Serial.println("Setting parameters...");

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis) {
    Serial1 << "w axis" << axis << ".controller.config.vel_limit " << 100000.0f << '\n';
    Serial1 << "w axis" << axis << ".motor.config.current_lim " << 31.0f << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }

  Serial.println("Ready!");
//  Serial.println("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)");
//  Serial.println("Send the character 's' to exectue test move");
//  Serial.println("Send the character 'b' to read bus voltage");
//  Serial.println("Send the character 'p' to read motor positions in a 10s loop");
//  Serial.println("Send the character 'h' to set current control mode");
//  Serial.println("Send the character 'm' to do custom move on axis0");
//  Serial.println("Send the character 'x' for idle state on axis0");
//  Serial.println("Send the character 't' for turn read turn on axis0");

}



void AS5047setup() {
  pinMode(AS5047D_select_pin, OUTPUT);

  SPI.begin();
  SPI.setDataMode(SPI_MODE1); // properties chip
  SPI.setBitOrder(MSBFIRST);  //properties chip

//  Serial.begin(115200);  // start serial for output REDUNDANT)
//  Serial.println(" AS5047D:");

//  AS5047D_Write( AS5047D_select_pin , SETTINGS1, 0x0004);
//  AS5047D_Write( AS5047D_select_pin , SETTINGS2, 0x0000);
//  AS5047D_Write( AS5047D_select_pin , ZPOSM, 0x0000); // is it really possible to initially set angle at 0 degrees??
//  AS5047D_Write( AS5047D_select_pin , ZPOSL, 0x0000);
}


void Odriveloop() {

  if (Serial.available()) {
    char c = Serial.read();

    

    // Run calibration sequence
    if (c == '0' || c == '1') {
      int motornum = c-'0';
      int requested_state;

      requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, true); // don't wait

    }

    // Sinusoidal test move
    if (c == 's') {
      Serial.println("Executing test move");
      for (float ph = 0.0f; ph < 6.28318530718f; ph += 0.01f) {
        float pos_m0 = 20000.0f * cos(ph);
        float pos_m1 = 20000.0f * sin(ph);
        odrive.SetPosition(0, pos_m0);
        odrive.SetPosition(1, pos_m1);
        delay(5);
      }
    }

    // Read bus voltage
    if (c == 'b') {
      Serial1 << "r vbus_voltage\n";
      Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
    }

    if (c == 'h') {
        // Sets up current control
        int requested_state;
        Serial1 << "w axis" << 0 << ".controller.config.control_mode " << 1 << '\n';

        requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
        Serial << "Axis" << 0 << ": Requesting state " << 1 << '\n';
        odrive.run_state(0, requested_state, true); // don't wait

        Serial.println("done");
    }

    if(c =='r'){readTuning();}
    
    if (c == 'k'){
      Serial.flush();
      delay(10);
      tuningRoutine();
    }

    if (c == 'w'){
      Serial.flush();
      delay(10);
      writeRoutine();
    }

    // custom move
    if (c == 'm') {
//      odrive_serial << "r vbus_voltage\n";
//      Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
//        Serial1 << "w axis" << axis << ".motor.config.current_lim " << 11.0f << '\n';

        int motornum = 0;
        int requested_state;
        int my_axis;
        int control_mode;
        
        my_axis = 0;
        control_mode = 1; // CURRENT CONTROL

//        requested_state = ODriveArduino::AXIS_STATE_IDLE;
//        Serial << "Axis" << my_axis << ": Requesting state " << requested_state << '\n';
//        odrive.run_state(motornum, requested_state, true); // don't wait

//        Serial1 << "w axis" << my_axis << ".controller.config.control_mode " << control_mode << '\n';
//
//        requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
//        Serial << "Axis" << my_axis << ": Requesting state " << requested_state << '\n';
//        odrive.run_state(motornum, requested_state, true); // don't wait

//        Serial.println(FastAS5047D_Read());
//        Serial.print("STARTING!!!!!!======================================================");
//        delay(1000);
//        Serial.print("2");
//        delay(1000);
//        Serial.println("1");
//        delay(1000);
        int myPos; //can read() just return as an int?
        int prevPos = FastAS5047D_Read();
        int prevPos2 = FastAS5047D_Read();
//        unsigned int loopCount = 0;
        float mySpeedDampen;
        float myError;
        float myCurrent;
        unsigned long loop_start_time = millis();
        unsigned long loop_current_time = millis();
        
        while( (loop_current_time - loop_start_time) < 12000 ){
          // if within +-10 degrees of balanced
          myPos = FastAS5047D_Read();
          if ( myPos >= 4061 && myPos <= 6791 ){
//            myError = (myPos-6560)*0.0003834951969714103074295218974*0.19;
//            myError = -(myPos-5361)*.00317;
            myError = -(myPos-5361)* Kp;
            mySpeedDampen = ( abs(prevPos2 - prevPos) + abs(myPos - prevPos) )/2 *Kd;
//            mySpeedDampen = abs(myPos - prevPos);
//            Serial.println( (myPos-6443)*0.0003834951969714103074295218974*.18 ,7);
//            Serial.println(myError,7);
//            myCurrent = -myError * 360. / 8.27 - mySpeedDampen*sgn(myPos-5361);
            myCurrent = myError + mySpeedDampen*sgn(myPos-5361);
//            myCurrent = -myError * 360. / 8.27;
//            myCurrent = -myError * 360. / 8.27 - 0.15;
            if (myCurrent > 18.0){
              myCurrent = 18.0;
            }
            else if (myCurrent < -18.0){
              myCurrent = -18.0;
            }
//            if (myCurrent < 0){
//              myCurrent += -0.15;
//            }
//            Serial.println(myCurrent);
            Serial1 << "w axis" << my_axis << ".controller.current_setpoint " << myCurrent << '\n';
//            Serial.println(abs(myPos - prevPos),6);
//            loopCount += 1;
            Serial.print(myPos);
            Serial.print(",");
            Serial.print(myCurrent,4);
            Serial.print(",");
            Serial.print(myError,8);
            Serial.print(",");
            Serial.println(mySpeedDampen,4);
          }
          else{
            Serial1 << "w axis" << my_axis << ".controller.current_setpoint " << 0.0f << '\n';
            break;
//            Serial.println(0.0);
          }
          prevPos2 = prevPos;
          prevPos = myPos;
//          delay(10);
          loop_current_time = millis();
        }

//        Serial.println(loopCount);

//        for (int i=0; i<= 20; i++){
// //        odrv0.axis0.controller.current_setpoint = 0.4
//          Serial1 << "w axis" << my_axis << ".controller.current_setpoint " << 0.5f << '\n';
//          delay(10);
//
//          Serial1 << "w axis" << my_axis << ".controller.current_setpoint " << -0.5f << '\n';
//          delay(10);         
//        }

        Serial1 << "w axis" << my_axis << ".controller.current_setpoint " << 0.0f << '\n';
        delay(10);

//        Serial.println(FastAS5047D_Read());
        
    }

    if (c == 'f') {

        int motornum = 0;
        int requested_state;
        int my_axis;
        int control_mode;
        
        my_axis = 0;
        control_mode = 1; // CURRENT CONTROL

//        requested_state = ODriveArduino::AXIS_STATE_IDLE;
//        Serial << "Axis" << my_axis << ": Requesting state " << requested_state << '\n';
//        odrive.run_state(motornum, requested_state, true); // don't wait

        Serial1 << "w axis" << my_axis << ".controller.config.control_mode " << control_mode << '\n';

        requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
        Serial << "Axis" << my_axis << ": Requesting state " << requested_state << '\n';
        odrive.run_state(motornum, requested_state, true); // don't wait

//        Serial.println(FastAS5047D_Read());
        Serial.print("friction test starting!!!!!!======================================================");
        
//        float myCurrent;
        char a;
        float myCurrent;

        while (!Serial.available()){}

        a = Serial.read();
        Serial.println(a);

        
        for (int i=0; i<= 0; i++){

          Serial1 << "w axis" << my_axis << ".controller.current_setpoint " << 0.5f-0.30f << '\n';
          delay(500);

          Serial1 << "w axis" << my_axis << ".controller.current_setpoint " << 0.0f << '\n';
          delay(5000);

          Serial1 << "w axis" << my_axis << ".controller.current_setpoint " << -0.5f-0.30f << '\n';
          delay(500);

          Serial1 << "w axis" << my_axis << ".controller.current_setpoint " << 0.0f << '\n';
          delay(5000);  
        }

        Serial1 << "w axis" << my_axis << ".controller.current_setpoint " << 0.0f << '\n';
        delay(10);
        
    }

    if (c == 'x') {
//      Serial1 << "r vbus_voltage\n";
//      Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
//        Serial1 << "w axis" << axis << ".motor.config.current_lim " << 11.0f << '\n';

        int motornum = 0;
        int requested_state;

        requested_state = ODriveArduino::AXIS_STATE_IDLE;
        Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
        odrive.run_state(motornum, requested_state, false); // don't wait

        
    }

    if (c == 't') {
      
        float pos_one = 0.0f;
        float pos_two = 2000.0f;
        
        odrive.SetPosition(0, pos_one);
        delay(5);

        AS5047loop();
        
        odrive.SetPosition(0, pos_two);
        delay(5);

        AS5047loop();
    }

    // print motor positions in a 10s loop
    if (c == 'p') {
      static const unsigned long duration = 10000;
      unsigned long start = millis();
      while(millis() - start < duration) {
        for (int motor = 0; motor < 2; ++motor) {
          Serial1 << "r axis" << motor << ".encoder.pos_estimate\n";
          Serial << odrive.readFloat() << '\t';
        }
        Serial << '\n';
      }
    }
  }
}





void AS5047loop()
{
  DumpRegisterValues();
  Serial.println();
  delay(200);
}



//******************************* PID quick tuning functions **********
// Mack Tang 2-24-2019
// This program demonstrates capability to
// quickly tune Kp, Ki, and Kd constants
// in PID control, without need
// to recompile code in between.

// Instructions to run
// 1) Open in Arduino IDE
// 2) Upload code, open Serial Monitor

// Example usages
// PURPOSE                COMMAND OVER SERIAL
// set Kp = 0.0156        wkp.015
// set Kd = -6.102        wkd-6.102
// increment Kp += qq     kp+
// decrement Kp -= qq     kd-
// set qq = 0.005         wqq.005
// print Kp, Kd, qq       r
// "run motors"           m

void tuningRoutine(){  
  if(Serial.available()){
    Serial.print("tuning mode active...");                
    char param = Serial.read();
    char cmd = Serial.read();
    switch(param){
      case 'p':
        if(cmd=='+'){Kp+=qq; Serial.print("Kp increased to "); Serial.println(Kp,8);}
        if(cmd=='-'){Kp-=qq; Serial.print("Kp decreased to "); Serial.println(Kp,8);}
        break;
      case 'd':
        if(cmd=='+'){Kd+=qq; Serial.print("Kd increased to "); Serial.println(Kd,8);}
        if(cmd=='-'){Kd-=qq; Serial.print("Kd decreased to "); Serial.println(Kd,8);}
        break;
    }
  }
}

void writeRoutine(){  
  if(Serial.available()){
    Serial.print("pid parameter writing mode active...");
    char param1 = Serial.read();
    char param2 = Serial.read();
    String s1 = Serial.readString();

    if(param1=='k'&&param2=='p'){Kp = s1.toFloat(); Serial.println(Kp,8);}
    if(param1=='k'&&param2=='d'){Kd = s1.toFloat(); Serial.println(Kd,8);}
    if(param1=='q'&&param2=='q'){qq = s1.toFloat(); Serial.println(qq,8);}
  }
}

void readTuning(){
  Serial.print("Kp:");    Serial.print(Kp,8);
  Serial.print("   Kd:"); Serial.print(Kd,8);
  Serial.print("   qq:"); Serial.println(qq,8);
}



//******************************* AS5047 functions ***********************************************************************************************

void DumpRegisterValues()
{
  Serial.print("NOP: "); Serial.println(AS5047D_Read( AS5047D_select_pin, NOP) & 0x3FFF, BIN); // strip bit 14..15
  Serial.print("ERRFL: "); Serial.println(AS5047D_Read( AS5047D_select_pin, ERRFL) & 0x3FFF, BIN); // strip bit 14..15
  Serial.print("PROG: "); Serial.println(AS5047D_Read( AS5047D_select_pin, PROG) & 0x3FFF, BIN); // strip bit 14..15
  Serial.print("DIAAGC: "); Serial.println(AS5047D_Read( AS5047D_select_pin, DIAAGC) & 0x3FFF, BIN); // strip bit 14..15
  
  Serial.print("CORDICMAG: "); Serial.println(AS5047D_Read( AS5047D_select_pin, CORDICMAG) & 0x3FFF, DEC); // strip bit 14..15
  Serial.print("ANGLEUNC: "); Serial.println(AS5047D_Read( AS5047D_select_pin, ANGLEUNC) & 0x3FFF, DEC); // strip bit 14..15
  Serial.print("ANGLECOM: "); Serial.println(AS5047D_Read( AS5047D_select_pin, ANGLECOM) & 0x3FFF, DEC); // strip bit 14..15

//  Serial.print("ZPOSM: "); Serial.println(AS5047D_Read( AS5047D_select_pin, ZPOSM) & 0x3FFF, BIN); // strip bit 14..15
//  Serial.print("ZPOSL: "); Serial.println(AS5047D_Read( AS5047D_select_pin, ZPOSL) & 0x3FFF, BIN); // strip bit 14..15
//  Serial.print("SETTINGS1: "); Serial.println(AS5047D_Read( AS5047D_select_pin, SETTINGS1) & 0x3FFF, BIN); // strip bit 14..15
//  Serial.print("SETTINGS2: "); Serial.println(AS5047D_Read( AS5047D_select_pin, SETTINGS2) & 0x3FFF, BIN); // strip bit 14..15
}

// ************************Write to AS5047D **************************
void AS5047D_Write( int SSPin, int address, int value)
{
  // take the SS pin low to select the chip:
  digitalWrite(SSPin, LOW);
  
  Serial.println(value, HEX);
  
  //  send in the address via SPI:
  
  byte v_l = address & 0x00FF;
  byte v_h = (unsigned int)(address & 0x3F00) >> 8;
  
  if (parity(address & 0x3F) == 1) v_h = v_h | 0x80; // set parity bit
  //v_h = v_h & (WR | 0x80);  // its  a write command and don't change the parity bit (0x80)
  
  Serial.print( " parity:  "); Serial.println(parity(address & 0x3F));
  Serial.print(v_h, HEX); Serial.print(" A ");  Serial.println(v_l, HEX);
  
  SPI.transfer(v_h);
  SPI.transfer(v_l);
  
  digitalWrite(SSPin, HIGH);
  
  delay(2);
  
  digitalWrite(SSPin, LOW);
  
  //  send value via SPI:
  
  v_l = value & 0x00FF;
  v_h = (unsigned int)(value & 0x3F00) >> 8;
  
  if (parity(value & 0x3F) == 1) v_h = v_h | 0x80; // set parity bit
  //v_h = v_h & (WR | 0x80); // its a write command and don't change the parity bit (0x80)
  
  Serial.print(v_h, HEX); Serial.print(" D ");  Serial.println(v_l, HEX);
  
  SPI.transfer(v_h);
  SPI.transfer(v_l);
  
  // take the SS pin high to de-select the chip:
  digitalWrite(SSPin, HIGH);
}

//*******************Read from AS5047D ********************************
unsigned int AS5047D_Read( int SSPin, unsigned int address)
{
  unsigned int result = 0;   // result to return
  
  byte res_h = 0;
  byte res_l = 0;
  
  // take the SS pin low to select the chip:
  digitalWrite(SSPin, LOW);
  
  //  send in the address and value via SPI:
  byte v_l = address & 0x00FF;
  byte v_h = (unsigned int)(address & 0x3F00) >> 8;
  
  if (parity(address | (RD << 8)) == 1) v_h = v_h | 0x80; // set parity bit
  
  v_h = v_h | RD; // its  a read command
  
  // Serial.print( " parity:  ");Serial.println(parity(address | (RD <<8)));
  // Serial.print(v_h, HEX); Serial.print(" A ");  Serial.print(v_l, HEX);  Serial.print(" >> ");
  
  res_h = SPI.transfer(v_h);
  res_l = SPI.transfer(v_l);
  
  digitalWrite(SSPin, HIGH);
  
  delay(2);
  
  digitalWrite(SSPin, LOW);
  
  //if (parity(0x00 | (RD <<8))==1) res_h = res_h | 0x80;  // set parity bit
  //res_h = res_h | RD;
  
  res_h = (SPI.transfer(0x00));
  res_l = SPI.transfer(0x00);
  
  res_h = res_h & 0x3F;  // filter bits outside data
  
  //Serial.print(res_h, HEX);   Serial.print(" R  ");  Serial.print(res_l, HEX);   Serial.print("  ");
  
  digitalWrite(SSPin, HIGH);
  
  return (result = (res_h << 8) | res_l);
}


//*******************Read from AS5047D ********************************
unsigned int FastAS5047D_Read()
{

//  unsigned int res_16;
  byte res_h = 0;
  byte res_l = 0;
  
  // take the SS pin low to select the chip:
  digitalWrite(53, LOW);

//  res_16 = SPI.transfer(0xFFFF);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  
  digitalWrite(53, HIGH);
    
  digitalWrite(53, LOW);

//  res_16 = SPI.transfer(0x0000);
  res_h = (SPI.transfer(0x00));
  res_l = SPI.transfer(0x00);
  
//  res_16 = res_16 & 0x3FFF;  // filter bits outside data
  res_h = res_h & 0x3F;  // filter bits outside data
    
  digitalWrite(53, HIGH);
  
//  return (res_16);
  return ((res_h << 8) | res_l);
}

//*******************check parity ******************************************
int parity(unsigned int x) {
  int parity = 0;
  while (x > 0) {
    parity = (parity + (x & 1)) % 2;
    x >>= 1;
  }
  return (parity);
}



static inline int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}
