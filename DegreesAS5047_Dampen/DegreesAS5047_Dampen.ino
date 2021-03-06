// pendulum down is 13536
// pendulum down is 13547
// 13541.5

// pendulum down is 13564
// pendulum down is 13581
// 13572.5

//avg og both 13577

//up target 5365

#include <SPI.h>
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

void setup() {
  pinMode(AS5047D_select_pin, OUTPUT);

  SPI.begin();
  SPI.setDataMode(SPI_MODE1); // properties chip
  SPI.setBitOrder(MSBFIRST);  //properties chip

  Serial.begin(115200);  // start serial for output
  Serial.println(" AS5047D:");

}

void loop()
{

  int myPos;
  int prevPos = AS5047D_Read();
  float mySpeedDampen;

  while(1){
    myPos = AS5047D_Read();
//    Serial.println(myPos);
//    Serial.println((myPos-6443)*.0219726563);
//    Serial.println( ((myPos-6516)*.000383495197)*.180000000, 6 );
    Serial.println( sgn(myPos-6516) );
    mySpeedDampen = abs(myPos - prevPos)*0.0003834951969714103074295218974 * .004;
//    Serial.println(mySpeedDampen,6);
    prevPos = myPos;
    delay(10);
  }
  
}

void DumpRegisterValues()
{

  Serial.print("ANGLECOM: "); Serial.println(AS5047D_Read() & 0x3FFF, DEC); // strip bit 14..15
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
unsigned int AS5047D_Read()
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
