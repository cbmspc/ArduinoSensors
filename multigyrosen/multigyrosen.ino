#define GYRO_FULL_SCALE_LEVEL 2


#if GYRO_FULL_SCALE_LEVEL == 2
  #define GYRO_CTRL4_VALUE 0b00110000
  #define GYRO_DEG_PER_DIGIT 0.07
#elif GYRO_FULL_SCALE_LEVEL == 1
  #define GYRO_CTRL4_VALUE 0b00010000
  #define GYRO_DEG_PER_DIGIT 0.0175
#else
  #define GYRO_CTRL4_VALUE 0b00000000
  #define GYRO_DEG_PER_DIGIT 0.00875
#endif


#include <SPI.h> //add arduino SPI library;
//list of all registers binary addresses;
byte WHO_AM_I           = 0B00001111;
byte CTRL_REG1          = 0B00100000;
byte CTRL_REG2          = 0B00100001;
byte CTRL_REG3          = 0B00100010;
byte CTRL_REG4          = 0B00100011;
byte CTRL_REG5          = 0B00100100;
byte REFERENCE          = 0B00100101;
byte OUT_TEMP           = 0B00100110;
byte STATUS_REG         = 0B00100111;
byte OUT_X_L            = 0B00101000;
byte OUT_X_H            = 0B00101001;
byte OUT_Y_L            = 0B00101010;
byte OUT_Y_H            = 0B00101011;
byte OUT_Z_L            = 0B00101100;
byte OUT_Z_H            = 0B00101101;
byte FIFO_CTRL_REG      = 0B00101110;
byte FIFO_SRC_REG       = 0B00101111;
byte INT1_CFG           = 0B00110000;
byte INT1_SRC           = 0B00110001;
byte INT1_TSH_XH        = 0B00110010;
byte INT1_TSH_XL        = 0B00110011;
byte INT1_TSH_YH        = 0B00110100;
byte INT1_TSH_YL        = 0B00110101;
byte INT1_TSH_ZH        = 0B00110110;
byte INT1_TSH_ZL        = 0B00110111;
byte INT1_DURATION      = 0B00111000;

byte Read  = 0B10000000;
byte Write = 0B00000000;

//chip select pin on arduino;
const int CS = 10;
/*
SPI.h sets these for us in arduino
const int SDI = 11;
const int SDO = 12;
const int SCL = 13;
*/











#include <Wire.h>



#ifndef L3G_h
#define L3G_h

#include <Arduino.h> // for byte data type

// device types

#define L3G_DEVICE_AUTO 0
#define L3G4200D_DEVICE 1
#define L3GD20_DEVICE   2


// SA0 states

#define L3G_SA0_LOW  0
#define L3G_SA0_HIGH 1
#define L3G_SA0_AUTO 2

// register addresses

#define L3G_WHO_AM_I      0x0F

#define L3G_CTRL_REG1     0x20
#define L3G_CTRL_REG2     0x21
#define L3G_CTRL_REG3     0x22
#define L3G_CTRL_REG4     0x23
#define L3G_CTRL_REG5     0x24
#define L3G_REFERENCE     0x25
#define L3G_OUT_TEMP      0x26
#define L3G_STATUS_REG    0x27

#define L3G_OUT_X_L       0x28
#define L3G_OUT_X_H       0x29
#define L3G_OUT_Y_L       0x2A
#define L3G_OUT_Y_H       0x2B
#define L3G_OUT_Z_L       0x2C
#define L3G_OUT_Z_H       0x2D

#define L3G_FIFO_CTRL_REG 0x2E
#define L3G_FIFO_SRC_REG  0x2F

#define L3G_INT1_CFG      0x30
#define L3G_INT1_SRC      0x31
#define L3G_INT1_THS_XH   0x32
#define L3G_INT1_THS_XL   0x33
#define L3G_INT1_THS_YH   0x34
#define L3G_INT1_THS_YL   0x35
#define L3G_INT1_THS_ZH   0x36
#define L3G_INT1_THS_ZL   0x37
#define L3G_INT1_DURATION 0x38
#define L3G_LOW_ODR       0x39


class L3G
{
  public:
    typedef struct vector
    {
      float x, y, z;
    } vector;

    vector g; // gyro angular velocity readings

    bool init(byte device = L3G_DEVICE_AUTO, byte sa0 = L3G_SA0_AUTO);

    void enableDefault(void);

    void writeReg(byte reg, byte value);
    byte readReg(byte reg);

    void read(void);

    // vector functions
    static void vector_cross(const vector *a, const vector *b, vector *out);
    static float vector_dot(const vector *a,const vector *b);
    static void vector_normalize(vector *a);
    
    //
    byte getAddress();

  private:
      byte _device; // chip type (4200D or D20)
      byte address;

      bool autoDetectAddress(void);
};

#endif





#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define L3G4200D_ADDRESS_SA0_LOW  (0xD0 >> 1)
#define L3G4200D_ADDRESS_SA0_HIGH (0xD2 >> 1)
#define L3GD20_ADDRESS_SA0_LOW    (0xD4 >> 1)
#define L3GD20_ADDRESS_SA0_HIGH   (0xD6 >> 1)

// Public Methods //////////////////////////////////////////////////////////////

bool L3G::init(byte device, byte sa0)
{
  _device = device;
  switch (_device)
  {
    case L3G4200D_DEVICE:
      if (sa0 == L3G_SA0_LOW)
      {
        address = L3G4200D_ADDRESS_SA0_LOW;
        return true;
      }
      else if (sa0 == L3G_SA0_HIGH)
      {
        address = L3G4200D_ADDRESS_SA0_HIGH;
        return true;
      }
      else
        return autoDetectAddress();
      break;

    case L3GD20_DEVICE:
      if (sa0 == L3G_SA0_LOW)
      {
        address = L3GD20_ADDRESS_SA0_LOW;
        return true;
      }
      else if (sa0 == L3G_SA0_HIGH)
      {
        address = L3GD20_ADDRESS_SA0_HIGH;
        return true;
      }
      else
        return autoDetectAddress();
      break;

    default:
      return autoDetectAddress();
  }
}

// Turns on the L3G's gyro and places it in normal mode.
void L3G::enableDefault(void)
{
  // 0x0F = 0b00001111
  // Normal power mode, all axes enabled
  writeReg(L3G_CTRL_REG1, 0x0F);
  writeReg(L3G_CTRL_REG4, GYRO_CTRL4_VALUE);  // Continuous update, user-defined sensitivity
}

// Writes a gyro register
void L3G::writeReg(byte reg, byte value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Reads a gyro register
byte L3G::readReg(byte reg)
{
  byte value;
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();
  return value;
}


byte L3G::getAddress()
{
  return address;
}


// Reads the 3 gyro channels and stores them in vector g
void L3G::read()
{
  Wire.beginTransmission(address);
  // assert the MSB of the address to get the gyro
  // to do slave-transmit subaddress updating.
  Wire.write(L3G_OUT_X_L | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)6);

  while (Wire.available() < 6);

  uint8_t xlg = Wire.read();
  uint8_t xhg = Wire.read();
  uint8_t ylg = Wire.read();
  uint8_t yhg = Wire.read();
  uint8_t zlg = Wire.read();
  uint8_t zhg = Wire.read();

  // combine high and low bytes
  g.x = (int16_t)(xhg << 8 | xlg);
  g.y = (int16_t)(yhg << 8 | ylg);
  g.z = (int16_t)(zhg << 8 | zlg);
}

void L3G::vector_cross(const vector *a,const vector *b, vector *out)
{
  out->x = a->y*b->z - a->z*b->y;
  out->y = a->z*b->x - a->x*b->z;
  out->z = a->x*b->y - a->y*b->x;
}

float L3G::vector_dot(const vector *a,const vector *b)
{
  return a->x*b->x+a->y*b->y+a->z*b->z;
}

void L3G::vector_normalize(vector *a)
{
  float mag = sqrt(vector_dot(a,a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

// Private Methods //////////////////////////////////////////////////////////////

bool L3G::autoDetectAddress(void)
{
  // try each possible address and stop if reading WHO_AM_I returns the expected response
  address = L3G4200D_ADDRESS_SA0_LOW;
  if (readReg(L3G_WHO_AM_I) == 0xD3) return true;
  address = L3G4200D_ADDRESS_SA0_HIGH;
  if (readReg(L3G_WHO_AM_I) == 0xD3) return true;
  address = L3GD20_ADDRESS_SA0_LOW;
  if (readReg(L3G_WHO_AM_I) == 0xD4 || readReg(L3G_WHO_AM_I) == 0xD7) return true;
  address = L3GD20_ADDRESS_SA0_HIGH;
  if (readReg(L3G_WHO_AM_I) == 0xD4 || readReg(L3G_WHO_AM_I) == 0xD7) return true;

  return false;
}





L3G gyro;
L3G gyro2;

//int16_t KID_GYRO_6x = 0;
//int16_t KID_GYRO_6y = 0;
//int16_t KID_GYRO_6z = 0;


int sampleNum=500;
int dc_offsetx=0;
int dc_offsetx2=0;
int dc_offsety=0;
int dc_offsety2=0;
int dc_offsetz=0;
int dc_offsetz2=0;
double noisex=0;
double noisex2=0;
double noisey=0;
double noisey2=0;
double noisez=0;
double noisez2=0;

unsigned long time;
int sampleTime=100;
int rate;
int rate2;



byte Who = 0;
int8_t X_L = 0;
int8_t X_H = 0;
int8_t Y_L = 0;
int8_t Y_H = 0;
int8_t Z_L = 0;
int8_t Z_H = 0;
    
int16_t X_AXIs = 0;
int16_t Y_AXIs = 0;
int16_t Z_AXIs = 0;
   
int G4a = 0;
int G4b = 0;

int G5a = 0;
int G5b = 0;

int Syn = 0;

char UserInput = ' ';
bool SleepMode = 0;
bool ContinuousMode = 0;
bool ReadOnce = 0;
unsigned long LastCommandAt = 0;
bool TimeOutWarned = 0;
char QueuedCommand = ' ';

int TRIG_TOTAL = 2000;
int TRIG_LOW = 1800;
int TRIG_SAFEMAX = 900;
int TRIG_SAFEMIN = 400;

bool isStopping = 0;
unsigned long isStoppingSince = 0;
int loopdelay = 0;

bool WorkingSensors[5] = {true, true, true, true, true};


void display_device_id () {
  Serial.println("MULTIGYROSEN");
}

void display_gyro_gains () {
  Serial.print("GYRO_DEGPERDIGIT: G1=");
  Serial.print(GYRO_DEG_PER_DIGIT, 12);
  Serial.print(" G2=");
  Serial.print(GYRO_DEG_PER_DIGIT, 12);
  Serial.print(" G3=");
  Serial.println(GYRO_DEG_PER_DIGIT, 12);
  
}


void setup ()
{
  bool gyro2successful = false;
  Serial.begin(115200);
  
  display_device_id();
  Serial.println("Serial began");
  
  Serial.println("Setting digital output pins");
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, INPUT);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  
  Serial.println("Setting goniometer pins");
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  analogReference(EXTERNAL);
  
  digitalWrite(6, HIGH);
  delay(100);
  digitalWrite(6, LOW);
  
  Serial.println("Initializing SPI");
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setDataMode(SPI_MODE3);
  SPI.begin();
  //initalize the chip select pins;
  pinMode(CS, OUTPUT);
  WriteRegister(CTRL_REG1,0B00001111);
  WriteRegister(CTRL_REG4,GYRO_CTRL4_VALUE);  // user-defined sensitivity
  delay(10);
  
  Serial.println("Initializing Wire I2C");
  Wire.begin();
  Serial.println("Initializing I2C Gyro");
  int tries = 0;
  while (!gyro.init()) {
    delay(50);
    tries++;
    if (tries > 20) {
      WorkingSensors[0] = false;
      WorkingSensors[1] = false;
      Serial.println("I2C Gyro not available");
      break;
    }
  }
  if (WorkingSensors[0]) {
   Serial.print("Gyro1 found at address: ");
   Serial.println(gyro.getAddress());
   gyro2.init(L3GD20_DEVICE, L3G_SA0_HIGH);
   byte gyro1address = gyro.getAddress();
   byte gyro2address;
   
   if (gyro1address == 107) {
    gyro2address = L3GD20_ADDRESS_SA0_LOW;
    if (gyro2.readReg(L3G_WHO_AM_I) == 0xD4 || gyro2.readReg(L3G_WHO_AM_I) == 0xD7) {
     gyro2.init(L3GD20_DEVICE, L3G_SA0_LOW);
     gyro2successful = 1;
     Serial.print("Gyro2 found at address: ");
     Serial.println(gyro2.getAddress());
    }
   }
   else {
    gyro2address = L3GD20_ADDRESS_SA0_HIGH;
    if (gyro2.readReg(L3G_WHO_AM_I) == 0xD4 || gyro2.readReg(L3G_WHO_AM_I) == 0xD7) {
     gyro2successful = 1;
     gyro2.init(L3GD20_DEVICE, L3G_SA0_HIGH);
     Serial.print("Gyro2 found at address: ");
     Serial.println(gyro2.getAddress());
    }
   }
  }
  if (!gyro2successful) WorkingSensors[1] = false;  
  
  // Override
//  WorkingSensors[1] = false;
  
//  gyro.init(L3GD20_DEVICE, L3G_SA0_HIGH);
//  gyro2.init(L3GD20_DEVICE, L3G_SA0_LOW);
  //WorkingSensors[1] = false;
  
  Serial.println("Enabling Gyro");
  if (WorkingSensors[0]) gyro.enableDefault();
  if (WorkingSensors[1]) gyro2.enableDefault();
  
  Serial.println("Checking Gonio");
  delay(50);
  CheckGonio();
  
  Serial.println("Checking SPI Gyro");
  delay(50);
  CheckSPIGyro();
  
//  Serial.println("Checking kid Arduino Gyro");
//  delay(50);
//  CheckKidArduino();
  
  
  // Turn on power LED
  digitalWrite(6, HIGH);
  LastCommandAt = millis();
  Serial.println("ONLINE");
  DisplayInstruction();
}

//void CheckKidArduino () {
//}

void CheckGonio () {
  G4a = analogRead(A0);
  G4b = analogRead(A1);
  G5a = analogRead(A2);
  G5b = analogRead(A3);
  if (G4a >= 1005 && G4b >= 1005) WorkingSensors[3] = false;
  if (G5a >= 1005 && G5b >= 1005) WorkingSensors[4] = false;
}

void CheckSPIGyro () {
  Who = ReadRegister(WHO_AM_I);
  X_L = ReadRegister(OUT_X_L);
  X_H = ReadRegister(OUT_X_H);
  Y_L = ReadRegister(OUT_Y_L);
  Y_H = ReadRegister(OUT_Y_H);
  Z_L = ReadRegister(OUT_Z_L);
  Z_H = ReadRegister(OUT_Z_H);
  X_AXIs = (int16_t)(X_L | (X_H<<8));
  Y_AXIs = (int16_t)(Y_L | (Y_H<<8));
  Z_AXIs = (int16_t)(Z_L | (Z_H<<8));
  if (X_AXIs == 0 && Y_AXIs == 0 && Z_AXIs == 0) WorkingSensors[2] = false;
}

void DisplayInstruction () {
  Serial.println("Press c to initiate continuous mode");
  Serial.println("Press s to stop continuous mode");
  Serial.println("Press g to get just one reading");
  Serial.println("Press ? to display device ID");
  DisplayStatus();
}

void DisplayStatus () {
  Serial.print("Sensor Status: G1=");  Serial.print(WorkingSensors[0], DEC);
  Serial.print(" G2=");  Serial.print(WorkingSensors[1], DEC);
  Serial.print(" G3=");  Serial.print(WorkingSensors[2], DEC);
  Serial.print(" G4=");  Serial.print(WorkingSensors[3], DEC);
  Serial.print(" G5=");  Serial.println(WorkingSensors[4], DEC);
}


void SendTrigger () {
  if (millis()%TRIG_TOTAL < TRIG_LOW)
    digitalWrite(7, LOW);
  else
    digitalWrite(7, HIGH);
}

void valid_command_entered () {
  LastCommandAt = time;
  if (TimeOutWarned) {
    TimeOutWarned = false;
    digitalWrite(6, HIGH);
  }
}

void loop ()
{
  time = millis();
  if (Serial.available() > 0) {
    UserInput = Serial.read();
    switch (UserInput) {
      case 'c':
        QueuedCommand = 'c';
        valid_command_entered();
        break;
      case 's':
        QueuedCommand = 's';
        valid_command_entered();
        break;
      case 'g':
        ReadOnce = 1;
        valid_command_entered();
        break;
      case '?':
        display_device_id();
        valid_command_entered();
        break;
      case 'n':
        valid_command_entered();
        break;
      case 'd':
        display_gyro_gains();
        valid_command_entered();
        break;
      case 'q':
        DisplayStatus();
        valid_command_entered();
      default:
        break;
    }
    UserInput = ' ';
  }
  
  if (time - LastCommandAt > 30000) {
    if (!SleepMode) {
      ContinuousMode = 0;
      isStopping = 0;
      ReadOnce = 0;
      while (millis()%TRIG_TOTAL >= TRIG_LOW) {delay(1);}
      digitalWrite(6, LOW);
      digitalWrite(7, LOW);
      Serial.println("Device has entered sleep mode. To wake up, enter any valid command.");
      DisplayInstruction();
      SleepMode = 1;
    }
    return;
  }
  else if (SleepMode && time - LastCommandAt <= 4000) {
    Serial.println("Device is now online.");
    digitalWrite(6, HIGH);
    SleepMode = 0;
    while (millis()%TRIG_TOTAL >= TRIG_LOW) {delay(1);}
  }
  else if (time - LastCommandAt > 20000) {
    if (!TimeOutWarned) {
      TimeOutWarned = true;
      Serial.println("Warning: Device will enter sleep mode in 10 seconds. Press n to prevent this.");
    }
    if (time%500 < 250)
      digitalWrite(6, LOW);
    else
      digitalWrite(6, HIGH);
  }

  
  // I2C reads
  if (WorkingSensors[0]) gyro.read();
  if (WorkingSensors[1]) gyro2.read();
  
  // SPI reads
  if (WorkingSensors[2]) {
   Who = ReadRegister(WHO_AM_I);
   X_L = ReadRegister(OUT_X_L);
   X_H = ReadRegister(OUT_X_H);
   Y_L = ReadRegister(OUT_Y_L);
   Y_H = ReadRegister(OUT_Y_H);
   Z_L = ReadRegister(OUT_Z_L);
   Z_H = ReadRegister(OUT_Z_H);
   //ReadSPIGyroBatch();
   X_AXIs = (int16_t)((int16_t)X_L | (((int16_t)X_H)<<8));
   Y_AXIs = (int16_t)((int16_t)Y_L | (((int16_t)Y_H)<<8));
   Z_AXIs = (int16_t)((int16_t)Z_L | (((int16_t)Z_H)<<8));
  }
  
  // Analog reads
  Syn = digitalRead(8);
  G4a = analogRead(A0);
  G4b = analogRead(A1);
  G5a = analogRead(A2);
  G5b = analogRead(A3);
  
  
  
  if (QueuedCommand == 'c' && millis()%TRIG_TOTAL < TRIG_SAFEMAX && millis()%TRIG_TOTAL >= TRIG_SAFEMIN) {
    // Only allow mode change during trig-low
    ContinuousMode = 1;
    isStopping = 0;
    QueuedCommand = ' ';
  }
  else if (QueuedCommand == 's' && millis()%TRIG_TOTAL < TRIG_SAFEMAX && millis()%TRIG_TOTAL >= TRIG_SAFEMIN) {
    isStopping = 1;
    isStoppingSince = millis();
    QueuedCommand = ' ';
  }
  
  if (isStopping && millis() - isStoppingSince >= 200 && millis()%TRIG_TOTAL < TRIG_SAFEMAX && millis()%TRIG_TOTAL >= TRIG_SAFEMIN) {
    // Delay stop a little bit
    ContinuousMode = 0;
    isStopping = 0;
    QueuedCommand = ' ';
  }
  
  if (ContinuousMode) {
    if (!isStopping) SendTrigger();
    DisplayOneReading();
  }
  else if (ReadOnce) {
    DisplayOneReading();
    ReadOnce = 0;
  }
  else {
    digitalWrite(7, LOW);
  }
  
  //Serial.println(millis()-time);
  
  
  loopdelay = 20+time-millis();
  if (loopdelay > 0) delay(loopdelay);
}




//void ReadSPIGyroBatch(){
//  byte Address = OUT_X_L;
//  digitalWrite(SCL, HIGH);
//  digitalWrite(CS, LOW); 
//  SPI.transfer(Address | 0x80 | 0x40);
//  X_L = SPI.transfer(0xFF);
//  X_H = SPI.transfer(0xFF);
//  Y_L = SPI.transfer(0xFF);
//  Y_H = SPI.transfer(0xFF);
//  Z_L = SPI.transfer(0xFF);
//  Z_H = SPI.transfer(0xFF);
//  digitalWrite(CS, HIGH);
////return(result);  
//}



byte ReadRegister(byte Address){
  byte result = 0;
  digitalWrite(CS, LOW); 
  SPI.transfer(Read | Address);
  result = SPI.transfer(0x00);
  //Serial.print(Address, BIN);
  //Serial.print(" : ");
  //Serial.println(result, BIN);
  digitalWrite(CS, HIGH);
return(result);  
}

void WriteRegister(byte Address, byte Value){
  digitalWrite(CS, LOW);
  SPI.transfer(Write | Address);
  SPI.transfer(Value);
  digitalWrite(CS, HIGH);
}


void DisplayOneReading () {
  Serial.print("Time=");
  Serial.print(time);
  Serial.print(" Syn=");
  Serial.print((int)Syn);
  Serial.print(" G1x=");
  Serial.print((int16_t)gyro.g.x);
  Serial.print(" G1y=");
  Serial.print((int16_t)gyro.g.y);
  Serial.print(" G1z=");
  Serial.print((int16_t)gyro.g.z);
  Serial.print(" G2x=");
  Serial.print((int16_t)gyro2.g.x);
  Serial.print(" G2y=");
  Serial.print((int16_t)gyro2.g.y);
  Serial.print(" G2z=");
  Serial.print((int16_t)gyro2.g.z);
  Serial.print(" G3x=");
  Serial.print((int16_t)X_AXIs);
  Serial.print(" G3y=");
  Serial.print((int16_t)Y_AXIs);
  Serial.print(" G3z=");
  Serial.print((int16_t)Z_AXIs);
  Serial.print(" G4a=");
  Serial.print((int)G4a);
  Serial.print(" G4b=");
  Serial.print((int)G4b);
  Serial.print(" G5a=");
  Serial.print((int)G5a);
  Serial.print(" G5b=");
  Serial.print((int)G5b);
//  Serial.print(" G6x=");
//  Serial.print((int16_t)KID_GYRO_6x);
//  Serial.print(" G6y=");
//  Serial.print((int16_t)KID_GYRO_6y);
//  Serial.print(" G6z=");
//  Serial.print((int16_t)KID_GYRO_6z);

//  Serial.print(" G3xL=");
//  Serial.print((int16_t)X_L);
//  Serial.print(" G3xH=");
//  Serial.print((int16_t)X_H);
//  Serial.print(" G3yL=");
//  Serial.print((int16_t)Y_L);
//  Serial.print(" G3yH=");
//  Serial.print((int16_t)Y_H);
//  Serial.print(" G3zL=");
//  Serial.print((int16_t)Z_L);
//  Serial.print(" G3zH=");
//  Serial.print((int16_t)Z_H);
  Serial.println("");
}



