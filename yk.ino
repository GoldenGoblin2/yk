#include <Wire.h>

#define Motor1 5
#define Motor2 6

const byte interruptPin1 = 2;
const byte interruptPin2 = 3;
const byte interruptPin3 = 18;

volatile unsigned long pwm_value[3] = {0, 0, 1500};
volatile unsigned long timer[3] = {0, 0, 0};

const int MPU=0x68;
double f64PrevTime_milis;
double f64CurTime_milis;
float f32AccX_ms2;
float f32AccY_ms2;
float f32AccZ_ms2;
float f32Tmp;
float f32GyroXAngv_degs;
float f32GyroYAngv_degs;
float f32GyroZAngv_degs;
float f32AverGyX_degs;
float f32AverGyY_degs;
float f32AverGyZ_degs;
float f32AverHeight_cm;
float f32CompAngleX_deg;
float f32CompAngleY_deg;
float f32CompAngleZ_deg;
float f32SumGyroX;
float f32TimeInterval_s;
float f32Height_cm;
int f32dist_cm;
float f32InitPwm = 128;
float f32Pwm1_pwm, f32Pwm2_pwm;
int i = 0; //avercount
int Count = 0; //CaliCount

float f32SumGyroXAngv_degs, f32SumGyroYAngv_degs, f32SumGyroZAngv_degs, f32SumHeight_cm;
float f32RollCMD_deg=0.0, f32PitchCMD_deg=0.0, f32YawCMD_deg=0.0, f32HeightCMD_cm=0;
float f32PrevRollAngle_deg, f32PrevPitchAngle_deg, f32PrevYawAngle_deg, f32PrevHeight_cm, f32PrevRollRate;
float pterm1, iterm1, dterm1, pterm2, iterm2, dterm2, pterm3, iterm3, dterm3, pterm4, iterm4, dterm4, dterm_prev;
float kp1=1.0, kp1_1=3.50, ki1=0.1, kd1=0.05, kp2=0.0, ki2=0.0, kd2=0.0, kp3=0.0, ki3=0.0, kd3=0.0, kp4=0.0, ki4=0.0, kd4=0.0;
float Rolloutput, Pitchoutput, Yawoutput;
float Zoutput;
float f32PreAngleX_deg;

float f32PwmTrim_pwm = 1300;

unsigned long f64PrevTime_bluemilis = 0;
unsigned long f64CurTime_bluemilis;

unsigned int mainprevtime;
unsigned int maincurtime;

void setup()
{
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  initMPU6050();
  Serial.begin(19200);
  delay(20);               // Give port time to initalize
  // accelgyro.initialize();
  Wire.begin();
  Serial3.begin(115200);  //set bit rate of serial port connecting LiDAR with Arduino
  delay(10);
  Serial2.begin(19200);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), calcPWM1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), calcPWM2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin3), calcPWM3, CHANGE);
  while(true) //Gyro bias calibraion function moved to setup
  {
      readAccelGyro();
      caliSensor();
//      if (i%20 == 0)
//      {
//        Serial.println(i/20);
//      }
      if(i>Count)
      {
        Serial.println("Ready");
//        Serial.println(f32AverGyX_degs);
          break;
      }
  }
//  pinMode(8, OUTPUT);

  unsigned int prevtime = millis();
  
  while(true)
  {
    delay(5);
    if(pwm_value[2]<1150)
    {
      break;
    }
  }
  Serial.println("START");
}

void loop()
{  
  maincurtime = millis();
  if( maincurtime - mainprevtime > 5)
  {
//    blue();
    tfminis();
    duration();
    initDT();
    readAccelGyro();
    calcDT();
    getAngle();
    RollPID();
  //  PitchPID();
  //  YawPID();
  //  tiltheight();
  //  heightPID();
    Motormixing();
//    Serial.print("deg : ");
//    Serial.print(f32CompAngleX_deg);
//    Serial.print("   pwm1 : ");
//    Serial.print(f32Pwm1_pwm);
//    Serial.print("   pwm2 : ");
//    Serial.println(f32Pwm2_pwm);
//
//    mainprevtime = maincurtime;
  }
//  analogWrite(5, f32Pwm1_pwm);
//  analogWrite(6, f32Pwm2_pwm);
}

void initDT()
{
  f64PrevTime_milis = millis();
}

void calcDT()
{
  f64CurTime_milis = millis();
  f32TimeInterval_s = (f64CurTime_milis - f64PrevTime_milis)/1000.0;
  f64PrevTime_milis = f64CurTime_milis;
}

void readAccelGyro()
{
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 14, true);
    f32AccX_ms2 = (Wire.read() << 8 | Wire.read()) / 16384.0 * 9.81 ;//16384는 분해능이 +-2g m/s^2을 만들기 위해서 9.81곱함
    f32AccY_ms2 = (Wire.read() << 8 | Wire.read()) / 16384.0 * 9.81 ;
    f32AccZ_ms2 = (Wire.read() << 8 | Wire.read()) / 16384.0 * 9.81 ;
    f32Tmp = (Wire.read() << 8 | Wire.read()) ; // 340;//340은 섭씨 단위로 바꾸기 위해
    f32GyroXAngv_degs = (Wire.read() << 8 | Wire.read()) / 131.0; //131은 degree/sec으로 만드는 분해능
    f32GyroYAngv_degs = (Wire.read() << 8 | Wire.read()) / 131.0;
    f32GyroZAngv_degs = (Wire.read() << 8 | Wire.read()) / 131.0;
    Serial.println(f32GyroXAngv_degs);
}

void initMPU6050()
{
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);
}

void getAngle()
{
  const float ALPHA = 0.996;
  float f32AccYZ_ms2, f32AccXZ_ms2, f32AccAngleY_deg, f32AccAngleX_deg, f32AccAngleZ_deg;
  float f32TmpAngleX_deg, f32TmpAngleY_deg, f32TmpAngleZ_deg;

  f32AccYZ_ms2 = sqrtf(powf(f32AccY_ms2, 2) + powf(f32AccZ_ms2, 2))*1.0000;
  f32AccXZ_ms2 = sqrtf(powf(f32AccX_ms2, 2) + powf(f32AccZ_ms2, 2))*1.0000;
  f32AccAngleY_deg = atanf(-f32AccX_ms2 / f32AccYZ_ms2)*180/PI;
  f32AccAngleX_deg = atanf(f32AccY_ms2 / f32AccXZ_ms2)*180/PI;
  f32AccAngleZ_deg = 0;

  if( i == 0 )
  {
    f32CompAngleX_deg = f32AccAngleX_deg;
    f32CompAngleY_deg = f32AccAngleY_deg;
    f32CompAngleZ_deg = f32AccAngleZ_deg;
  }
  
  f32TmpAngleX_deg = f32CompAngleX_deg + (f32GyroXAngv_degs - f32AverGyX_degs) * f32TimeInterval_s;
  f32TmpAngleY_deg = f32CompAngleY_deg + (f32GyroYAngv_degs - f32AverGyY_degs) * f32TimeInterval_s;
  f32TmpAngleZ_deg = f32CompAngleZ_deg + (f32GyroZAngv_degs - f32AverGyZ_degs) * f32TimeInterval_s;
  
  f32CompAngleX_deg = ALPHA * f32TmpAngleX_deg + (1.0-ALPHA) * f32AccAngleX_deg;
  f32CompAngleY_deg = ALPHA * f32TmpAngleY_deg + (1.0-ALPHA) * f32AccAngleY_deg;
  f32CompAngleZ_deg = f32TmpAngleZ_deg;

  if(isnan(f32CompAngleX_deg))
  {
    f32CompAngleX_deg = f32PreAngleX_deg;;
  }
  f32PreAngleX_deg = f32CompAngleX_deg;

}

void caliSensor()
{

  if(i<Count)
  {   
    f32AverGyX_degs=0; f32AverGyY_degs=0; f32AverGyZ_degs=0;
    f32SumGyroXAngv_degs += f32GyroXAngv_degs;
    f32SumGyroYAngv_degs += f32GyroYAngv_degs;
    f32SumGyroZAngv_degs += f32GyroZAngv_degs;
    f32SumHeight_cm += f32Height_cm;
  }
  else if(i==Count)
  {
    f32AverGyX_degs = f32SumGyroXAngv_degs/i;
    f32AverGyY_degs = f32SumGyroYAngv_degs/i;
    f32AverGyZ_degs = f32SumGyroZAngv_degs/i;
    f32AverHeight_cm = f32SumHeight_cm/i;
  }
  i++;
}

void blue()
{
  f64CurTime_bluemilis = millis();
  if( f64CurTime_bluemilis - f64PrevTime_bluemilis > 100)
  {
    Serial2.print("@");
    float f32blueangle_deg = 0;
    if( f32CompAngleX_deg < 0 )
    {
      f32blueangle_deg = abs(f32CompAngleX_deg);
      Serial2.print("-");
    }
    else
    {
      Serial2.print("+");
    }
    int f32intpart = (int)f32blueangle_deg;
    int f32decimalpart = round((f32blueangle_deg - f32intpart)*100);
//    Serial.print("int : ");
    checkprintHEX(f32intpart);
//    Serial.print("decimal : ");
    checkprintHEX(f32decimalpart);
//    Serial.print("dist : ");
    checkprintHEX(f32dist_cm);
    int f32sumpart = f32intpart + f32decimalpart + f32dist_cm;
    if( f32sumpart > 256 )
    {
      f32sumpart -= 256;
    }
    checkprintHEX(f32sumpart);
    f64PrevTime_bluemilis = f64CurTime_bluemilis;
  }
  
}

int checkprintHEX(int inputnum)
{
  if(inputnum < 17)
  {
    Serial2.print("0");
  }
  Serial2.print( inputnum, HEX);
}

void RollPID()
{
  float f32RollError_deg, f32RollRateCMD, f32RollRateError, f32RollRateDelta, tau = 0.9, dterm_now;
 
  f32RollError_deg = f32RollCMD_deg - f32CompAngleX_deg;
  f32RollRateCMD = kp1 * f32RollError_deg;
  
  f32RollRateError = f32RollRateCMD - f32GyroXAngv_degs - f32AverGyX_degs;
  f32RollRateDelta = f32GyroXAngv_degs - f32AverGyX_degs - f32PrevRollRate;
  f32PrevRollRate = f32GyroXAngv_degs - f32AverGyX_degs;
  
  pterm1 = kp1_1 * f32RollRateError;
  iterm1 += ki1 * f32RollRateError * f32TimeInterval_s;
  dterm_now = -kd1 * (f32RollRateDelta / f32TimeInterval_s);
  dterm1 = tau*dterm_prev  + (1-tau)*dterm_now;
  dterm_prev = dterm_now;
  iterm1 = min(iterm1, 200);
  dterm1 = min(dterm1, 200);
  iterm1 = max(-200, iterm1);
  dterm1 = max(-200, dterm1);

  Rolloutput = pterm1 + iterm1 + dterm1;
  Serial.print("NowAngle : ");
  Serial.print(f32CompAngleX_deg);
  Serial.print("     CMDAngle : ");
  Serial.print(f32RollCMD_deg);
  Serial.print("     Error : ");
  Serial.println(f32RollError_deg);

  Serial.print("NowGyroX : ");
  Serial.print(f32GyroXAngv_degs - f32AverGyX_degs);
  Serial.print("     CMDAngv : ");
  Serial.print(f32RollRateCMD);
  Serial.print("     Error : ");
  Serial.println(f32RollRateError);
  Serial.println("--------------------");
//  
//  Serial.print("Rolloutput : ");
//  Serial.print(Rolloutput);
//  Serial.print("     pterm1 : ");
//  Serial.print(pterm1);
//  Serial.print("     iterm1 : ");
//  Serial.print(iterm1);
//  Serial.print("     dterm1 : ");
//  Serial.println(dterm1);
}



//void RollPID()
//{
//  float f32RollError_deg, f32RollDelta_deg;
//
//  f32RollError_deg =  f32RollCMD_deg - f32CompAngleX_deg;
//  f32RollDelta_deg = f32CompAngleX_deg - f32PrevRollAngle_deg;
//  f32PrevRollAngle_deg = f32CompAngleX_deg;
//
//  pterm1 = kp1 * f32RollError_deg;
//  iterm1 += ki1 * f32RollError_deg * f32TimeInterval_s;
//  dterm1 = -kd1 * (f32RollDelta_deg / f32TimeInterval_s);
//
//  Rolloutput = pterm1 + iterm1 + dterm1;
////  Serial.print("Rolloutput : ");
////  Serial.println(Rolloutput);
////  Serial.print("pterm1 : ");
////  Serial.println(pterm1);
////  Serial.print("iterm1 : ");
////  Serial.println(iterm1);
////  Serial.print("dterm1 : ");
////  Serial.println(dterm1);
////  Serial.print("NowAngle : ");
////  Serial.println(f32CompAngleX_deg);
////  Serial.println("------------");
//}



/*
void PitchPID()
{
  float f32PitchError_deg, f32PitchDelta_deg;

  f32PitchError_deg =  f32PitchCMD_deg - f32CompAngleY_deg;
  f32PitchDelta_deg = f32CompAngleY_deg - f32PrevPitchAngle_deg;
  f32PrevPitchAngle_deg = f32CompAngleY_deg;

  pterm2 = kp2 * f32PitchError_deg;
  iterm2 += ki2 * f32PitchError_deg * f32TimeInterval_s;
  dterm2 = -kd2 * (f32PitchDelta_deg / f32TimeInterval_s);

  Pitchoutput = pterm2 + iterm2 + dterm2;
//  Serial.println(Pitchoutput);
}

void YawPID()
{
  float f32YawError_deg, f32YawDelta_deg;

  f32YawError_deg =  f32YawCMD_deg - f32CompAngleZ_deg;
  f32YawDelta_deg = f32CompAngleZ_deg - f32PrevYawAngle_deg;
  f32PrevYawAngle_deg = f32CompAngleZ_deg;

  pterm3 = kp3 * f32YawError_deg;
  iterm3 += ki3 * f32YawError_deg * f32TimeInterval_s;
  dterm3 = -kd3 * (f32YawDelta_deg / f32TimeInterval_s);

  Yawoutput = pterm3 + iterm3 + dterm3;
//  Serial.println(Yawoutput);
}

void tiltheight()
{
//  height = (float)f32dist_cm * cos(CompFilterAccX);
  height = (float)f32dist_cm * cos(0.0);
  Height = height-averheight; // 초기값 빼준 보정 고도
//  Serial.print("Height : ");
//  Serial.println(Height);
//  Serial.print("averheight : ");
//  Serial.println(averheight);
  
}
*/

void heightPID()
{
  float f32HeightError_cm, f32HeightDelta_cm;

  f32HeightError_cm =  f32HeightCMD_cm - (f32Height_cm - f32AverHeight_cm);
  f32HeightDelta_cm = f32Height_cm - f32PrevHeight_cm;
  f32PrevHeight_cm = f32Height_cm;

  pterm4 = kp4 * f32HeightError_cm;
  iterm4 += ki4 * f32HeightError_cm  * f32TimeInterval_s;
  dterm4 = -kd4 * (f32HeightDelta_cm / f32TimeInterval_s);

  Zoutput = pterm4 + iterm4 + dterm4;
//  Serial.print("Heightcmd : ");
//  Serial.println(Zoutput);

}

void Motormixing()
{
  f32Pwm1_pwm = f32PwmTrim_pwm - Rolloutput;// + Zoutput;
  f32Pwm2_pwm = f32PwmTrim_pwm + Rolloutput;// + Zoutput;
  f32Pwm1_pwm = min(f32Pwm1_pwm, 1800);
  f32Pwm2_pwm = min(f32Pwm2_pwm, 1800);
  f32Pwm1_pwm = max(1200, f32Pwm1_pwm);
  f32Pwm2_pwm = max(1200, f32Pwm2_pwm);
  f32Pwm1_pwm = map(f32Pwm1_pwm,1200,1800,148 ,230);
  f32Pwm2_pwm = map(f32Pwm2_pwm,1200,1800,148 ,230);
}

void calcPWM1()
{
  if(digitalRead(interruptPin1) == HIGH) 
  { 
    timer[0] = micros();
  } 
  else 
  {
    if(timer[0] != 0) 
    {
      pwm_value[0] = micros() - timer[0];
    }
  } 
} 

void calcPWM2()
{
  if(digitalRead(interruptPin2) == HIGH) 
  { 
    timer[1] = micros();
  } 
  else 
  {
    if(timer[1] != 0) 
    {
      pwm_value[1] = micros() - timer[1];
//      f32RollCMD_deg = map(pwm_value[1], 1100, 2000, -20, 20);
    }
  } 
} 

void calcPWM3()
{
  if(digitalRead(interruptPin3) == HIGH) 
  { 
    timer[2] = micros();
  } 
  else 
  {
    if(timer[2] != 0) 
    {
      pwm_value[2] = micros() - timer[2];
    }
    if(pwm_value[2] > 1900)
    {
      exit(0);
    }
  } 
} 

// Write the duration of PWM
void duration()
{
//  Serial.println(String(pwm_value[0]) + "   " + String(pwm_value[1]) + "   " + String(pwm_value[2]));
  f32HeightCMD_cm = map(pwm_value[0], 1100, 2000, 0, 200);
  
//  Serial.println(String(Z_target_height) + "   " + String(roll_target_angle));
}

void tfminis()
{
  int uart[9] = {0};
  const int HEADER=0x59;
  if(Serial3.available()>0)
  {
    if(Serial3.read() == HEADER)
    {
      for (int j = 0; j < 8; j++) 
      { 
        uart[j] = Serial3.read();
      }
      if( uart[1] != -1)
      {
        f32dist_cm = uart[1];
      }
//      Serial.print("dist = ");
//      Serial.println(f32dist_cm);
    }
  }
}
