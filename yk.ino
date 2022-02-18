#include "stdio.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include <Wire.h>
#include <TFMPlus.h>  // Include TFMini Plus Library v1.5.0
#include "printf.h" 

#define Motor1 5
#define Motor2 6

TFMPlus tfmP; 

const byte interruptPin1 = 2;
const byte interruptPin2 = 3;

volatile unsigned long pwm_value[2] = {0, 0};
volatile unsigned long timer[2] = {0, 0};

char buffer[100];

MPU9250 accelgyro;
I2Cdev   I2C_M;

void get_one_sample_date_mxyz();
void getAccel_Data(void);

void getACCAngle();
void calcGyroAngle();
void CompFilterAcc();

uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float heading;
float tiltheading;

double Axyz[3];
double Gxyz[3];
double Mxyz[3];
const double DEG_PER_SEC = 32767 / 250;
#define sample_num_mdate  5000

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

double temperature;
double pressure;
double atm;
double altitude;
double dt;
double AccAngleX;
double AccAngleY;
double AccAngleZ;
double GyroAngleX;
double GyroAngleY;
double GyroAngleZ;
double CompFilterAccX;
double CompFilterAccY;
double CompFilterAccZ;
double accel_yz;
double accel_xz;
double sumAcX=0, sumAcY=0, sumAcZ=0;
double sumGyX=0, sumGyY=0, sumGyZ=0, sumheight=0;
double averGyX, averGyY, averGyZ;
double averAcX, averAcY, averAcZ, averheight;
double Accx, Accy, Accz;
int i = 0; //avercount
double t_now;
double t_prev;
int16_t tfDist = 0;    // Distance to object in centimeters
int16_t tfFlux = 0;    // Strength or quality of return signal
int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip

double tstart;
double tstop;
double dInput1, dInput2, dInput3, dInput4;
double roll_target_angle=0.0,  pitch_target_angle=0.0, Yaw_target_angle=0.0, Z_target_height=0;
double prev_roll_angle, prev_pitch_angle, prev_yaw_angle, prev_Z_height;
double pterm1, iterm1, dterm1, pterm2, iterm2, dterm2, pterm3, iterm3, dterm3;
double kp1=0.2, ki1=0.05, kd1=0.05, kp2=0.0, ki2=0.0, kd2=0.0, kp3=0.0, ki3=0.0, kd3=0.0, kp4=0.0, ki4=0.0, kd4=0.0;
double error1, error2, error3, error4;
double Rolloutput, Pitchoutput, Yawoutput;
double height;

double pterm4, iterm4, dterm4;
double Zoutput;
double Height;
double pwm1, pwm2;



double mtrim = 1100;

void setup()
{
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  Serial.begin(19200);
  delay(20);               // Give port time to initalize
  printf_begin();          // Initialize printf.
  accelgyro.initialize();
  Wire.begin();
  Serial1.begin(115200);  //set bit rate of serial port connecting LiDAR with Arduino
  delay(10);
  tfmP.begin(&Serial1);
  delay(10);
  Serial2.begin(19200);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), calcPWM1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), calcPWM2, CHANGE);
  delay(10000);
}

void loop() 
{
  tstart = millis();
  duration();
  initDT();
  getAccel_Data();
  getGyro_Data(); 
  getACCAngle();
  calcDT();
  calcGyroAngle();
  CompFilterAcc();
  caliSensor();
  RollPID();
//  PitchPID();
//  YawPID();
//  tiltheight();
//  heightPID();
  Motormixing();
  
  Serial.println(CompFilterAccX);
//  Serial.println(AccAngleX);
//  Serial.print(CompFilterAccY);
//  Serial.print(",");
//  Serial.print(CompFilterAccZ);
  Serial.print("\n");
  tstart = millis();



  
//  blue();
//  tfminis();

  tstop = millis();
//  Serial.print("time:");
//  Serial.println(tstop-tstart);
  

}


void initDT(){
  t_prev = millis();
}

void calcDT(){
  t_now = millis();
  dt = (t_now - t_prev)/1000.0;
  t_prev = t_now;
}

void Mxyz_init_calibrated ()
{

//    Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
//    Serial.print("  ");
//    Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
//    Serial.print("  ");
//    Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
//    while (!Serial.find("ready"));
//    Serial.println("  ");
//    Serial.println("ready");
//    Serial.println("Sample starting......");
//    Serial.println("waiting ......");


}
void get_calibration_Data ()
{
    for (int i = 0; i < sample_num_mdate; i++)
    {
        get_one_sample_date_mxyz();

        if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
        if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
        if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];

        if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
        if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
        if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];
    }
    mx_max = mx_sample[1];
    my_max = my_sample[1];
    mz_max = mz_sample[1];

    mx_min = mx_sample[0];
    my_min = my_sample[0];
    mz_min = mz_sample[0];
    
    mx_centre = (mx_max + mx_min) / 2;
    my_centre = (my_max + my_min) / 2;
    mz_centre = (mz_max + mz_min) / 2;
}

void getAccel_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = (double) ax / 16384.0;
    Axyz[1] = (double) ay / 16384.0;
    Axyz[2] = (double) az / 16384.0;
}

void getACCAngle(void)
{   
    
    double accel_yz, accel_xz;

    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Accx = (double) ax / 16384.0;
    Accy = (double) ay / 16384.0;
    Accz = (double) az / 16384.0;
    accel_yz = (double) sqrt(pow(Accy, 2) + pow(Accz, 2))*1.0000;
    accel_xz = (double) sqrt(pow(Accx, 2) + pow(Accz, 2))*1.0000;
    AccAngleY = (double) atan(-Accx / accel_yz)*180/PI;
    AccAngleX = (double) atan(Accy / accel_xz)*180/PI;
    AccAngleZ = 0;
    
}

void getGyro_Data(void)
{   
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    Gxyz[0] = (double) (gx-averGyX) * 250 / 32768;
    Gxyz[1] = (double) (gy-averGyY) * 250 / 32768;
    Gxyz[2] = (double) (gz-averGyZ) * 250 / 32768;
}

void calcGyroAngle()
{ 
  GyroAngleX += ((gx - averGyX) / DEG_PER_SEC) * dt;  //각속도로 변환
  GyroAngleY += ((gy - averGyY) / DEG_PER_SEC) * dt;
  GyroAngleZ += ((gz - averGyZ) / DEG_PER_SEC) * dt;
}

void CompFilterAcc()
{
  const double ALPHA = 0.8;
  double tmp_angleX, tmp_angleY, tmp_angleZ; //임시 각도

  if( i == 0 )
  {
    CompFilterAccX = AccAngleX;
    CompFilterAccY = AccAngleY;
    CompFilterAccZ = AccAngleZ;
    
  }

  tmp_angleX = CompFilterAccX + ((gx - averGyX) / DEG_PER_SEC) * dt;
  tmp_angleY = CompFilterAccY + ((gy - averGyY) / DEG_PER_SEC) * dt;
  tmp_angleZ = CompFilterAccZ + ((gz - averGyZ) / DEG_PER_SEC) * dt;
  
  CompFilterAccX = ALPHA * tmp_angleX + (1.0-ALPHA) * AccAngleX;
  CompFilterAccY = ALPHA * tmp_angleY + (1.0-ALPHA) * AccAngleY;
  CompFilterAccZ = tmp_angleZ;

}


void caliSensor()
{
  if(i<10)
  {   
    averGyX=0; averGyY=0; averGyZ=0;
    sumAcX+=ax; sumAcY+=ay; sumAcZ+=az;
    sumGyX+=gx; sumGyY+=gy; sumGyZ+=gz; 
    sumheight+=height; 
     // Serial.println("a");
 
  }
  else if(i=10)
  {
    averGyX=sumGyX/i; averGyY=sumGyY/i; averGyZ=sumGyZ/i;
    averAcX=sumAcX/i; averAcY=sumAcY/i; averAcZ=sumAcZ/i;
    averheight=sumheight/i;
  }
  i++;
}

void tfminis()
{
    if( tfmP.getData( tfDist, tfFlux, tfTemp)) // Get data from the device.
  {
    printf( "Dist:%04icm ", tfDist);   // display distance,
    printf("\r\n");
  }
  else                  // If the command fails...
  {
    tfmP.printFrame();  // display the error and HEX dataa
  }
}

void blue()
{
  if (Serial2.available()) 
  {
    Serial.write(Serial2.read());
  }
  if (Serial.available()) 
  {
    Serial2.write(Serial.read());
  }
}

void RollPID()
{

  error1 =  roll_target_angle - CompFilterAccX; 
  dInput1 = CompFilterAccX - prev_roll_angle;
  pterm1 = kp1 * error1;
  iterm1 += ki1 * error1 * dt;
  dterm1 = -kd1 * (dInput1 / dt);

  Rolloutput = pterm1 + iterm1 + dterm1;
//  Serial.println(Rolloutput);
}

void PitchPID()
{

  error2 =  pitch_target_angle - CompFilterAccY; 
  dInput2 = CompFilterAccY - prev_pitch_angle;
  prev_pitch_angle = CompFilterAccY;

  pterm2 = kp2 * error2;
  iterm2 += ki2 * error2 * dt;
  dterm2 = -kd2 * (dInput2 / dt);

  Pitchoutput = pterm2 + iterm2 + dterm2;
//  Serial.println(Pitchoutput);
}

void YawPID()
{

  error3 =  Yaw_target_angle - CompFilterAccZ; 
  dInput3 = CompFilterAccZ - prev_yaw_angle;
  prev_yaw_angle = CompFilterAccZ;

  pterm3 = kp3 * error3;
  iterm3 += ki3 * error3 * dt;
  dterm3 = -kd3 * (dInput3 / dt);

  Yawoutput = pterm3 + iterm3 + dterm3;
//  Serial.println(Yawoutput);
}

void tiltheight()
{
//  height = tfDist * cos(CompFilterAccX);
    height = tfDist * cos(0);
  Height = height-averheight; // 초기값 빼준 보정 고도
//  Serial.print("Height : ");
//  Serial.println(Height);
//  Serial.print("averheight : ");
//  Serial.println(averheight);
  
}



void heightPID()
{

  error4 =  (Z_target_height) - (height-averheight); 
  dInput4 = height - prev_Z_height;
  prev_Z_height = height;

  pterm3 = kp4 * error4;
  iterm3 += ki4 * error4 * dt;
  dterm3 = -kd4 * (dInput4 / dt);

  Zoutput = pterm4 + iterm4 + dterm4;
//  Serial.print("Heightcmd : ");
//  Serial.println(Zoutput);
 
}

void Motormixing()
{

   pwm1 = mtrim - Rolloutput + Zoutput;
   pwm2 = mtrim + Rolloutput + Zoutput;
  analogWrite(5, pwm1);
  analogWrite(6, pwm2);
//  Serial.print("pwm1 : ");
//  Serial.println(pwm1);
//  Serial.print("pwm2 : ");
//  Serial.println(pwm2);
  
}


void calcPWM1() 
{
  if(digitalRead(interruptPin1) == HIGH) { 
    timer[0] = micros();
    } 
  else {
    if(timer[0] != 0) {
      pwm_value[0] = micros() - timer[0];
      }
    } 
} 

void calcPWM2() 
{
  if(digitalRead(interruptPin2) == HIGH) { 
    timer[1] = micros();
    } 
  else {
    if(timer[1] != 0) {
      pwm_value[1] = micros() - timer[1];
      }
    } 
} 

// Write the duration of PWM
void duration() 
{
//  Serial.println(String(pwm_value[0]) + "   " + String(pwm_value[1]));
//  Z_target_height = map(pwm_value[0], 1100, 2000, 0, 200);
//  roll_target_angle = map(pwm_value[1], 1100, 2000, -20, 20);
//  Serial.println(String(Z_target_height) + "   " + String(roll_target_angle));
}
