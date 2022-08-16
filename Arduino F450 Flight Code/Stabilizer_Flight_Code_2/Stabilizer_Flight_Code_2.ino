
//TO DO LIST
//1. kalman setpoint adjustment (roll, pitch might be switched)
//2. Check kalman getRate and switch if necessary
//3. Check if all variables related to rate PID works as expected





//#include <ros.h>
//#include <test_gazebo/serialdata_msg.h>
//
//ros::NodeHandle  nh;
//
//test_gazebo::serialdata_msg log_msg;
//ros::Publisher logger("logger", &log_msg);


#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;


#include "I2Cdev.h"
#include "MPU6050.h"
#include "MS5611.h"

File myFile;
const int chipSelect = 53;

unsigned long time;


const int MPU = 0x68; // MPU6050 I2C address


byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5, last_channel_6;

int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4, receiver_input_channel_5, receiver_input_channel_6;

unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, timer_6;





float control_r_s = 0;
float control_p_s = 0;

int counterRF = 0;
int counterPF = 0;
int counterYF = 0;
int counterGXF = 0;
int counterGYF = 0;


bool set = false;
bool armed = false;
float t_start = 0;
float t = 0;


float AccX, AccY, AccZ;
float accAngleX, accAngleY;

float GyroX, GyroY, GyroZ;
float gyroAngleX, gyroAngleY, gyroAngleZ;


float roll,pitch,yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;

float kal_roll;
float kal_pitch;

int c = 0;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data


MS5611 MS5611(0x77);

Servo motor1;     // create servo object to control the ESC
Servo motor2;
Servo motor3;
Servo motor4;
bool calibration = true;

int receiver_pins[] = {A8, A9, A10, A11, A12, A13, A14};
int receiver_values[] = {0, 0, 0, 0, 0, 0};
//CH0 (Throttle): 1000~2000
//CH1 (Roll): 1260~1752 => -a~a
//CH2 (Pitch): 1240~1740 => -b~b
//CH3 (Yaw): 1240~1740 => -c~c

double x_des = 0;
double y_des = 0;
double z_des = 2;
double roll_des = 0;
double pitch_des = 0;
double yaw_des = 0;
double rollrate_des;
double pitchrate_des;
//double des_roll_hat;
//double des_pitch_hat;
double x =0;
double y =0;
double z =0;
double z_i = 0;
double z_it = 0;
double z_p = 0;
double prev_z = 0;
double filtered_z = 0;
double prev_filtered_z = 0;
double imaginary_z = 0;

double T;
double P;

int32_t x_i = 0;
int32_t y_i = 0;

#define LED_PIN 13
bool blinkState = false;
float Ts = 0.01;
//double error_sum_x = 0;
//double error_sum_y = 0;
//double error_sum_z = 0;

//double error_x = 0;
//double error_y = 0;
//double error_z = 0;
double error_roll = 0;
double error_pitch = 0;
double error_yaw = 0;


double error_rollrate = 0;
double error_pitchrate = 0;
double error_yawrate = 0;

double error_rollrate_prev = 0;
double error_pitchrate_prev = 0;
double error_yawrate_prev = 0;

double error_rollrate_sum = 0;
double error_pitchrate_sum = 0;
double error_yawrate_sum = 0;



//double kpx;
//double kpy;
//double kpz;
double kpr;
double kpp;
double kpyaw;
//double kix;
//double kiy;
//double kiz;
double kir;
double kip;
double kiyaw;
//double kdx;
//double kdy;
//double kdz;
//double kdr;
double kdp;
double kdyaw;

double kpr_rate;
double kpp_rate;
double kpyaw_rate;
double kir_rate;
double kip_rate;
double kiyaw_rate;
double kdr_rate;
double kdp_rate;
double kdyaw_rate;

double setpoint_roll;
double setpoint_pitch;

double rollrate;
double pitchrate;


int m1 = 0;
int m2 = 0;
int m3 = 0;
int m4 = 0;

int r_s = 0;
int p_s = 0;
int y_s = 0;
int t_s = 0;
int filtered_yaw = 0;

int arm_switch;

int cali = 1;


void setup() {

PCICR |= (1 << PCIE2 );

PCMSK2 |= (1 << PCINT16);
PCMSK2 |= (1 << PCINT17);
PCMSK2 |= (1 << PCINT18);
PCMSK2 |= (1 << PCINT19);
PCMSK2 |= (1 << PCINT20);
PCMSK2 |= (1 << PCINT21);
PCMSK2 |= (1 << PCINT22);

//  nh.initNode();
//  nh.advertise(logger);
  
  
  MS5611.begin();
  Serial.begin(115200);

#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

  
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  pinMode(LED_PIN, OUTPUT);
  motor1.attach(2, 1000, 2000);
  motor2.attach(5, 1000, 2000);
  motor3.attach(3, 1000, 2000);
  motor4.attach(6, 1000, 2000);


  t_start = millis();
}

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
////////////////////////////MAIN CODE//////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

void loop() {



t_s = map(receiver_input_channel_1, 1112, 2108, 1000, 2000);
r_s = map(receiver_input_channel_2, 992, 1984, 1000, 2000);
p_s = map(receiver_input_channel_3, 988, 1980, 1000, 2000);
y_s = map(receiver_input_channel_4, 972, 1968, 1000, 2000);

arm_switch = receiver_input_channel_6;

setpoint_roll = map(r_s, 1000, 2000, -40, 40);
setpoint_pitch = map(p_s, 1000, 2000, -40, 40);

getAttitude();
attitudePID();
      
if (arm_switch < 1500 && t_s < 1100) {


    m1 = 1000;
    m2 = 1000;
    m3 = 1000;
    m4 = 1000;

}

if (arm_switch < 1500 && t_s > 1100) {
  

    m1 = t_s;
    m2 = t_s;
    m3 = t_s;
    m4 = t_s;

    if (t_s > 1900)
    {
    m1 = 2000;
    m2 = 2000;
    m3 = 2000;
    m4 = 2000;    
    }
    
}
    


if (arm_switch > 1500) {

    t = millis();

      if (t_s < 1100) {
        

    m1 = 1000;
    m2 = 1000;
    m3 = 1000;
    m4 = 1000;

      
      }


      
    else if (t_s > 1100) {
      
      if (t_s > 1800) {t_s = 1800;}

      

  
      m1 = 0*(int)150*control_r_s - (int)150*control_p_s + 0*y_s + t_s;
      m2 = -0*(int)150*control_r_s - (int)150*control_p_s - 0*y_s + t_s;
      m3 = -0*(int)150*control_r_s + (int)150*control_p_s + 0*y_s + t_s;
      m4 = 0*(int)150*control_r_s + (int)150*control_p_s - 0*y_s + t_s;

      if (m1 < 1100) {m1 = 1100;}
      if (m2 < 1100) {m2 = 1100;}
      if (m3 < 1100) {m3 = 1100;}
      if (m4 < 1100) {m4 = 1100;}

      if (m1 > 2000) {m1 = 2000;}
      if (m2 > 2000) {m2 = 2000;}
      if (m3 > 2000) {m3 = 2000;}
      if (m4 > 2000) {m4 = 2000;}

    }
      


      Serial.print(p_s);
      Serial.print(",  ");
      Serial.print(setpoint_pitch);
      Serial.print(",  ");      
      Serial.print(kal_pitch);
      Serial.print(",  ");
      Serial.print(pitchrate_des);
      Serial.print(",  ");
      Serial.print(pitchrate);
      Serial.print(",  ");
      Serial.print(kpp_rate*error_pitchrate);
      Serial.print(",  ");
      Serial.print(kdp_rate*(error_pitchrate - error_pitchrate_prev));
      Serial.print(",  ");
      Serial.print(control_p_s);
      Serial.print(",  ");
      Serial.print(m1);
      Serial.print(",  ");
      Serial.print(m2);
      Serial.print(",  ");
      Serial.print(m3);
      Serial.print(",  ");
      Serial.print(m4);
      Serial.print(",  ");
      Serial.println((t-t_start)/1000);


      
    } 
        
      motor1.writeMicroseconds(m1);
      motor2.writeMicroseconds(m2);
      motor3.writeMicroseconds(m3);
      motor4.writeMicroseconds(m4);


  
}



void attitudePID() {

  kpr = 1.0;
  kpp = 0.5;

  kpr_rate = 0;
  kir_rate = 0;
  kdr_rate = 0;
  
  kpp_rate = 0.5;
  kip_rate = 0;
  kdp_rate = 0.0015;
  
  error_roll = setpoint_roll - kal_roll;
  rollrate_des = (kpr)*error_roll;
  
  error_pitch = setpoint_pitch - kal_pitch;
  pitchrate_des = (kpp)*error_pitch; 
 


  error_rollrate = rollrate_des - rollrate;
  error_pitchrate = pitchrate_des - pitchrate;

  error_rollrate_sum += error_rollrate;
  error_pitchrate_sum += error_pitchrate;

  control_r_s = kpr_rate*error_rollrate + kir_rate*error_rollrate_sum*Ts + (kdr_rate*(error_rollrate - error_rollrate_prev));
  control_p_s = kpp_rate*error_pitchrate + kip_rate*error_pitchrate_sum*Ts + (kdp_rate*(error_pitchrate - error_pitchrate_prev)/Ts);

  if (control_r_s < -1) {control_r_s = -1;}
  else if (control_r_s > 1) {control_r_s = 1;}
  
  if (control_p_s < -1) {control_p_s = -1;}
  else if (control_p_s > 1) {control_p_s = 1;}
  
  error_rollrate_prev = error_rollrate;
  error_pitchrate_prev = error_pitchrate;

  }


void getAltitudeP() {
  
    MS5611.read();
    MS5611.setOversampling(OSR_ULTRA_HIGH);
    P = MS5611.getPressure();
    T = MS5611.getTemperature() + 273.15;
    z_p = ((8.31432*T)/(9.80665*0.0289644))*log(1013.25/P);
//    Serial.println(z_p);
  }


  


ISR(PCINT2_vect) {
//Channel 1========================================
if (last_channel_1 == 0 && PINK & B00000001) {
  last_channel_1 = 1;
  timer_1 = micros();
  }

else if (last_channel_1 == 1 && !(PINK & B00000001)) {
  last_channel_1 = 0;
  receiver_input_channel_1 = micros() - timer_1;
  }


//Channel 2========================================
if (last_channel_2 == 0 && PINK & B00000010) {
  last_channel_2 = 1;
  timer_2 = micros();
  }

else if (last_channel_2 == 1 && !(PINK & B00000010)) {
  last_channel_2 = 0;
  receiver_input_channel_2 = micros() - timer_2;
  }

//Channel 3========================================
if (last_channel_3 == 0 && PINK & B00000100) {
  last_channel_3 = 1;
  timer_3 = micros();
  }

else if (last_channel_3 == 1 && !(PINK & B00000100)) {
  last_channel_3 = 0;
  receiver_input_channel_3 = micros() - timer_3;
  }

//Channel 4========================================
if (last_channel_4 == 0 && PINK & B00001000) {
  last_channel_4 = 1;
  timer_4 = micros();
  }

else if (last_channel_4 == 1 && !(PINK & B00001000)) {
  last_channel_4 = 0;
  receiver_input_channel_4 = micros() - timer_4;
  }

//Channel 5========================================
if (last_channel_5 == 0 && PINK & B00010000) {
  last_channel_5 = 1;
  timer_5 = micros();
  }

else if (last_channel_5 == 1 && !(PINK & B00010000)) {
  last_channel_5 = 0;
  receiver_input_channel_5 = micros() - timer_5;
  }

//Channel 6========================================
if (last_channel_6 == 0 && PINK & B00100000) {
  last_channel_6 = 1;
  timer_6 = micros();
  }

else if (last_channel_6 == 1 && !(PINK & B00100000)) {
  last_channel_6 = 0;
  receiver_input_channel_6 = micros() - timer_6;
  }





  
}
