
#include <ros.h>
#include <test_gazebo/serialdata_msg.h>

ros::NodeHandle  nh;

test_gazebo::serialdata_msg log_msg;
ros::Publisher logger("logger", &log_msg);


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
double des_roll_hat;
double des_pitch_hat;
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
double error_sum_x = 0;
double error_sum_y = 0;
double error_sum_z = 0;
double error_sum_roll = 0;
double error_sum_pitch = 0;
double error_sum_yaw = 0;

double prev_error_x = 0;
double prev_error_y = 0;
double prev_error_z = 0;
double prev_error_roll = 0;
double prev_error_pitch = 0;
double prev_error_yaw = 0;


double error_x = 0;
double error_y = 0;
double error_z = 0;
double error_roll = 0;
double error_pitch = 0;
double error_yaw = 0;

double error_diff_x = 0;
double error_diff_y = 0;
double error_diff_z = 0;
double error_diff_roll = 0;
double error_diff_pitch = 0;
double error_diff_yaw = 0 ;

double kpx;
double kpy;
double kpz;
double kpr;
double kpp;
double kpyaw;
double kix;
double kiy;
double kiz;
double kir;
double kip;
double kiyaw;
double kdx;
double kdy;
double kdz;
double kdr;
double kdp;
double kdyaw;


int m1 = 0;
int m2 = 0;
int m3 = 0;
int m4 = 0;

int r_s = 0;
int p_s = 0;
int y_s = 0;
int t_s = 0;
int filtered_yaw = 0;

//////////////////////////Moving Average Filter Variables////////////////////////
double filtered_t = 0;
double tbuff[10];
int n1 = 10;
double sum1 = 0;
int counter_1 = 1;

double filtered_r = 0;
double rbuff[10];
int n2 = 10;
double sum2 = 0;
int counter_2 = 1;

double filtered_p = 0;
double pbuff[10];
int n3 = 10;
double sum3 = 0;
int counter_3 = 1;


double prev_yaw = 0;

double ybuff[10];
int n4 = 10;
double sum4 = 0;
int counter_4 = 1;

int arm_switch;



////////////////////////////////////////////////////////////////////////////////



void setup() {

//  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(logger);
  
  tbufffill();
  rbufffill();
  pbufffill();
  ybufffill();  
  
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


//  Serial.print("Initializing SD card...");
//
//  if (!SD.begin()) {
//    Serial.println("initialization failed!");
////    return;
//  }
//  Serial.println("initialization done.");
//  
//  myFile = SD.open("test.txt", FILE_WRITE);
//
//  // if the file opened okay, write to it:
//  if (myFile) {
//    myFile.println("testing 1, 2, 3.");
//    myFile.close();
//  } else {
//    // if the file didn't open, print an error:
//    Serial.println("error opening test.txt");
//    
//  }
    
  
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
    receive();
    motor1.writeMicroseconds(1000);
    motor2.writeMicroseconds(1000);
    motor3.writeMicroseconds(1000);
    motor4.writeMicroseconds(1000);

while (arm_switch > 1500) 
  {
    
    t = millis();





//    myFile = SD.open("test.txt", FILE_WRITE);
    receive();
    //logging();
    getAttitude();


 
    
    if ((r_s < 50 && r_s > -50) && (p_s < 50 && p_s > -50)) {

      attitudePID();
      m1 = 250*control_r_s - 250*control_p_s + y_s + t_s;
      m2 = -250*control_r_s - 250*control_p_s - y_s + t_s;
      m3 = -250*control_r_s + 250*control_p_s + y_s + t_s;
      m4 = 250*control_r_s + 250*control_p_s - y_s + t_s;
      
     if (m1 < 1000) {m1 = 1000;}
    else if (m1 > 2000) {m1 = 2000;}
  
    if (m2 < 1000) {m2 = 1000;}
    else if (m2 > 2000) {m2 = 2000;}
    
    if (m3 < 1000) {m3 = 1000;}
    else if (m3 > 2000) {m3 = 2000;}
    
    if (m4 < 1000) {m4 = 1000;}
    else if (m4 > 2000) {m4 = 2000;}
      
      
      }
    
    else {
      m1 = r_s - p_s + y_s + t_s;
      m2 = -r_s - p_s - y_s + t_s;
      m3 = -r_s + p_s + y_s + t_s;
      m4 = r_s + p_s - y_s + t_s;


           if (m1 < 1000) {m1 = 1000;}
    else if (m1 > 2000) {m1 = 2000;}
  
    if (m2 < 1000) {m2 = 1000;}
    else if (m2 > 2000) {m2 = 2000;}
    
    if (m3 < 1000) {m3 = 1000;}
    else if (m3 > 2000) {m3 = 2000;}
    
    if (m4 < 1000) {m4 = 1000;}
    else if (m4 > 2000) {m4 = 2000;}
      
    } 

      motor1.writeMicroseconds(m1);
      motor2.writeMicroseconds(m2);
      motor3.writeMicroseconds(m3);
      motor4.writeMicroseconds(m4);

      
      
        logging();

        
//      Serial.print(kal_roll);
//      Serial.print(",  ");
//      Serial.print(kal_pitch);
//      Serial.print(",  ");
//      Serial.print(control_r_s);
//      Serial.print(",  ");
//      Serial.print(control_p_s);
//      Serial.print(",  ");
//      Serial.print(m1);
//      Serial.print(",  ");
//      Serial.print(m2);
//      Serial.print(",  ");
//      Serial.print(m3);
//      Serial.print(",  ");
//      Serial.print(m4);
//      Serial.print(",  ");
//      Serial.println((t-t_start)/1000);
  
    }
}



void attitudePID() {
  kpr = 2.5;
  kir = 0;
  kdr = 0.1;

  kpp = 2.5;
  kip = 0;
  kdp = 0.1;

  kpyaw = 1;
  kiyaw = 0;
  kdyaw = 0.06;
  
  error_roll = roll_des- (kal_roll*PI/180);
  error_sum_roll += error_roll*Ts;
  error_diff_roll = error_roll - prev_error_roll;
  
  if (error_sum_roll > 30) {
    error_sum_roll = 30;
    }
  if (error_sum_roll < -30) {
    error_sum_roll = -30;
    }
  
  control_r_s = (kpr)*error_roll + (kir)*(error_sum_roll) + (1/Ts)*(error_diff_roll)*(kdr) ;
  if (control_r_s < -1) 
  {
    control_r_s = -1;
  }

  if (control_r_s > 1) 
  {
    control_r_s = 1;  
  }
  
  
  
  error_pitch = pitch_des - (kal_pitch*PI/180);
  error_sum_pitch += error_pitch*Ts;
  error_diff_pitch = error_pitch - prev_error_pitch;
   
   if (error_sum_pitch > 30) {
    error_sum_roll = 30;
    }
  if (error_sum_pitch < -30) {
    error_sum_pitch = -30;
    }
  
  control_p_s = (kpp)*error_pitch + (kip)*(error_sum_pitch) + (1/Ts)*(error_diff_pitch)*(kdp) ;

  if (control_p_s < -1) 
  {
    control_p_s = -1;
  }

  if (control_p_s > 1) 
  {
    control_p_s = 1;  
  }
    

 
  prev_error_roll = error_roll;
  prev_error_pitch = error_pitch;


  }



/////////////////////////////////////GET RC VALUE//////////////////////////////////////

//////////////////////ARMING AND DISARMING////////////////////////
//void arm() {
//  if (arm_switch > 1500) {
//      armed = true;
//      }
//  else {
//      armed = false;    
//    }
//   }
////////////////////////////////////////////////////////////////



  ////NEED MAF for RC Signals!////

void tbufffill() {
    double tbuff[n1];
  for (int i = 0; i < n1; i++) {
     tbuff[i] = 0;
    }
}

void rbufffill() {
    double rbuff[n2];
  for (int i = 0; i < n2; i++) {
     rbuff[i] = 0;
    }
}

void pbufffill() {
    double pbuff[n3];
  for (int i = 0; i < n3; i++) {
     pbuff[i] = 0;
    }
}

void ybufffill() {
    double ybuff[n4];
  for (int i = 0; i < n4; i++) {
     ybuff[i] = 0;
    }
}

int MAFT() {
  sum1 = 0;
//  for (int i = 0; i<n; i++) {
//    Serial.print(tbuff[i]);
//    Serial.print(", ");
//    if (i==n-1) {
//      Serial.println(tbuff[i]);
//      }
//  }
  if (counter_1 < n1) {
  tbuff[counter_1-1] = receiver_values[0];
    filtered_t = receiver_values[0];
  }
  
  else if (counter_1 == n1) {
    for (int i = 0; i < n1; i++) {
      sum1 += tbuff[i];
    }
    filtered_t = sum1/n1;
  }
  
  else if (counter_1 == n1+1) {
    tbuff[0] = receiver_values[0];
    for (int j = 0; j < n1; j++) {
      sum1 += tbuff[j];
    }
    filtered_t = sum1/n1;
  }
  
  else if (counter_1 > n1+1) {
    for (int k = n1-1; k > 0; k--) {
      tbuff[k] = tbuff[k-1];
    }
    tbuff[0] = receiver_values[0];
  for (int l = 0; l < n1; l++) {
      sum1 += tbuff[l];
      }
    filtered_t = sum1/n1;   
  }

  counter_1++;
  return filtered_t;
}

int MAFR() {
  sum2 = 0;
//  for (int i = 0; i<n; i++) {
//    Serial.print(rbuff[i]);
//    Serial.print(", ");
//    if (i==n-1) {
//      Serial.println(rbuff[i]);
//      }
//  }
  if (counter_2 < n2) {
  rbuff[counter_2-1] = receiver_values[1];
    filtered_r = receiver_values[1];
  }
  
  else if (counter_2 == n2) {
    for (int i = 0; i < n2; i++) {
      sum2 += rbuff[i];
    }
    filtered_r = sum2/n2;
  }
  
  else if (counter_2 == n2+1) {
    rbuff[0] = receiver_values[1];
    for (int j = 0; j < n2; j++) {
      sum2 += rbuff[j];
    }
    filtered_r = sum2/n2;
  }
  
  else if (counter_2 > n2+1) {
    for (int k = n2-1; k > 0; k--) {
      rbuff[k] = rbuff[k-1];
    }
    rbuff[0] = receiver_values[1];
  for (int l = 0; l < n2; l++) {
      sum2 += rbuff[l];
      }
    filtered_r = sum2/n2;   
  }

  counter_2++;
  return filtered_r;
}

int MAFP() {
  sum3 = 0;
//  for (int i = 0; i<n; i++) {
//    Serial.print(tbuff[i]);
//    Serial.print(", ");
//    if (i==n-1) {
//      Serial.println(tbuff[i]);
//      }
//  }
  if (counter_3 < n3) {
  pbuff[counter_3-1] = receiver_values[2];
    filtered_p = receiver_values[2];
  }
  
  else if (counter_3 == n3) {
    for (int i = 0; i < n3; i++) {
      sum3 += pbuff[i];
    }
    filtered_p = sum3/n3;
  }
  
  else if (counter_3 == n3+1) {
    pbuff[0] = receiver_values[2];
    for (int j = 0; j < n3; j++) {
      sum3 += pbuff[j];
    }
    filtered_p = sum3/n3;
  }
  
  else if (counter_3 > n3+1) {
    for (int k = n3-1; k > 0; k--) {
      pbuff[k] = pbuff[k-1];
    }
    pbuff[0] = receiver_values[2];
  for (int l = 0; l < n3; l++) {
      sum3 += pbuff[l];
      }
    filtered_p = sum3/n3;   
  }

  counter_3++;
  return filtered_p;
}

int MAFY() {
  sum4 = 0;
//  for (int i = 0; i<n; i++) {
//    Serial.print(tbuff[i]);
//    Serial.print(", ");
//    if (i==n-1) {
//      Serial.println(tbuff[i]);
//      }
//  }
  if (counter_4 < n4) {
  ybuff[counter_4-1] = receiver_values[3];
    filtered_yaw = receiver_values[3];
  }
  
  else if (counter_4 == n4) {
    for (int i = 0; i < n4; i++) {
      sum4 += ybuff[i];
    }
    filtered_yaw = sum4/n4;
  }
  
  else if (counter_4 == n4+1) {
    ybuff[0] = receiver_values[3];
    for (int j = 0; j < n4; j++) {
      sum4 += ybuff[j];
    }
    filtered_yaw = sum4/n4;
  }
  
  else if (counter_4 > n4+1) {
    for (int k = n4-1; k > 0; k--) {
      ybuff[k] = ybuff[k-1];
    }
    ybuff[0] = receiver_values[3];
  for (int l = 0; l < n4; l++) {
      sum4 += ybuff[l];
      }
    filtered_yaw = sum4/n4;   
  }

  counter_4++;
  return filtered_yaw;
}
  
void getAltitudeP() {
  
    MS5611.read();
    MS5611.setOversampling(OSR_ULTRA_HIGH);
    P = MS5611.getPressure();
    T = MS5611.getTemperature() + 273.15;
    z_p = ((8.31432*T)/(9.80665*0.0289644))*log(1013.25/P);
//    Serial.println(z_p);
  }


//void logging() {
//
//  if (myFile) {
//    t = millis();
//    myFile.print("TIME:");
//    myFile.print((t-t_start)/1000);
//    myFile.print(", ");
//    myFile.print("ROLL:");
//    myFile.print(kal_roll);
//    myFile.print(", ");
//    myFile.print("PITCH:");
//    myFile.print(kal_pitch);
////    myFile.print(", ");
////    myFile.print("YAW:");
////    myFile.print(yaw);
//    myFile.print(", ");    
//    myFile.print(m1);
//    myFile.print(", ");
//    myFile.print(m2);
//    myFile.print(", ");
//    myFile.print(m3);
//    myFile.print(", ");
//    myFile.print(m4);
//    myFile.print(", ");
//    myFile.print(control_r_s);
//    myFile.print(", ");
//    myFile.println(control_p_s);    
//    myFile.close();
//  }
//}
