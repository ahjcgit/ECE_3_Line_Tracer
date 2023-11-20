#include <ECE3.h>
#include <stdio.h>

const int left_nslp_pin=31; // nslp HIGH ==> awake & ready for PWM
const int right_nslp_pin=11; // nslp HIGH ==> awake & ready for PWM
const int left_dir_pin=29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;

const int LED_RF = 75;

//int oldleftSpd = 30;
//int oldrightSpd = 30;
int baseSpd = 30;
int leftSpd = 30;
int rightSpd = 30;
//int wheelSpd = 80;
int distance = 300;

int blackbar = 0;

uint16_t sensorValues[8];

float sensorWhite[8] = {611, 635, 651, 659, 594.7, 584.4, 589, 672.71};
float sensorBlack[8] = {1684.25, 1765, 1789, 1686.33, 994.3, 1811.1, 1811, 1592.036};

int16_t weights[8] = {-15/8, -14/8, -12/8, -8/8, 8/8, 12/8, 14/8, 15/8};

float sensorNormal[8];

volatile float error;
volatile float preverror;
volatile float barcheck;
float kp = 0.024; //proportional
float kd = 0.01; //derivative

/*
 * -23000, 25000
 * 
 * - means we should turn right
 */

void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission

  ECE3_Init();

  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

//  pinMode(13,INPUT);

  pinMode(LED_RF, OUTPUT);

  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_nslp_pin,HIGH);
//  digitalWrite(left_nslp_pin,LOW);
//  digitalWrite(right_nslp_pin,LOW);

  resetEncoderCount_left();
  resetEncoderCount_right();
  
  delay(2000);

}


void loop()
{
  // read raw sensor values
  ECE3_read_IR(sensorValues);

  error = 0;

  barcheck = 0;

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance
  for (unsigned char i = 0; i < 8; i++)
  {
    sensorNormal[i] = (sensorValues[i]-sensorWhite[i])*1000/sensorBlack[i];

    barcheck += sensorNormal[i];

    sensorNormal[i] = sensorNormal[i]*weights[i];

    error += sensorNormal[i];
  }

  float pid = kp*error + kd*(error-preverror);
  preverror = error;

  if(barcheck > 8500){
    leftSpd = 0;
    rightSpd = 0;

  analogWrite(left_pwm_pin,0); // Maintain speed 
  analogWrite(right_pwm_pin,0);

    delay(5000);

    blackbar += 1;
  }
  //Serial.print(error);
  
  //Serial.println();

  if(blackbar < 2){
    leftSpd = baseSpd - pid;
    rightSpd = baseSpd + pid;
  
    if(leftSpd <= 0){
      leftSpd = 0;
    }
    if(leftSpd >= 60){
      leftSpd = 60;
    }
  
    if(rightSpd <= 0){
      rightSpd = 0;
    }
    if(rightSpd >= 60){
      rightSpd = 60;
    }
  }

  blackbar = 0;

  digitalWrite(left_dir_pin,LOW);  // Set car direction to forward
  digitalWrite(right_dir_pin,LOW);
  
//  ChangeWheelSpeeds(oldleftSpd, leftSpd, oldrightSpd, rightSpd); // Stat car up
  analogWrite(left_pwm_pin,leftSpd); // Maintain speed 
  analogWrite(right_pwm_pin,rightSpd);
  
  //oldleftSpd = leftSpd;
  //oldrightSpd = rightSpd;
  
  resetEncoderCount_left();
  resetEncoderCount_right();
}





int average()  //average pulse count
{
  int getL=getEncoderCount_left();
  int getR=getEncoderCount_right();
//  Serial.print(getL);Serial.print("\t");Serial.println(getR);
  return ((getEncoderCount_left() + getEncoderCount_right())/2);
}
