#include <ECE3.h>
#include <stdio.h>

const int left_nslp_pin=31; // nslp HIGH ==> awake & ready for PWM
const int right_nslp_pin=11; // nslp HIGH ==> awake & ready for PWM
const int left_dir_pin=29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;

const int LED_RF = 75;

int oldleftSpd = 30;
int oldrightSpd = 30;
int baseSpd = 30;
int leftSpd = 30;
int rightSpd = 30;
//int wheelSpd = 80;
int distance = 300;

uint16_t sensorValues[8];

float sensorWhite[8] = {661, 685, 651, 709, 594.7, 584.4, 589, 672.71};
float sensorBlack[8] = {1684.25, 1765, 1789, 1686.33, 994.3, 1811.1, 1811, 1592.036};

int16_t weights[8] = {-15, -14, -12, -8, 8, 12, 14, 15}; 

float sensorNormal[8];

volatile float error;

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

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance
  for (unsigned char i = 0; i < 8; i++)
  {
    sensorNormal[i] = (sensorValues[i]-sensorWhite[i])*1000/sensorBlack[i];

    sensorNormal[i] = sensorNormal[i]*weights[i];
    
    //Serial.print(sensorNormal[i]);
    //Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor

    error += sensorNormal[i];
  }

  error = error/2000;

  //Serial.print(error);
  
  //Serial.println();

  //if(error >= 8){ error = 8; }
  //if(error <= -8){error = -8;}

  leftSpd = baseSpd - error;
  rightSpd = baseSpd + error;

  if(leftSpd <= 20){
    leftSpd = 20;
  }
  if(leftSpd >= 50){
    leftSpd = 50;
  }

  if(rightSpd <= 20){
    rightSpd = 20;
  }
  if(rightSpd >= 50){
    rightSpd = 50;
  }

  digitalWrite(left_dir_pin,LOW);  // Set car direction to forward
  digitalWrite(right_dir_pin,LOW);
  
  ChangeWheelSpeeds(oldleftSpd, leftSpd, oldrightSpd, rightSpd); // Stat car up
  analogWrite(left_pwm_pin,leftSpd); // Maintain speed 
  analogWrite(right_pwm_pin,rightSpd);
  
  oldleftSpd = leftSpd;
  oldrightSpd = rightSpd;
  
  /*while(average() < distance); // Maintain speed 
  ChangeWheelSpeeds(wheelSpd, 0, wheelSpd, 0); //Stop car

//  Change direction to go backwards
  digitalWrite(left_dir_pin,HIGH); // Set car direction to  backward
  digitalWrite(right_dir_pin,HIGH);
  
  ChangeWheelSpeeds(0, wheelSpd, 0, wheelSpd); // Stat car up 
  analogWrite(left_pwm_pin,wheelSpd); // Maintain speed 
  analogWrite(right_pwm_pin,wheelSpd);
  resetEncoderCount_left();
  resetEncoderCount_right();  
  while(average() < distance) ;
  ChangeWheelSpeeds(wheelSpd, 0, wheelSpd, 0); //Stop car
  */
  
  resetEncoderCount_left();
  resetEncoderCount_right();

  delay(30);
}


void  ChangeWheelSpeeds(int initialLeftSpd, int finalLeftSpd, int initialRightSpd, int finalRightSpd) {
/*  
 *   This function changes the car speed gradually (in about 30 ms) from initial
 *   speed to final speed. This non-instantaneous speed change reduces the load 
 *   on the plastic geartrain, and reduces the failure rate of the motors. 
 */
  int diffLeft  = finalLeftSpd-initialLeftSpd;
  int diffRight = finalRightSpd-initialRightSpd;
  int stepIncrement = 20;
  int numStepsLeft  = abs(diffLeft)/stepIncrement;
  int numStepsRight = abs(diffRight)/stepIncrement;
  int numSteps = max(numStepsLeft,numStepsRight);
  
  int pwmLeftVal = initialLeftSpd;        // initialize left wheel speed 
  int pwmRightVal = initialRightSpd;      // initialize right wheel speed 
  int deltaLeft = (diffLeft)/numSteps;    // left in(de)crement
  int deltaRight = (diffRight)/numSteps;  // right in(de)crement

  for(int k=0;k<numSteps;k++) {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(left_pwm_pin,pwmLeftVal);    
    analogWrite(right_pwm_pin,pwmRightVal); 
    delay(30);   
  } // end for int k
//  if(finalLeftSpd  == 0) analogWrite(left_pwm_pin,0); ;
//  if(finalRightSpd == 0) analogWrite(right_pwm_pin,0);
  analogWrite(left_pwm_pin,finalLeftSpd);  
  analogWrite(right_pwm_pin,finalRightSpd);  
} // end void  ChangeWheelSpeeds

//void ChangeBaseSpeed(int initialBaseSpd, int finalBaseSpd) {
//  /*  
// *   This function changes the car base speed gradually (in about 300 ms) from
// *   initialBaseSpeed to finalBaseSpeed. This non-instantaneous speed change
// *   reduces the load on the plastic geartrain, and reduces the failure rate of 
// *   the motors. 
// */
//  int speedChangeTime = 300; // milliseconds
//  int numSteps = 5;
//  int pwmVal = initialBaseSpd; // initialize left wheel speed 
//  int deltaSpeed = (finalBaseSpd-initialBaseSpd)/numSteps; // in(de)crement
//  for(int k=0;k<numSteps;k++) {
////    pwmVal = pwmLeftVal + deltaSpeed;
////    analogWrite(left_pwm_pin,pwmVal);    
////    analogWrite(right_pwm_pin,pwmVal); 
////    delay(60);   
//  } // end for int k
//} // end void ChangeBaseSpeed





int average()  //average pulse count
{
  int getL=getEncoderCount_left();
  int getR=getEncoderCount_right();
//  Serial.print(getL);Serial.print("\t");Serial.println(getR);
  return ((getEncoderCount_left() + getEncoderCount_right())/2);
}
