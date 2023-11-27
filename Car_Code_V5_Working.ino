#include <ECE3.h>
#include <stdio.h>

const int left_nslp_pin=31; // nslp HIGH ==> awake & ready for PWM
const int right_nslp_pin=11; // nslp HIGH ==> awake & ready for PWM
const int left_dir_pin=29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;

const int LED_RF = 75;

const int ir1 = 65; // left side of car
const int ir2 = 48;
const int ir3 = 64;
const int ir4 = 47;
const int ir5 = 52;
const int ir6 = 68;
const int ir7 = 53;
const int ir8 = 69; // right side of car

//int oldleftSpd = 30;
//int oldrightSpd = 30;

//changed spd to 35
int baseSpd = 35;
int leftSpd = 45;
int rightSpd = 45;
//int wheelSpd = 80;
int distance = 300;

//int blackbar = 0;

uint16_t sensorValues[8];

float sensorWhite[8] = {611, 635, 651, 659, 594.7, 584.4, 589, 672.71};
float sensorBlack[8] = {1684.25, 1765, 1789, 1686.33, 994.3, 1811.1, 1811, 1692.036};

float weights[8] = {-15/8, -14/8, -12/8, -8/8, 8/8, 12/8, 14/8, 15/8};

float sensorNormal[8];

volatile float error;
volatile float preverror;
//volatile float barcheck;
float kp = 0.048; //proportional
  float kd = 0.045 ; //derivative

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
  
  delay(500);

}
boolean runLoop = true;
void loop()
{
  if(runLoop)
  {
      // read raw sensor values
  ECE3_read_IR(sensorValues);
  int barCounter = 0;
  for (int i = 0 ; i < 8; i ++)
  {
    if (sensorValues[i] > 2400)
    {
      barCounter++;
    }
  }

  if (barCounter == 8 && average() > 3000 && average() < 3600)
  {
    analogWrite(left_pwm_pin,0 ); // stop car
    analogWrite(right_pwm_pin,0);
    delay(1000);
    while (average() > 3000 && average() < 3500)
    {
          digitalWrite(left_dir_pin,LOW);
          analogWrite(left_pwm_pin,123); // Maintain speed 
          digitalWrite(right_dir_pin,HIGH);
          analogWrite(right_pwm_pin,123);
          
    }
    
  }

  if (average()>500 && average() <1200 || average() > 1600 && average() < 2050 || average() > 4400 && average() < 4750 || average() > 5600 && average() < 6250 )
  {
    digitalWrite(LED_RF, HIGH);
    sensorValues[0] = 0;
    sensorValues[7] = 0;
    //Serial.println("Counter set");
  }
  digitalWrite(LED_RF, LOW);
  error = 0;
  
  //barcheck = 0;

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance
  for (unsigned char i = 0; i < 8; i++)
  {
    sensorNormal[i] = (sensorValues[i]-sensorWhite[i])*1000/sensorBlack[i];

    //barcheck += sensorNormal[i];

    sensorNormal[i] = sensorNormal[i]*weights[i];

    error += sensorNormal[i];
  }

  //check if both outer sensors are being activated
//  Serial.print("sensor 0:");
//  Serial.println(sensorValues[0]);
//  Serial.print("Sensor 7:");
//  Serial.println(sensorValues[7]);


//
//  int sensorCount = read1+read2+read3+read4+read5+read6+read7+read8;
  float pid = kp*error + kd*(error-preverror);
  preverror = error;

//  if(barcheck > 8500){
//    leftSpd = 0;
//    rightSpd = 0;
//
//    analogWrite(left_pwm_pin,0); //Set speed to 0
//    analogWrite(right_pwm_pin,0);
//
//    delay(5000);
//
//    blackbar += 1;
//  }

  //Serial.print(error);
  
  //Serial.println();

  //if(blackbar < 2){
    leftSpd = baseSpd - pid;
    rightSpd = baseSpd + pid;
  
    if(leftSpd <= 0){
      leftSpd = 0;
    }
    if(leftSpd >= 70){
      leftSpd = 70;
    }
  
    if(rightSpd <= 0){
      rightSpd = 0;
    }
    if(rightSpd >= 70){
      rightSpd = 70;
    }

  //}

  //blackbar = 0;

  digitalWrite(left_dir_pin,LOW);  // Set car direction to forward
  digitalWrite(right_dir_pin,LOW);
  
  analogWrite(left_pwm_pin,leftSpd); // Maintain speed 
  analogWrite(right_pwm_pin,rightSpd);
      if (average()>6000 && barCounter == 8 )
  {
    digitalWrite(left_dir_pin,LOW);  // Set car direction to forward
    digitalWrite(right_dir_pin,LOW);
    analogWrite(left_pwm_pin,0 ); // stop car
    analogWrite(right_pwm_pin,0);
    runLoop = false;
  }
  }

}

int average()  //average pulse count
{
  int getL=getEncoderCount_left();
  int getR=getEncoderCount_right();
  //Serial.print(getL);Serial.print("\t");Serial.println(getR);
  return ((getEncoderCount_left() + getEncoderCount_right())/2);
}
