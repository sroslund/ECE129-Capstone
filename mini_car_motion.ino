/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/ 
 * OSOYOO SG90 steering car Lesson 3 Line tracking
 * Tutorial URL https://osoyoo.com/?p=37188
 * CopyRight www.osoyoo.com

 * This project will show you how to make Osoyoo robot car in auto drive mode and avoid obstacles
 *   
 * 
 */
#include <PWMServo.h>

 
#define IN1 7
#define IN2 8
#define ENA 5       //  ENA pin
#define MESSAGE_LENGTH 16

#define FRONT 130 // MAX 180,       // steering to front 
int SHARP_RIGHT=FRONT+50; // max +50
int SHARP_LEFT=FRONT-45; // max -45
int  RIGHT=FRONT+10;
int  LEFT=FRONT-10;

#define DELAY_TIME 1000   
  
#define LFSensor_0 A0  //OLD D3
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 A4  //OLD D10
 

#define SLOW_SPEED 150 // at 150 does not start moving on its own
#define FAST_SPEED 250 // maximum speed
#define MID_SPEED 200
#define SERVO_PIN    9  //servo connect to D3
 
PWMServo head;
/*motor control*/
void go_Back(int speed)  //Forward
{

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2,LOW);
   analogWrite(ENA,speed);
}
 
void go_Advance(int throttle_percentage)
{

  digitalWrite(IN1, LOW);
  digitalWrite(IN2,HIGH);
  unsigned int pwm = throttle_percentage + 150;
  if(pwm > FAST_SPEED) {
    pwm = FAST_SPEED;    
  }
  if(pwm < SLOW_SPEED && pwm != 150) {
    pwm = SLOW_SPEED;
  }
  if(pwm == 150) {
    pwm = 0;
  }
  analogWrite(ENA,pwm);
}
 
void turn(int steering_angle)
{
  unsigned int adjusted_angle = steering_angle + 130;
  if (adjusted_angle > 180) {
    adjusted_angle = 180;
  }
  if (adjusted_angle < 85) {
    adjusted_angle = 85;
  }
  head.write(adjusted_angle);

}
 
void stop_Stop()    //Stop
{

  digitalWrite(IN1, LOW);
  digitalWrite(IN2,LOW);

  analogWrite(ENA,0);
}

  
void setup() {

  pinMode(ENA, OUTPUT); 
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 


  head.attach(SERVO_PIN);
  turn(0);
  stop_Stop();
  Serial.begin(9600);
  while (!Serial){
  ;
}
 
}

 
void loop() {
  auto_tracking();
}

void auto_tracking(){
  while(Serial.available()>0) {
    static char buffer[MESSAGE_LENGTH];
    static unsigned int position = 0;
    char inByte = Serial.read();    

    if(inByte != '\n') {  // && (position < MESSAGE_LENGTH-1)
      buffer[position] = inByte;
      position++;
    } else {
      buffer[position] = '\0';
      Serial.println(buffer);

      int steering = atoi(buffer);
      Serial.println(steering);      
      turn(steering);
      unsigned int throttle_percentage = buffer[position-2]*10-480+buffer[position-1]-48;
      Serial.println(throttle_percentage); 
      go_Advance(throttle_percentage);      
      position = 0;
    }
  }
}




