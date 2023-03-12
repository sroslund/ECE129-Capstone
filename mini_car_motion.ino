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
 

#define FRONT 170       // steering to front 
int SHARP_RIGHT=FRONT+33;
int SHARP_LEFT=FRONT-40;
int  RIGHT=FRONT+16;
int  LEFT=FRONT-20;
  
#define DELAY_TIME 1000   
  
#define LFSensor_0 A0  //OLD D3
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 A4  //OLD D10
 

#define SPEED 150
#define FAST_SPEED 200 
#define MID_SPEED 180
#define SERVO_PIN    9  //servo connect to D3
 
PWMServo head;
/*motor control*/
void go_Back(int speed)  //Forward
{

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2,LOW);
   analogWrite(ENA,speed);
}
 
void go_Advance(int speed)  //Forward
{

  digitalWrite(IN1, LOW);
  digitalWrite(IN2,HIGH);
 

   analogWrite(ENA,speed);
}
 
void turn(int angle)
{

  head.write(angle);

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
  turn(FRONT);
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
  char buffer[16];
  if (Serial.available() > 0){
    int size = Serial.readBytesUntil('\n', buffer, 12);
    if (buffer[0] == 'r'){
      turn(RIGHT);
    }
    if (buffer[0] == 'l'){
      turn(LEFT);
    }
    if (buffer[0] == 'R'){
      turn(SHARP_RIGHT);
    }
    if (buffer[0] == 'L'){
      turn(SHARP_LEFT);
    }
    if (buffer[0] == 'f'){
      turn(FRONT);
    }
  }


}
