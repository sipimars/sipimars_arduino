
#include <ros.h>
#include <geometry_msgs/Point.h>
#include <Servo.h> 
 
Servo grip;  // create servo object to control a servo 
Servo wrist;  // create servo object to control a servo 

ros::NodeHandle  nh;

int dir[] = {12,13};
int pwm[] = {3, 11};
int brake[] = {9, 8};
int cur[] = {0,1};
int DIR_FWD[] = {LOW,HIGH};
int DIR_REV[] = {HIGH,LOW};
int loopCount = 0;

#define WRIST_PIN 5
#define GRIP_PIN  4

void stopMotors(void) 
{
    for(int i=0;i<2;++i) {
      digitalWrite(brake[i], HIGH);
      analogWrite(pwm[i], 0);
    }
}

void motorMessageCb( const geometry_msgs::Point& cmd_msg){
  if( (cmd_msg.x == 0) && (cmd_msg.y == 0) ) {
    stopMotors();
    return;
  }
  loopCount = cmd_msg.z;
  int efforts[2];
  efforts[0] = cmd_msg.x;
  efforts[1] = cmd_msg.y;
  for(int i=0;i<2;++i) {
    digitalWrite(brake[i], LOW);
    if(efforts[i] < 0) {
      digitalWrite(dir[i], DIR_REV[i]);
      analogWrite(pwm[i], -efforts[i]);
    } else {
      digitalWrite(dir[i], DIR_FWD[i]);
      analogWrite(pwm[i], efforts[i]);
    }
  }
  return;
}

void gripMessageCb( const geometry_msgs::Point& cmd_msg){
  grip.write(cmd_msg.x);
  wrist.write(cmd_msg.y);
}

ros::Subscriber<geometry_msgs::Point> motorSubscriber("motor_cmd", motorMessageCb );
ros::Subscriber<geometry_msgs::Point> gripSubscriber("grip_cmd", gripMessageCb );

void setup() 
{ 
  grip.attach(GRIP_PIN);  // attaches the servo on pin 9 to the servo object 
  grip.write(90);
  wrist.attach(WRIST_PIN);  // attaches the servo on pin 9 to the servo object 
  wrist.write(120);

  for(int i=0;i<2;++i) {
    pinMode(dir[i], OUTPUT);
    pinMode(pwm[i], OUTPUT);
    analogWrite(pwm[i], 0);
    pinMode(brake[i], OUTPUT);
  }
  nh.initNode();
  nh.subscribe(motorSubscriber);
  nh.subscribe(gripSubscriber);
} 
void loop() 
{
  nh.spinOnce();
  delay(50);
  if(loopCount >0) {
    loopCount--;
  } else {
    stopMotors();
  }
} 
