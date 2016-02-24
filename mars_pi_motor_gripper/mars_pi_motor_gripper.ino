/******************************************************
 * Arduino code for the roadrunner with a motor shield for
 * gearmotors and a servo powered gripped with a wrist joint
 * ****************************************************/
#include <ros.h>
#include <geometry_msgs/Point.h>
#include <Servo.h> 
 
// create servo objects to controls servos 
Servo grip;  // open and close the gripper
Servo wrist; // move wrist up and down

// handle for the ros node interface
ros::NodeHandle  nh;

// map the pins to the motor shield functions
int dir[] = {12,13};
int pwm[] = {3, 11};
int brake[] = {9, 8};
int cur[] = {0,1};
// set the directions for the left and right wheels
int DIR_FWD[] = {LOW,HIGH};
int DIR_REV[] = {HIGH,LOW};
// loopCount is used as a timer to allow commands to act over a limited time
// this is necessary in case communication is lost, the motors must 
// automatically stop after some time
int loopCount = 0;

// map the servo motor pins
#define WRIST_PIN 5
#define GRIP_PIN  4

void stopMotors(void) 
{
    for(int i=0;i<2;++i) {
      digitalWrite(brake[i], HIGH);
      analogWrite(pwm[i], 0);
    }
}

// callback function when a motor command ROS message is received
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

// callback function when a gripper command ROS message is received
void gripMessageCb( const geometry_msgs::Point& cmd_msg){
  // write the values directly to the servos.
  // TODO add some limit checking or value mapping
  grip.write(cmd_msg.x);
  wrist.write(cmd_msg.y);
}

// create the ros topic subscribers
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
