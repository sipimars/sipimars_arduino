#include <Servo.h> 
// these define which servo each joint is connected to
#define BASE       2
#define SHOULDER   3
#define ELBOW      4
#define WRIST      5
#define WRIST_ROT  6
#define GRIPPER    7
// these are the servo objects
Servo base;  
Servo shoulder;  
Servo elbow;  
Servo wrist;  
Servo wrist_rot;  
Servo gripper;  
 
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    } 
  }
}

 
void setup() 
{ 
  // set up the servos
  base.attach(BASE);  
  shoulder.attach(SHOULDER); 
  elbow.attach(ELBOW);  
  wrist.attach(WRIST); 
  wrist_rot.attach(WRIST_ROT); 
  gripper.attach(GRIPPER);  
  
  // send them all to middle position
  base.write(90);
  shoulder.write(90);
  elbow.write(90);
  wrist.write(90);
  wrist_rot.write(90);
  gripper.write(90);

  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  // print instructions
  Serial.println("Lynxmotion Arm controller");
  Serial.println("Enter commands to move a joint to an angle:");
  Serial.println("  W=wrist, R=wrist Rotation, G=gripper,");
  Serial.println("  E=elbow, S=shoulder, B=base");
  Serial.println("  Example:  E20 moves Elbow to angle 20 degrees");
}

void loop() 
{
  int val;
  if (stringComplete) {
    val = inputString.substring(1).toInt();
    if(val >= 20 && val <= 160) {
      switch(inputString[0]) {
      case 'G':  // gripper
        gripper.write(val);
        break;
      case 'W':  // wrist
        wrist.write(val);
        break;
      case 'R':  // wrist rotation
        wrist_rot.write(val);
        break;
      case 'E':  // Elbow
        elbow.write(val);
        break;
      case 'S':  // shoulder
        shoulder.write(val);
        break;
      case 'B':  // base
        base.write(val);
        break;
      } 
    } else {
        Serial.println("Value out of range!");
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
  delay(20);
} 


