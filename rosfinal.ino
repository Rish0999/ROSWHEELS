/*
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: ROS node that publishes the accumulated ticks for each wheel
 * (/right_ticks and /left_ticks topics) at regular intervals using the 
 * built-in encoder (forward = positive; reverse = negative). 
 * The node also subscribes to linear & angular velocity commands published on 
 * the /cmd_vel topic to drive the robot accordingly.
 * Reference: Practical Robotics in C++ book (ISBN-10 : 9389423465)
 */
// Handles startup and shutdown of ROS

////////////////// Tick Data Publishing Variables and Constants ///////////////


 
// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
// Range of 65,535
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Time interval for measurements in milliseconds
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;
 
////////////////// Motor Controller Variables and Constants ///////////////////
int pwm1=3;
int pwm2=5;
int pwm3=6;
int pwm4=9;
int dir1=2;
int dir2=4;
int dir3=7;
int dir4=8;

// How much the PWM value can change each cycle
const int PWM_INCREMENT = 1;
 
// Number of ticks per wheel revolution. We won't use this in this code.
const int TICKS_PER_REVOLUTION = 620;
 
// Wheel radius in meters
const double WHEEL_RADIUS = 0.06;
 
// Distance from center of the left tire to the center of the right tire in m
const double WHEEL_BASE = 0.29;
 
// Number of ticks a wheel makes moving a linear distance of 1 meter
// This value was measured manually.
const double TICKS_PER_METER = 3100; // Originally 2880
 
// Proportional constant, which was measured by measuring the 
// PWM-Linear Velocity relationship for the robot.
const int K_P = 300;
 
// Y-intercept for the PWM-Linear Velocity relationship for the robot
const int b = 52;
 
// Correction multiplier for drift. Chosen through experimentation.
const int DRIFT_MULTIPLIER = 120;
 
// Turning PWM output (0 = min, 255 = max for PWM values)
const int PWM_TURN = 80;
 
// Set maximum and minimum limits for the PWM values
const int PWM_MIN = 80; // about 0.1 m/s
const int PWM_MAX = 100; // about 0.172 m/s
 
// Set linear velocity and PWM variable values for each wheel
double velLeftWheel = 0;
double velRightWheel = 0;
double pwmLeftReq = 0;
double pwmRightReq = 0;
 
// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;
 
/////////////////////// Tick Data Publishing Functions ////////////////////////
 
// Increment the number of ticks

 
// Increment the number of ticks

 
/////////////////////// Motor Controller Functions ////////////////////////////
 
// Calculate the left wheel linear velocity in m/s every time a 
// tick count message is rpublished on the /left_ticks topic. 

 
// Take the velocity command as input and calculate the PWM values.
// void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
   
//   // Record timestamp of last velocity command received
//   lastCmdVelReceived = (millis()/1000);
//    Serial.println(cmdVel.linear.x);
//   // Calculate the PWM value given the desired velocity 
//   pwmLeftReq = K_P * cmdVel.linear.x + b;
//   pwmRightReq = K_P * cmdVel.linear.x + b;
 
//   // Check if we need to turn 
//   if (cmdVel.angular.z != 0.0) {
 
//     // Turn left
//     if (cmdVel.angular.z > 0.0) {
//       pwmLeftReq = -PWM_TURN;
//       pwmRightReq = PWM_TURN;
//     }
//     // Turn right    
//     else {
//       pwmLeftReq = PWM_TURN;
//       pwmRightReq = -PWM_TURN;
//     }
//   }
//   // Go straight
//   else {
     
//     // Remove any differences in wheel velocities 
//     // to make sure the robot goes straight
//     static double prevDiff = 0;
//     static double prevPrevDiff = 0;
//     double currDifference = velLeftWheel - velRightWheel; 
//     double avgDifference = (prevDiff+prevPrevDiff+currDifference)/3;
//     prevPrevDiff = prevDiff;
//     prevDiff = currDifference;
 
//     // Correct PWM values of both wheels to make the vehicle go straight
//     pwmLeftReq -= (int)(avgDifference * DRIFT_MULTIPLIER);
//     pwmRightReq += (int)(avgDifference * DRIFT_MULTIPLIER);
//   }
 
//   // Handle low PWM values
//   if (abs(pwmLeftReq) < PWM_MIN) {
//     pwmLeftReq = 0;
//   }
//   if (abs(pwmRightReq) < PWM_MIN) {
//     pwmRightReq = 0;  
//   }  
// }
 
void set_pwm_values() {
 
  // These variables will hold our desired PWM values
  static int pwmLeftOut = 0;
  static int pwmRightOut = 0;
 
  // If the required PWM is of opposite sign as the output PWM, we want to
  // stop the car before switching direction
  static bool stopped = false;
  if ((pwmLeftReq * velLeftWheel < 0 && pwmLeftOut != 0) ||
      (pwmRightReq * velRightWheel < 0 && pwmRightOut != 0)) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }
 
  // Set the direction of the motors
  if (pwmLeftReq > 0) { // Left wheel forward
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, HIGH);
  }
  else if (pwmLeftReq < 0) { // Left wheel reverse
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, LOW);
  }
  else if (pwmLeftReq == 0 && pwmLeftOut == 0 ) { // Left wheel stop
    digitalWrite(pwm1, LOW);
    digitalWrite(pwm2, LOW);
  }
  else { // Left wheel stop
    digitalWrite(pwm1, LOW);
    digitalWrite(pwm2, LOW); 
  }
 
  if (pwmRightReq > 0) { // Right wheel forward
    digitalWrite(dir3, HIGH);
    digitalWrite(dir4, HIGH);
  }
  else if(pwmRightReq < 0) { // Right wheel reverse
    digitalWrite(dir3, LOW);
    digitalWrite(dir4, LOW);
  }
  else if (pwmRightReq == 0 && pwmRightOut == 0) { // Right wheel stop
    digitalWrite(pwm3, LOW);
    digitalWrite(pwm4, LOW);
  }
  else { // Right wheel stop
    digitalWrite(pwm3, LOW);
    digitalWrite(pwm4, LOW); 
  }
 
  // Increase the required PWM if the robot is not moving
  if (pwmLeftReq != 0 && velLeftWheel == 0) {
    pwmLeftReq *= 1.5;
  }
  if (pwmRightReq != 0 && velRightWheel == 0) {
    pwmRightReq *= 1.5;
  }
 
  // Calculate the output PWM value by making slow changes to the current value
  if (abs(pwmLeftReq) > pwmLeftOut) {
    pwmLeftOut += PWM_INCREMENT;
  }
  else if (abs(pwmLeftReq) < pwmLeftOut) {
    pwmLeftOut -= PWM_INCREMENT;
  }
  else{}
   
  if (abs(pwmRightReq) > pwmRightOut) {
    pwmRightOut += PWM_INCREMENT;
  }
  else if(abs(pwmRightReq) < pwmRightOut) {
    pwmRightOut -= PWM_INCREMENT;
  }
  else{}
 
  // Conditional operator to limit PWM output at the maximum 
  pwmLeftOut = (pwmLeftOut > PWM_MAX) ? PWM_MAX : pwmLeftOut;
  pwmRightOut = (pwmRightOut > PWM_MAX) ? PWM_MAX : pwmRightOut;
 
  // PWM output cannot be less than 0
  pwmLeftOut = (pwmLeftOut < 0) ? 0 : pwmLeftOut;
  pwmRightOut = (pwmRightOut < 0) ? 0 : pwmRightOut;
 
  // Set the PWM value on the pins
  analogWrite(pwm1, pwmLeftOut); 
  analogWrite(pwm2, pwmLeftOut); 
  analogWrite(pwm3, pwmRightOut); 
  analogWrite(pwm4, pwmRightOut); 

  // Serial.println(pwmLeftOut);
  // Serial.println(pwmRightOut);
}
 
// Set up ROS subscriber to the velocity command

void setup() {
  
  Serial.begin(9600);
  // Set pin states of the encoder
  
 
  // Every time the pin goes high, this is a tick
  // Motor control pins are outputs
 
pinMode(pwm1,OUTPUT);
pinMode(pwm2,OUTPUT);
pinMode(pwm3,OUTPUT);
pinMode(pwm4,OUTPUT);
pinMode(dir1,OUTPUT);
pinMode(dir2,OUTPUT);
pinMode(dir3,OUTPUT);
pinMode(dir4,OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(pwm1, LOW);
  digitalWrite(pwm2, LOW);
  digitalWrite(pwm3, LOW);
  digitalWrite(pwm4, LOW);
  
  // Set the motor speed
  analogWrite(pwm1, 0); 
  analogWrite(pwm2, 0);
  analogWrite(pwm3, 0); 
  analogWrite(pwm4, 0);
 
  // ROS Setup
}
 
void loop() {
   
  
    // Record the time
  currentMillis = millis();
  if (Serial.available() > 0) {

    int linear_x = Serial.read();
    pwmLeftReq = K_P * linear_x + b;
  pwmRightReq = K_P * linear_x + b;
  }
  //  pwmLeftReq = K_P * linear_x + b;
  // pwmRightReq = K_P * linear_x + b;
  // if((millis()/1000) - lastCmdVelReceived > 1) {
  //   // Serial.println(500);
  //   pwmLeftReq = 0;
  //   pwmRightReq = 0;
  // }
 
  set_pwm_values();

  
  
}