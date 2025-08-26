// Constants
#define MOTOR_PIN 3 // Pin connected to L298N motor driver
#define SERVO_PIN 5 // Pin connected to servo motor
#define FRONT_SENSOR_PIN A0 // Pin connected to front ultrasonic sensor
#define LEFT_SENSOR_PIN A1 // Pin connected to left ultrasonic sensor
#define RIGHT_SENSOR_PIN A2 // Pin connected to right ultrasonic sensor
#define REAR_SENSOR_PIN A3 // Pin connected to rear ultrasonic sensor
#define PUSH_BUTTON_PIN 2 // Pin connected to push button
#define MAX_SPEED 255 // Maximum speed for DC motor
#define MIN_SPEED 0 // Minimum speed for DC motor
#define MAX_SERVO_ANGLE 180 // Maximum angle for servo motor
#define MIN_SERVO_ANGLE 0 // Minimum angle for servo motor
#define FRONT_DISTANCE_THRESHOLD 18 // Distance threshold for front sensor
#define SIDE_DISTANCE_THRESHOLD 30 // Distance threshold for side sensors
#define REVERSE_TIME 500 // Time in milliseconds for reverse movement

// Variables
int motorSpeed = 0; // Current speed of DC motor
int servoAngle = 90; // Current angle of servo motor
int frontDistance = 0; // Distance measured by front sensor
int leftDistance = 0; // Distance measured by left sensor
int rightDistance = 0; // Distance measured by right sensor
int rearDistance = 0; // Distance measured by rear sensor
bool isRunning = false; // Flag to indicate if car is running
bool isReversing = false; // Flag to indicate if car is reversing

// Setup function
void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Set pin modes
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(FRONT_SENSOR_PIN, INPUT);
  pinMode(LEFT_SENSOR_PIN, INPUT);
  pinMode(RIGHT_SENSOR_PIN, INPUT);
  pinMode(REAR_SENSOR_PIN, INPUT);
  pinMode(PUSH_BUTTON_PIN, INPUT_PULLUP);
  
  // Set initial servo angle
  servoAngle = map(servoAngle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE, 0, 180);
  servoWrite(SERVO_PIN, servoAngle);
}

// Main loop
void loop() {
  // Check if push button is pressed to start the run
  if (digitalRead(PUSH_BUTTON_PIN) == LOW && !isRunning) {
    isRunning = true;
    Serial.println("Starting run...");
  }
  
  // Check if car is running
  if (isRunning) {
    // Read sensor values
    frontDistance = readUltrasonicSensor(FRONT_SENSOR_PIN);
    leftDistance = readUltrasonicSensor(LEFT_SENSOR_PIN);
    rightDistance = readUltrasonicSensor(RIGHT_SENSOR_PIN);
    rearDistance = readUltrasonicSensor(REAR_SENSOR_PIN);
    
    // Check if front sensor detects an obstacle closer than the threshold
    if (frontDistance < FRONT_DISTANCE_THRESHOLD) {
      // Stop the car
      motorSpeed = MIN_SPEED;
      analogWrite(MOTOR_PIN, motorSpeed);
      
      // Decide the best turn direction based on side distances
      if (leftDistance > rightDistance) {
        // Turn left
        servoAngle = map(90, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE, 0, 180);
        servoWrite(SERVO_PIN, servoAngle);
      } else {
        // Turn right
        servoAngle = map(90, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE, 180, 0);
        servoWrite(SERVO_PIN, servoAngle);
      }
      
      // Wait for a brief moment before continuing
      delay(500);
    } else {
      // Check if both sides are blocked
      if (leftDistance < SIDE_DISTANCE_THRESHOLD && rightDistance < SIDE_DISTANCE_THRESHOLD) {
        // Reverse briefly if rear is clear
        if (rearDistance > SIDE_DISTANCE_THRESHOLD) {
          isReversing = true;
          motorSpeed = -MAX_SPEED;
          analogWrite(MOTOR_PIN, motorSpeed);
          delay(REVERSE_TIME);
          isReversing = false;
        }
        
        // Turn towards the side with more space
        if (leftDistance > rightDistance) {
          // Turn left
          servoAngle = map(90, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE, 0, 180);
          servoWrite(SERVO_PIN, servoAngle);
        } else {
          // Turn right
          servoAngle = map(90, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE, 180, 0);
          servoWrite(SERVO_PIN, servoAngle);
        }
      } else {
        // Gradually accelerate the car
        if (motorSpeed < MAX_SPEED) {
          motorSpeed++;
          analogWrite(MOTOR_PIN, motorSpeed);
        }
