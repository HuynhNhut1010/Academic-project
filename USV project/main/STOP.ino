/*
 * Motor Control Functions
 */

// Function to stop the motors
void STOP() {
  state = 0;                                                                 // Update state to indicate STOP
  analogWrite(pinMotorL, 70);                                                // Set left motor to minimal movement
  analogWrite(pinMotorR, 10);                                                // Set right motor to minimal movement
  Serial.println("MOTOR L AND R : 10 and 70");                               // Log the motor speeds
  Serial.println("*************************STOP*************************");  // Indicate the stop action
}

// Function to move forward with a specified speed
void go_ahead(int speed_set) {
  pcf8574.digitalWrite(P4, 1);             // Enable left motor forward
  pcf8574.digitalWrite(P5, 1);             // Enable left motor forward
  pcf8574.digitalWrite(P6, 1);             // Enable right motor forward
  pcf8574.digitalWrite(P7, 1);             // Enable right motor forward
  control_servo(180);                      // Align servo to go straight
  analogWrite(pinMotorR, speed_set);       // Set right motor speed
  analogWrite(pinMotorL, speed_set + 60);  // Set left motor speed with offset
  Serial.println("Go ahead");              // Log the action
}

// Function to turn right at a specific angle
void turn_right(int angle) {
  control_servo(angle);                              // Adjust servo to the specified angle (e.g., 5 degrees for right turn)
  analogWrite(pinMotorL, 73);                        // Increase left motor speed for the turn
  analogWrite(pinMotorR, 13);                        // Decrease right motor speed for the turn
  pcf_MUI.digitalWrite(P0, 0);                       // Disable additional motor control if needed
  pcf_MUI.digitalWrite(P1, 0);                       // Disable additional motor control
  analogWrite(pinMotorT, speed_motorT.toInt() + 3);  // Adjust motor T speed for support
  Serial.println("Turn right");                      // Log the action
}

// Function to turn left at a specific angle
void turn_left(int angle) {
  control_servo(angle);                              // Adjust servo to the specified angle (e.g., 270 degrees for left turn)
  analogWrite(pinMotorL, 73);                        // Increase left motor speed for the turn
  analogWrite(pinMotorR, 13);                        // Decrease right motor speed for the turn
  pcf_MUI.digitalWrite(P0, 1);                       // Enable additional motor control if needed
  pcf_MUI.digitalWrite(P1, 1);                       // Enable additional motor control
  analogWrite(pinMotorT, speed_motorT.toInt() + 3);  // Adjust motor T speed for support
  Serial.println("Turn left");                       // Log the action
}

// Function to move backward
void backward(int speed_set) {
  analogWrite(pinMotorL, 70);  // Set left motor to reverse speed
  analogWrite(pinMotorR, 10);  // Set right motor to reverse speed
  delay(200);                  // Short delay for the backward action
  control_servo(180);          // Align servo to straight
  //analogWrite(pinMotorL, speed_set);  // Uncomment if left motor speed needs to be adjusted dynamically
  Serial.println("backward");  // Log the action
}
