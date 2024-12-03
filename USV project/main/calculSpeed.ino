/*
 * Adjusts motor speed based on the remaining distance to the next waypoint.
 */
void calculSpeed() {
  // Determine speed based on the distance to the next point
  if (distanceToNextPoint >= 15) {  // If far from the next point
    if (speed_edit.toInt() == 0) {  // If no user-defined speed is set
      speed_set = 15;               // Default speed
    } else {
      speed_set = speed_edit.toInt();  // Use the user-defined speed
    }
  } else if (distanceToNextPoint >= 5 && distanceToNextPoint < 15) {  // If moderately close
    if (speed_edit.toInt() == 0) {
      speed_set = 15;  // Default speed
    } else {
      speed_set = speed_edit.toInt() - 1;  // Slightly decrease user-defined speed
    }
  } else if (distanceToNextPoint < 5) {  // If close to the waypoint
    if (speed_edit.toInt() == 0) {
      speed_set = 13;  // Reduced default speed
    } else {
      speed_set = speed_edit.toInt() - 3;  // Further decrease user-defined speed
    }
  } else if (distanceToNextPoint <= 1) {  // If very close or at the waypoint
    speed_set = 10;                       // Minimal speed (can imply STOP)
  }

  // Apply calculated speed to motors
  analogWrite(pinMotorR, speed_set + 1);   // Right motor speed
  analogWrite(pinMotorL, speed_set + 60);  // Left motor speed with offset
}

/*
 * Controls the motors to manage individual motor speeds and directions.
 */
void controlMotor() {
  // Manage the right motor based on the "tribord" flag
  if (tribord) {
    analogWrite(pinMotorR, speed_set);                     // Apply calculated speed to right motor
    Serial.println("Right motor : " + String(speed_set));  // Log speed
  } else {
    analogWrite(pinMotorR, 10);          // Set minimal speed for the right motor
    Serial.println("Right motor : 10");  // Log default fallback speed
  }
  delay(10);  // Short delay for stability

  // Manage the left motor based on the "babord" flag
  if (babord) {
    analogWrite(pinMotorL, speed_set);                    // Apply calculated speed to left motor
    Serial.println("Left motor : " + String(speed_set));  // Log speed
  } else {
    analogWrite(pinMotorL, 70);         // Set minimal speed for the left motor
    Serial.println("Left motor : 70");  // Log default fallback speed
  }
  delay(10);  // Short delay for stability
}
