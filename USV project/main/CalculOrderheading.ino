// Function to calculate the order for heading adjustment based on GPS and compass data
void calculOrderheadinging(float headingCompass) {
  if (gps.location.isValid()) {
    /********************* Calculate distance to the next waypoint ***********************/
    distanceToNextPoint = (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),  // Current latitude
      gps.location.lng(),  // Current longitude
      nextLati,            // Target latitude
      nextLong             // Target longitude
    );

    /******** Calculate course to the next waypoint (direction needed to travel) ********/
    courseToNextPoint = TinyGPSPlus::courseTo(
      gps.location.lat(),  // Current latitude
      gps.location.lng(),  // Current longitude
      nextLati,            // Target latitude
      nextLong             // Target longitude
    );
    // 0 = straight ahead, 90 = right turn, 180 = U-turn, 270 = left turn
  }

  // Print the calculated distance and course for debugging
  Serial.print("Distance : ");
  Serial.println(distanceToNextPoint);
  Serial.print("Course : ");
  Serial.println(courseToNextPoint);

  /********************* Calculate the adjustment to heading ***********************/
  courseTo180 = 180 - courseToNextPoint;                // Adjust course to 180 degrees
  cap180 = normalize360(headingCompass + courseTo180);  // normalize to [0, 360)

  // Map the calculated angle to PWM range for servo control
  pwm0 = map(cap180, 0, 360, SERVOMIN, SERVOMAX);

  // Adjust the servo PWM gradually to reach the desired position
  if (pwm_temp > pwm0) {  // Decrease PWM
    while (pwm_temp > pwm0) {
      pca9685.setPWM(SER0, 0, pwm_temp);  // Control servo 0
      pca9685.setPWM(SER1, 0, pwm_temp);  // Control servo 1
      pwm_temp--;
      delay(5);  // Smooth adjustment delay
    }
    pwm_temp = pwm0;
  } else {  // Increase PWM
    while (pwm_temp < pwm0) {
      pca9685.setPWM(SER0, 0, pwm_temp);  // Control servo 0
      pca9685.setPWM(SER1, 0, pwm_temp);  // Control servo 1
      pwm_temp++;
      delay(5);  // Smooth adjustment delay
    }
    pwm_temp = pwm0;
  }

  /********************* Determine turn direction or go straight ***********************/
  if (cap180 > 185 && cap180 < 273) {  // Turn left
    Serial.println(" Turn left");
  } else if (cap180 >= 273 || cap180 < 88) {  // Turn right
    Serial.println(" Turn right");
  } else if (cap180 >= 88 && cap180 < 175) {  // Turn slightly right
    Serial.println(" Turn right");
  } else {  // Go straight
    Serial.println(" Go ahead");
  }
}

/*************************** normalize angle to [0, 360) ******************************/
short normalize360(short valeur) {
  if (valeur >= 360) {  // Reduce angle if it exceeds 360
    valeur = valeur - 360;
  }
  if (valeur <= 0) {                                         // Adjust angle if it's negative
    valeur = 180 + (360 - courseToNextPoint) + magSensor();  // Recalculate within range
    if (valeur >= 360) {
      valeur = valeur - 360;  // Reduce if it exceeds 360 after recalculation
    }
  }
  return valeur;
}

// Function to control servo based on desired heading
void control_servo(int cap) {
  pwm0 = map(cap, 0, 360, SERVOMIN, SERVOMAX);  // Map heading to PWM range
  if (pwm_temp > pwm0) {                        // Decrease PWM to reach target
    while (pwm_temp > pwm0) {
      pca9685.setPWM(SER0, 0, pwm_temp);  // Control servo 0
      pca9685.setPWM(SER1, 0, pwm_temp);  // Control servo 1
      pwm_temp--;
      delay(5);  // Smooth adjustment delay
    }
    pwm_temp = pwm0;
  } else {  // Increase PWM to reach target
    while (pwm_temp < pwm0) {
      pca9685.setPWM(SER0, 0, pwm_temp);  // Control servo 0
      pca9685.setPWM(SER1, 0, pwm_temp);  // Control servo 1
      pwm_temp++;
      delay(5);  // Smooth adjustment delay
    }
    pwm_temp = pwm0;
  }
}
