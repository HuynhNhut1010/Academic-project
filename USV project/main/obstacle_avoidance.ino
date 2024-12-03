void obstacle_avodance() {
  // Switch through different states of obstacle avoidance
  switch (state_avodance) {
    case 0:  // Default state, checking for obstacles
      // If an obstacle is detected in front (distance_1 < obtacle_distance)
      if (distance_1 < obtacle_distance) {
        // If obstacles are also detected on the sides (distance_2 or distance_4)
        if (distance_2 < obtacle_distance || distance_4 < obtacle_distance) {
          // If an obstacle is detected in the right (distance_3), stop the robot
          if (distance_3 < obtacle_distance) {
            analogWrite(pinMotorT, 0);  // Stop motors
            analogWrite(pinMotorL, 0);
            analogWrite(pinMotorR, 0);
            Serial.println("Stop!!!!!");
            // Send HTTP request to inform the server about the obstacle
            adresseHttp = "http://" + ipRas + ":" + port + "/api/esp/post-status?obtacle=1";
            Serial.println(adresseHttp);
            http.begin(client, adresseHttp);
            httpCode = http.POST("");  // POST request to notify about obstacle
            http.end();

            // Delete the coordinates from the server
            adresseHttp = "http://" + ipRas + ":" + port + "/api/esp/delete-coor";
            http.begin(client, adresseHttp);
            httpCode = http.POST("");
            Serial.println(httpCode);
            // Reset the state machine after stopping
            state_avodance = 0;
            http.end();
            j = 0;
            state = 0;
          } else {
            // If there's no obstacle in the right, move to state 1 (Turn Right)
            state_avodance = 1;
          }
        } else {
          // If the side sensors are clear, check the right sensor
          if (distance_3 < obtacle_distance) {
            // Move to state 3 for special handling
            state_avodance = 3;
          } else {
            // If all directions are clear, move to state 1 (Turn Right)
            state_avodance = 1;
          }
        }
      } else {
        // If no obstacle in front, move to state 2 (Turn Left)
        state_avodance = 2;
      }
      break;

    case 1:  // State for turning right
      // If there's an obstacle in front or left, turn right
      if (distance_1 < obtacle_distance) {
        if (distance_2 < obtacle_distance) {
          Serial.println("turn right");
          turn_right(10);      // Turn right with a specified angle (10 degrees)
          state_avodance = 0;  // Return to default state
        } else {
          Serial.println("turn right");
          turn_right(10);      // Turn right
          state_avodance = 0;  // Reset state
        }
      } else {
        // If there's no obstacle in front, check the side sensors
        if (distance_2 < obtacle_distance) {
          if (distance_4 < obtacle_distance) {
            Serial.println("turn right");
            turn_right(5);  // Turn right slightly
            state_avodance = 0;
          } else {
            Serial.println("Turn right");
            turn_right(5);  // Turn right
            state_avodance = 0;
          }
        } else if (distance_4 < obtacle_distance) {
          // If obstacle is detected on the left, turn left
          Serial.println("Turn left");
          turn_left(270);  // Turn left (270 degrees)
          state_avodance = 0;
        } else {
          // If no obstacles, continue forward and move to state 1
          Serial.println("Done");
          delay(100);                  // Short delay
          analogWrite(pinMotorT, 10);  // Move motors forward at speed 10
          pcf_MUI.digitalWrite(P0, 1);
          pcf_MUI.digitalWrite(P1, 1);
          state = 1;
          state_avodance = 0;
        }
      }
      break;

    case 2:  // State for turning left
      // If there's an obstacle in front or right, turn left
      if (distance_3 < obtacle_distance) {
        if (distance_2 < obtacle_distance || distance_4 < obtacle_distance) {
          Serial.println("turn left");
          turn_left(350);  // Turn left (350 degrees)
          state_avodance = 0;
        } else {
          Serial.println("turn left");
          turn_left(350);              // Turn left
          analogWrite(pinMotorL, 75);  // Adjust motor speeds
          analogWrite(pinMotorR, 15);
          state_avodance = 0;
        }
      } else {
        // If no obstacle in the front, check the side sensors
        if (distance_2 < obtacle_distance) {
          if (distance_4 < obtacle_distance) {
            Serial.println("Turn right");
            turn_right(5);  // Turn right
            state_avodance = 0;
          } else {
            Serial.println("Turn right");
            turn_right(90);  // Turn right 90 degrees
            analogWrite(pinMotorL, 75);
            analogWrite(pinMotorR, 15);
            state_avodance = 0;
          }
        } else if (distance_4 < obtacle_distance) {
          // If obstacle on the left, turn left
          Serial.println("Turn left");
          turn_left(270);  // Turn left 270 degrees
          analogWrite(pinMotorL, 75);
          analogWrite(pinMotorR, 15);
          state_avodance = 0;
        } else {
          // If no obstacle detected, stop and move to state 1
          Serial.println("Done");
          analogWrite(pinMotorT, 10);
          delay(100);  // Short delay
          pcf_MUI.digitalWrite(P0, 1);
          pcf_MUI.digitalWrite(P1, 1);
          state = 1;
          state_avodance = 0;
        }
      }
      break;

    case 3:  // Special case for reversing or adjusting path
      // If obstacle is detected in front or right, handle accordingly
      if (distance_1 < obtacle_distance) {
        if (distance_3 < obtacle_distance) {
          state_avodance = 0;
          if (distance_3 < 30) {
            if (distance_1 < 30) {
              analogWrite(pinMotorT, 0);  // Stop the robot
              analogWrite(pinMotorL, 0);
              analogWrite(pinMotorR, 0);
              Serial.println("Stop!!!!!");
              // Inform the server about the obstacle
              adresseHttp = "http://" + ipRas + ":" + port + "/api/esp/post-status?obtacle=1";
              Serial.println(adresseHttp);
              http.begin(client, adresseHttp);
              httpCode = http.POST("");
              http.end();
              // Delete coordinates from server
              adresseHttp = "http://" + ipRas + ":" + port + "/api/esp/delete-coor";
              http.begin(client, adresseHttp);
              httpCode = http.POST("");
              Serial.println(httpCode);
              state_avodance = 0;
              state = 0;
              j = 0;
            } else {
              control_servo(215);  // Turn left
            }
          } else if (distance_1 < 30) {
            control_servo(160);  // Turn right
            analogWrite(pinMotorL, 75);
            analogWrite(pinMotorR, 15);
          } else {
            control_servo(180);  // Move straight
            analogWrite(pinMotorL, 75);
            analogWrite(pinMotorR, 15);
          }
        } else {
          // If no obstacle on the right, turn right
          Serial.println("turn right");
          turn_right(90);  // Turn right 90 degrees
          analogWrite(pinMotorL, 75);
          analogWrite(pinMotorR, 15);
          state_avodance = 0;
        }
      } else {
        // If obstacles are detected on the sides, turn right or left
        if (distance_2 < obtacle_distance) {
          if (distance_4 < obtacle_distance) {
            Serial.println("Turn right");
            turn_right(5);
            state_avodance = 0;
          } else {
            Serial.println("Turn right");
            turn_right(5);
            analogWrite(pinMotorL, 75);
            analogWrite(pinMotorR, 15);
            state_avodance = 0;
          }
        } else if (distance_4 < obtacle_distance) {
          Serial.println("Turn left");
          turn_left(270);  // Turn left 270 degrees
          analogWrite(pinMotorL, 75);
          analogWrite(pinMotorR, 15);
          state_avodance = 0;
        } else {
          // If no obstacles detected, stop and reset state
          Serial.println("Done");
          analogWrite(pinMotorT, 10);
          delay(100);  // Short delay
          pcf_MUI.digitalWrite(P0, 1);
          pcf_MUI.digitalWrite(P1, 1);
          state = 1;
          state_avodance = 0;
        }
      }
      break;
  }
}
