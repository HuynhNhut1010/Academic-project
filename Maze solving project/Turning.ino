// void turn_around() {
//   //noInterrupts();
//   left_obtacle = lox1.readRange() / 10.0;
//   right_obtacle = lox3.readRange() / 10.0;
//   //stop();
//   int turn = 0;
//   if (right_obtacle > left_obtacle) {
//     digitalWrite(in1, HIGH);
//     digitalWrite(in2, LOW);
//     turn = 0;
//   } else {
//     digitalWrite(in3, HIGH);
//     digitalWrite(in4, LOW);
//     turn = 1;
//   }
//   mpu6050.update();
//   display.clearDisplay();
//   print_oled("turn around", 0, 0);
//   PrevDirec = mpu6050.getAngleZ();
//   while (abs(PrevDirec - mpu6050.getAngleZ()) <= 173) {
//     setpoint_turning = 173;
//     input_turning = abs(PrevDirec - mpu6050.getAngleZ());
//     pid_turning.Compute();
//     if (turn == 0) {
//       if (output_turning < 0) {
//         digitalWrite(in1, LOW);
//         digitalWrite(in2, HIGH);
//         output_turning = -output_turning;
//       } else {
//         digitalWrite(in1, HIGH);
//         digitalWrite(in2, LOW);
//       }
//     } else {
//       if (output_turning < 0) {
//         digitalWrite(in3, LOW);
//         digitalWrite(in4, HIGH);
//         output_turning = -output_turning;
//       } else {
//         digitalWrite(in3, HIGH);
//         digitalWrite(in4, LOW);
//       }
//     }

//     analogWrite(enB, output_turning);
//     analogWrite(enA, output_turning - 10);
//     // Serial.print(PrevDirec);
//     // Serial.print("  ");
//     // Serial.print(mpu6050.getAngleZ());
//     // Serial.println();

//     mpu6050.update();
//   }
//   display.clearDisplay();
//   Serial.println("done");
//   analogWrite(enA, 0);
//   analogWrite(enB, 0);
//   digitalWrite(in1, LOW);
//   digitalWrite(in2, HIGH);
//   digitalWrite(in3, LOW);
//   digitalWrite(in4, HIGH);
//   position = position + 2;
//   if (position >= 5) {
//     position = position % 4;
//   }

//   xung[0] = 0;
//   xung[1] = 0;
//   p_prev = 0;
//   //interrupts();
// }

// void turn_left() {
//   //noInterrupts();
//   //stop();
//   Serial.println(first_angle);
//   digitalWrite(in3, HIGH);
//   digitalWrite(in4, LOW);
//   mpu6050.update();
//   PrevDirec = mpu6050.getAngleZ();
//   display.clearDisplay();
//   print_oled("turn left", 0, 0);
//   while (abs(PrevDirec - mpu6050.getAngleZ()) <= 84) {  //abs(PrevDirec - mpu6050.getAngleZ()) <= 90first_angle + 90 >= mpu6050.getAngleZ()
//     setpoint_turning = 84;
//     input_turning = abs(PrevDirec - mpu6050.getAngleZ());
//     pid_turning.Compute();
//     if (output_turning < 0) {
//       digitalWrite(in3, LOW);
//       digitalWrite(in4, HIGH);
//       output_turning = -output_turning;
//     } else {
//       digitalWrite(in3, HIGH);
//       digitalWrite(in4, LOW);
//     }
//     analogWrite(enB, output_turning);
//     analogWrite(enA, output_turning);
//     // Serial.print(output_turning);
//     // Serial.print("  ");
//     // Serial.print(PrevDirec);
//     // Serial.print("  ");
//     Serial.print(mpu6050.getAngleZ());
//     Serial.println();
//     mpu6050.update();
//   }
//   // mpu6050.update();
//   // first_angle = first_angle + 90;

//   analogWrite(enA, 0);
//   analogWrite(enB, 0);
//   digitalWrite(in3, LOW);
//   digitalWrite(in4, HIGH);
//   //delay(500);

//   print_oled(String(PrevDirec), 0, 10);
//   print_oled(String(mpu6050.getAngleZ()), 0, 20);
//   Serial.println("done");


//   position++;
//   if (position >= 5) {
//     position = position % 4;
//   }

//   // xung[0] = 0;
//   // xung[1] = 0;
//   // p_prev = 0;
//   //interrupts();
// }

// void turn_right() {
//   //noInterrupts();
//   Serial.println(ahead_obtacle);
//   //stop();
//   digitalWrite(in1, HIGH);
//   digitalWrite(in2, LOW);
//   mpu6050.update();
//   PrevDirec = mpu6050.getAngleZ();
//   display.clearDisplay();
//   print_oled("turn right", 0, 0);\
//   setpoint_turning = 83;
//   while (abs(PrevDirec - mpu6050.getAngleZ()) <= 83) {
//     input_turning = abs(PrevDirec - mpu6050.getAngleZ());
//     pid_turning.Compute();
//     if (output_turning < 0) {
//       digitalWrite(in1, LOW);
//       digitalWrite(in2, HIGH);
//       output_turning = -output_turning;
//     } else {
//       digitalWrite(in1, HIGH);
//       digitalWrite(in2, LOW);
//     }
//     Serial.print(mpu6050.getAngleZ());
//     Serial.println();
//     analogWrite(enB, output_turning);
//     analogWrite(enA, output_turning);
//     mpu6050.update();
//   }
//   position--;
//   analogWrite(enA, 0);
//   analogWrite(enB, 0);
//   digitalWrite(in1, LOW);
//   digitalWrite(in2, HIGH);
//   //delay(200);
//   Serial.println("done");
//   digitalWrite(in1, LOW);
//   digitalWrite(in2, HIGH);
//   while (position <= 0) {
//     position = 4 + position;
//   }
//   xung[0] = 1;
//   xung[1] = 1;
//   p_prev = 0;
//   display.clearDisplay();
//   //interrupts();
// }

void stop() {
  // analogWrite(enA, 0);
  // analogWrite(enB, 0);
  // delay(300);
  analogWrite(enA, 150);
  analogWrite(enB, 150);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(100);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}



// void caculate_speed() {

//   noInterrupts();
//   for (int i = 0; i < 2; i++) {
//     pos[i] = posi[i];
//     xung_i[i] = xung[i];
//   }
//   interrupts();

//   long currT = micros();
//   float deltaT = ((float)(currT - prevT_i)) / (1.0e6);
//   for (int i = 0; i < 2; i++) {
//     velocity_i[i] = (pos[i] - posPrev[i]) / deltaT;
//     posPrev[i] = pos[i];
//     v1[i] = velocity_i[i] / 210.0 * 60.0;  // rpm

//     v1Flit[i] = 0.854 * v1Flit[i] + 0.0728 * v1[i] + 0.0728 * v1Prev[i];  //filter V1
//     v1Prev[i] = v1[i];
//     input[i] = v1Flit[i];
//   }
//   prevT_i = currT;
// }

void go_ahead() {

  //int reverse = 0;
  //print_oled("Go ahead", 40, 0);
  if (lox1.isRangeComplete()) {
    left_obtacle = lox1.readRange()  / 10.0;
  }
  if (lox2.isRangeComplete()) {
    ahead_obtacle = lox2.readRange()  / 10.0;
  }
  // if (lox3.isRangeComplete()) {
  //   right_obtacle = lox3.readRangeResult() / 10.0;
  // }
  if (lox4.isRangeComplete()) {
    ahead_obtacle2 = lox4.readRange() / 10.0;
  }

  if (left_obtacle < 17) {

    input_follow = left_obtacle;
    setpoint_follow = 7;
    pid_follow.Compute();  //follow wall
                           // for (int i = 0; i < 2; i++) {
                           //   if (output[i] < 0) {
                           //     digitalWrite(in1, HIGH);
                           //     digitalWrite(in2, LOW);
                           //     digitalWrite(in3, HIGH);
                           //     digitalWrite(in4, LOW);
                           //     pwr = -output[i];
                           //   } else {
                           //     digitalWrite(in1, LOW);
                           //     digitalWrite(in2, HIGH);
                           //     digitalWrite(in3, LOW);
                           //     digitalWrite(in4, HIGH);
                           //     pwr = output[i];
                           //   }
                           // }
    pwr_follow0 = analog0 + output_follow - 4;
    pwr_follow1 = analog0 - output_follow;
    analogWrite(enB, pwr_follow0);  //trai 140 + output_follow
    analogWrite(enA, pwr_follow1);  //phai 150 - output_follow
    // if (ahead_obtacle < 45 || left_obtacle > 15) {
    //   analogWrite(enB, pwr_follow0 - 10);  //trai
    //   analogWrite(enA, pwr_follow1 - 10);
    // }
    // else if (reverse == 1) {
    //     pwr_follow0 = analog0 - output_follow;
    //     pwr_follow1 = analog0 + output_follow + 13;
    //     analogWrite(enB, pwr_follow0); //trai 140 - output_follow
    //     analogWrite(enA, pwr_follow1); //phai 150 + output_follow
    //     if(ahead_obtacle < 45 || left_obtacle > 25){
    //       analogWrite(enB, pwr_follow0 - 18); //trai
    //       analogWrite(enA, pwr_follow1 - 18);
    //     }
    // }
  }
  if (left_obtacle < 5) {
    while (ahead_obtacle <= 10) {
      ahead_obtacle = lox2.readRange() / 10.0;
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enB, 60 - 5);
      analogWrite(enA, 90);
    }
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, analog0 + 10);  //trai
    analogWrite(enA, analog0 - 20);
  }
  if (ahead_obtacle < 7) {
    while (ahead_obtacle <= 10) {
      ahead_obtacle = lox2.readRange() / 10.0;
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enB, 60 - 5);
      analogWrite(enA, 70);
    }
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  // if (right_obtacle < 3) {
  //   analogWrite(enB, pwr_follow0 - 20);  //trai
  //   analogWrite(enA, pwr_follow1 + 40);
  // }
}

void go_ahead_right() {

  if (lox1.isRangeComplete()) {
    left_obtacle = lox1.readRangeResult() / 10.0;
  }
  if (lox2.isRangeComplete()) {
    ahead_obtacle = lox2.readRangeResult() / 10.0;
  }
  // if (lox3.isRangeComplete()) {
  //   right_obtacle = lox3.readRangeResult() / 10.0;
  // }
  if (lox4.isRangeComplete()) {
    ahead_obtacle2 = lox4.readRange() / 10.0;
  }

  if (left_obtacle > 20) {
    input_follow = right_obtacle;
    setpoint_follow = 7;

    pid_follow.Compute();
    pwr_follow0 = analog0 - output_follow;
    pwr_follow1 = analog0 + output_follow - 7;
    analogWrite(enB, pwr_follow0);  //trai 140 + output_follow
    analogWrite(enA, pwr_follow1);  //phai 150 - output_follow
    if (right_obtacle < 3) {
      analogWrite(enB, pwr_follow0-20);  //trai
      analogWrite(enA, pwr_follow1);
    }
  }
  else if(left_obtacle < 20 || right_obtacle > 20 ){
      input_follow = left_obtacle;
      setpoint_follow = 7;

      pid_follow.Compute();
      pwr_follow0 = analog0 + output_follow;
      pwr_follow1 = analog0 - output_follow - 3;
      analogWrite(enB, pwr_follow0);  //trai 140 + output_follow
      analogWrite(enA, pwr_follow1);  //phai 150 - output_follow
      if (left_obtacle < 3) {
        analogWrite(enB, pwr_follow0 );  //trai
        analogWrite(enA, pwr_follow1 - 10);
      }
  }

}

void balance() {
  lox2.startRangeContinuous(50);
  lox4.startRangeContinuous(50);
  while (!lox2.isRangeComplete()) { ; }
  while (!lox4.isRangeComplete()) { ; }
  if (lox2.isRangeComplete()) {
    ahead_obtacle = lox2.readRangeResult() / 10.0 - 4;
  }
  if (lox4.isRangeComplete()) {
    ahead_obtacle2 = lox4.readRange() / 10.0;
  }
  setpoint[0] = 0;
  while ((ahead_obtacle - ahead_obtacle2) >= 0.6 || (ahead_obtacle - ahead_obtacle2) <= 0) {
    Serial.print(ahead_obtacle - ahead_obtacle2);
    Serial.print("  ");
    Serial.println();
    ahead_obtacle = lox2.readRangeResult() / 10.0 - 4;
    ahead_obtacle2 = lox4.readRange() / 10.0;
    input[0] = ahead_obtacle - ahead_obtacle2;
    pid.Compute();
    if (output[0] > 0) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enA, output[0]);
    }
    if (output[0] < 0) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enA, -output[0]);
    }

    // if ((ahead_obtacle >= 8 && ahead_obtacle2 >= 8)) {
    //   analogWrite(enB, 0);  //trai
    //   analogWrite(enA, 0);
    //   break;
    // }
  }
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, 0);
  analogWrite(enA, 0);
  lox2.stopRangeContinuous();
  lox4.stopRangeContinuous();
}