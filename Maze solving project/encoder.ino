#include <MPU6050_tockn.h>
//#include <analogWrite.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_VL53L0X.h"
#include <SPI.h>
#include <Adafruit_GFX.h>

//define pin OLED
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 32     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


//define pin
#define enA 4  //right
#define in1 16
#define in2 17
#define enB 19  //left
#define in3 5
#define in4 18

#define button 34

#define trig 2
#define echo 13

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x33
#define LOX4_ADDRESS 0x32
#define SHT_LOX1 26  //left
#define SHT_LOX2 25  //above left
#define SHT_LOX3 33  //right
#define SHT_LOX4 32  //above right
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();


VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;
int time_update = 0;
MPU6050 mpu6050(Wire);

int ena[] = { 27, 99 };  //27, 14 left      36, 39 right
int enb[] = { 14, 99 };

//maze
int reset = 1;
int continue_left = 0;
int nga3 = 1;
int maze[300][6];
int route[300][6];
int route_i = 0;
int direction[3];
int position = 1;
int PrevDirec = 0;
int Direc_z = 0;
int Prev_analogB;
int Prev_analogA;
int wall = 15;
int sensor[300][4];
int point;
int m = 0;
int repeat_position = 0;
int p_prev = 0;
int p = 0;
int state = 0;
int dead_end = 0;
int recent_position = 0;
int next_point;
int check_route = 1;
int turn;
int xung_i[2] = { 0, 0 };
int time_display = 0;
int increase = 0;
int decrease = 0;
int stop_button = 0;
int choose = 0;
int button_value;
int flag = 0;
int time_read = 0;
int timeDelay = 6000;
int flag_choose = 0;
int analog0 = 60;
int test = 0;
double firahead_obtacle2st_angle = 0;
int time_test = 0;
int xung_prev = 0;
int solve = 0;
int offset_speed = 0;
//define obtackle
double right_obtacle = 13;
double ahead_obtacle = 13;
double left_obtacle = 13;
double ahead_obtacle2 = 13;
int error_sensor1 = 0;
int error_sensor2 = 0;
int error_sensor3 = 0;
int error_sensor4 = 0;
int first_angle = 0;
int ahead_obtacle3 = 0;
int duration = 0;

//define encoder
int xung[2];
int prevT_i = 0;
float velocity[] = { 0, 0 };
float velocity_i[] = { 0, 0 };
int posPrev[] = { 0, 0 };
int posi[] = { 0, 0 };
int pos[] = { 0, 0 };
float v1Flit[] = { 0, 0 };
float v1Prev[] = { 0, 0 };
float v1[] = { 0, 0 };


//define PID
int speed_set = 300;
double originalSetpoint = speed_set;  //set Speed
double setpoint[] = { originalSetpoint, originalSetpoint };
double input[2], output[2];
double Kp[] = { 90, 90 };  //1 ben phai 0 ben trai
double Ki[] = { 12, 12 };
double Kd[] = { 2, 2 };
double Kp_turning = 101;
double Ki_turning = 8;
double Kd_turning = 140;
double input_turning, output_turning;
double setpoint_turning;
double Kp_follow = 101;  //105 , low speed: 3
double Ki_follow = 8;    //200, low speed: 50
double Kd_follow = 63;   //7.5, low speed: 0.04
double input_follow, output_follow;
double setpoint_follow = 0;
int pwr = 0;
int pwr_follow0 = 0;
int pwr_follow1 = 0;
PID pid(&input[0], &output[0], &setpoint[0], Kp[0], Ki[0], Kd[0], 0);
PID pid1(&input[1], &output[1], &setpoint[1], Kp[1], Ki[1], Kd[1], 0);
PID pid_turning(&input_turning, &output_turning, &setpoint_turning, Kp_turning, Ki_turning, Kd_turning, DIRECT);
PID pid_follow(&input_follow, &output_follow, &setpoint_follow, Kp_follow, Ki_follow, Kd_follow, DIRECT);

void setup() {

  Serial.begin(115200);

  Wire.begin();
  //Wire.setClock(400000);
 // mpu6050.begin();
  //mpu6050.calcGyroOffsets(true);
  //pinMode(33, INPUT);
  //pinMode(26, INPUT);
  //pinMode(32, INPUT);
  for (int i = 0; i < 2; i++) {
    pinMode(ena[i], INPUT);
    pinMode(enb[i], INPUT);
  }
  pinMode(enA, OUTPUT);  //right
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);  //left
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  pinMode(button, INPUT);

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  pinMode(SHT_LOX4, OUTPUT);
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  setID();

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-130, 130);

  pid1.SetMode(AUTOMATIC);
  pid1.SetSampleTime(10);
  pid1.SetOutputLimits(-100, 100);

  pid_turning.SetMode(AUTOMATIC);
  pid_turning.SetSampleTime(10);
  pid_turning.SetOutputLimits(-55, 55);

  pid_follow.SetMode(AUTOMATIC);
  pid_follow.SetSampleTime(10);
  pid_follow.SetOutputLimits(-25, 25);

  //attachInterrupt(digitalPinToInterrupt(ena[0]), readEncoder0, RISING);
  for (int i = 0; i <= 5; i++) {
    maze[0][i] = 0;
  }
  for (int i = 0; i <= 3; i++) {
    sensor[0][i] = i + 1;
  }
  maze[0][2] = 1;
  route[0][0] = 0;
  Serial.println();
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }


  display.clearDisplay();
  // float angle[100];
  // for (int iii=0;iii<100;iii++){
  // mpu6050.update();
  // angle[iii] =  mpu6050.getAngleZ();
  // Serial.println(angle[iii]);
  // delay(50);
  // }
  // for (int iii=0;iii<100;iii++){
  // first_angle = angle[iii] + first_angle;
  // }
  // first_angle = first_angle / 100.000;
  // Serial.print("first_angle:  ");
  // Serial.println(first_angle);
  // mpu6050.update();
  // first_angle = mpu6050.getAngleZ();
  lox1.startRangeContinuous(50);
  lox2.startRangeContinuous(50);
  lox3.startRangeContinuous(50);
  lox4.startRangeContinuous(50);
  while (!lox3.isRangeComplete()) { Serial.println("Load sensor3"); }
  while (!lox2.isRangeComplete()) { Serial.println("Load sensor2"); }
  while (!lox1.isRangeComplete()) { Serial.println("Load sensor2"); }
  while (!lox4.isRangeComplete()) { Serial.println("Load sensor4"); }
}

void loop() {
  if (millis() > time_update + 700) {
    time_update = millis();
    if (!lox1.isRangeComplete()) {
      error_sensor1++;
    } else {
      error_sensor1--;
      if (error_sensor1 <= 0) {
        error_sensor1 = 0;
      }
    }
    if (!lox2.isRangeComplete()) {
      error_sensor2++;
    } else {
      error_sensor2--;
      if (error_sensor2 <= 0) {
        error_sensor1 = 0;
      }
    }
    if (!lox3.isRangeComplete()) {
      error_sensor3++;
    } else {
      error_sensor3--;
      if (error_sensor3 <= 0) {
        error_sensor1 = 0;
      }
    }
    if (!lox4.isRangeComplete()) {
      error_sensor4++;
    } else {
      error_sensor4--;
    }
    if (error_sensor1 >= 7 || error_sensor2 >= 7 || error_sensor3 >= 7 || error_sensor4 >= 7) {
      analogWrite(enA,0);
      analogWrite(enB,0);
      display.clearDisplay();
      print_oled("Error sensor", 0, 0);
      resetSensor();
      delay(500);
      while (!lox3.isRangeComplete()) { Serial.println("Load sensor3"); }
      while (!lox2.isRangeComplete()) { Serial.println("Load sensor2"); }
      while (!lox1.isRangeComplete()) { Serial.println("Load sensor1"); }
      while (!lox4.isRangeComplete()) { Serial.println("Load sensor4"); }
      error_sensor1 = 0;
      error_sensor2 = 0;
      error_sensor3 = 0;
      error_sensor4 = 0;
    }
    if (stop_button == 1) {
      state = 9; // Setting PID
      analyze_map();
      display.clearDisplay();
      print_oled("Solve ready", 0, 0);
    }
  }
  switch (state) {
    case 0:
      {
        print_oled("Point:   ", 0, 20);
        print_oled(String(maze[m][0]), 50, 20);
        if ((right_obtacle > 25 && (xung[0] - xung_prev) >= 1300) || ahead_obtacle < 18 || left_obtacle > 25 || continue_left == 1) {
          continue_left = 0;
          lox1.stopRangeContinuous();
          lox2.stopRangeContinuous();
          lox3.stopRangeContinuous();
          lox4.stopRangeContinuous();
          lox1.rangingTest(&measure1, false);
          lox2.rangingTest(&measure2, false);
          lox3.rangingTest(&measure3, false);
          lox4.rangingTest(&measure4, false);
          left_obtacle = measure1.RangeMilliMeter / 10.0;
          ahead_obtacle = measure2.RangeMilliMeter / 10.0;
          right_obtacle = measure3.RangeMilliMeter / 10.0;
          ahead_obtacle2 = measure4.RangeMilliMeter / 10.0;
          if (left_obtacle > 20) {
            state = 1;
            break;
          } else if (ahead_obtacle < 20) {
            if (ahead_obtacle2 < 15) {
              if (right_obtacle < 15) {
                state = 2;
                dead_end = 1;
                break;
              } else {
                state = 3;
                break;
              }
            }
          }
          // point++;
          if (right_obtacle > 15 && (xung[0] - xung_prev) >= 1300 && ahead_obtacle > 40) {
            // stop();
            // delay(200);
            analogWrite(enB, analog0);
            analogWrite(enA, analog0 - offset_speed);
            Serial.println("UPDATE");
            display.clearDisplay();
            xung_prev = xung[0];
            print_oled("UPDATE", 0, 0);
            m = m + 1;
            Serial.println(m);
            scan_maze();
            //flag_scan = 1;
          }
          lox1.startRangeContinuous(50);
          lox2.startRangeContinuous(50);
          lox3.startRangeContinuous(50);
          lox4.startRangeContinuous(50);
          while (!lox2.isRangeComplete()) { ; }
          while (!lox1.isRangeComplete()) { ; }
          while (!lox3.isRangeComplete()) { ; }
          while (!lox3.isRangeComplete()) { ; }
        }
        go_ahead();
        break;
      }
    case 1:  // turn left
      {
        lox1.stopRangeContinuous();
        lox2.stopRangeContinuous();
        lox3.stopRangeContinuous();
        lox4.stopRangeContinuous();
        xung[0] = 0;

        while (xung[0] <= 600) {  //200
          analogWrite(enB, analog0);
          analogWrite(enA, analog0 - offset_speed);
          left_obtacle = lox1.readRange() / 10.0;
        }
        stop();
        delay(100);
        right_obtacle = lox3.readRange() / 10.0;
        ahead_obtacle2 = lox4.readRange() / 10.0;
        ahead_obtacle = lox2.readRange() / 10.0;
        if (ahead_obtacle2 < 15 && ahead_obtacle < 15) {
          while (ahead_obtacle2 <= 7) {
            ahead_obtacle2 = lox4.readRange() / 10.0;
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);
            analogWrite(enB, 110);
            analogWrite(enA, 110 - offset_speed);
          }
          balance();
        }

        if (solve == 0 && left_obtacle > 15) {
          display.clearDisplay();
          print_oled("UPDATE", 0, 0);
          Serial.println("UPDATE");
          m = m + 1;
          Serial.println(m);
        }

        for (int i = 0; i <= m; i++) {
          for (int j = 0; j <= 5; j++) {
            Serial.print(maze[i][j]);
            Serial.print("  ");
          }
          Serial.print("   sensor: ");
          for (int j = 0; j <= 5; j++) {
            Serial.print(sensor[i][j]);
            Serial.print("  ");
          }
          Serial.println();
        }
        if (left_obtacle > 15) {
          scan_maze();
          turn_left();
          delay(100);
          ahead_obtacle = lox2.readRange() / 10.0;
          if (ahead_obtacle > 10) {
            xung[0] = 0;
            while (xung[0] <= 300 && ahead_obtacle >= 10) {  //200
              analogWrite(enB, analog0);
              analogWrite(enA, analog0 - offset_speed);
              ahead_obtacle = lox2.readRange() / 10.0;
              left_obtacle = lox1.readRange() / 10.0;
            }
            if (left_obtacle > 15) {
              continue_left = 1;
            }
            lox1.startRangeContinuous(50);
            lox2.startRangeContinuous(50);
            lox3.startRangeContinuous(50);
            lox4.startRangeContinuous(50);
            while (!lox2.isRangeComplete()) { ; }
            while (!lox1.isRangeComplete()) { ; }
            while (!lox3.isRangeComplete()) { ; }
            while (!lox4.isRangeComplete()) { ; }
          } else {
            lox1.startRangeContinuous(50);
            lox2.startRangeContinuous(50);
            lox3.startRangeContinuous(50);
            lox4.startRangeContinuous(50);
            while (!lox2.isRangeComplete()) { ; }
            while (!lox1.isRangeComplete()) { ; }
            while (!lox3.isRangeComplete()) { ; }
            while (!lox4.isRangeComplete()) { ; }
            turn_right();
          }
        }
        if (solve == 0) {
          state = 0;
        } else {
          state = 9;
        }
        xung[0] = 0;
        xung_i[0] = 0;
        xung_prev = 0;
        p = 0;
        p_prev = p;
        lox1.startRangeContinuous(50);
        lox2.startRangeContinuous(50);
        lox3.startRangeContinuous(50);
        lox4.startRangeContinuous(50);
        while (!lox2.isRangeComplete()) { ; }
        while (!lox1.isRangeComplete()) { ; }
        while (!lox3.isRangeComplete()) { ; }
        while (!lox4.isRangeComplete()) { ; }
        break;
      }

    case 2:  //turn around
      {
        lox1.stopRangeContinuous();
        lox2.stopRangeContinuous();
        lox3.stopRangeContinuous();
        lox4.stopRangeContinuous();
        stop();
        delay(100);
        //update maze
        //point++;
        left_obtacle = lox1.readRange() / 10.0;
        right_obtacle = lox3.readRange() / 10.0;
        if (solve == 0) {
          display.clearDisplay();
          print_oled("UPDATE", 0, 0);
          Serial.println("UPDATE");
          m = m + 1;
          Serial.println(m);
          scan_maze();
        }
        ahead_obtacle = lox2.readRange() / 10.0;
        ahead_obtacle2 = lox4.readRange() / 10.0;
        while (ahead_obtacle2 <= 7 || ahead_obtacle <= 7) {
          ahead_obtacle2 = lox4.readRange() / 10.0;
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
          analogWrite(enB, 110);
          analogWrite(enA, 110 - offset_speed);
        }
        balance();

        for (int i = 0; i <= m; i++) {
          for (int j = 0; j <= 5; j++) {
            Serial.print(maze[i][j]);
            Serial.print("  ");
          }
          Serial.print("   sensor: ");
          for (int j = 0; j <= 5; j++) {
            Serial.print(sensor[i][j]);
            Serial.print("  ");
          }
          Serial.println();
        }

        turn_around();
        Serial.print("position: ");
        Serial.println(position);
        xung[0] = 0;
        while (xung[0] <= 300) {  //200
          analogWrite(enB, analog0);
          analogWrite(enA, analog0 - offset_speed);
          //mpu6050.update();
        }
        if (solve == 0) {
          state = 0;
        } else {
          state = 9;
        }

        xung[0] = 0;
        xung_i[0] = 0;
        xung_prev = 0;
        p = 0;
        p_prev = p;
        lox1.startRangeContinuous(50);
        lox2.startRangeContinuous(50);
        lox3.startRangeContinuous(50);
        lox4.startRangeContinuous(50);
        while (!lox2.isRangeComplete()) { ; }
        while (!lox1.isRangeComplete()) { ; }
        while (!lox3.isRangeComplete()) { ; }
        while (!lox4.isRangeComplete()) { ; }
        break;
      }

    case 3:  //turn right
      {
        stop();
        delay(100);
        lox1.stopRangeContinuous();
        lox2.stopRangeContinuous();
        lox3.stopRangeContinuous();
        lox4.stopRangeContinuous();
        ahead_obtacle2 = lox4.readRange() / 10.0;
        while (ahead_obtacle2 <= 7) {
          ahead_obtacle2 = lox4.readRange() / 10.0;
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
          analogWrite(enB, 110);
          analogWrite(enA, 110 - offset_speed);
        }
        if (solve == 0) {
          balance();
          display.clearDisplay();
          print_oled("UPDATE", 0, 0);
          Serial.println("UPDATE");
          m = m + 1;
          Serial.println(m);
          scan_maze();
          turn_right();
          state = 0;
          xung[0] = 0;
          while (xung[0] <= 400 && ahead_obtacle > 10) {  //200
            analogWrite(enB, analog0);
            analogWrite(enA, analog0 - offset_speed);
            ahead_obtacle = lox2.readRange() / 10.0;
            left_obtacle = lox1.readRange() / 10.0;
          }
          if (left_obtacle > 15) {
            continue_left = 1;
          }
        } else {
          xung[0] = 0;
          while (xung[0] <= 650) {  //200
            analogWrite(enB, analog0);
            analogWrite(enA, analog0 - offset_speed);
          }
          stop();
          delay(100);
          turn_right();
          delay(100);
          xung[0] = 0;
          while (xung[0] <= 300 && ahead_obtacle >= 2) {  //200
            analogWrite(enB, analog0);
            analogWrite(enA, analog0 - offset_speed);
            ahead_obtacle = lox2.readRange() / 10.0;
          }
          state = 9;
        }
        xung[0] = 0;
        xung_i[0] = 0;
        xung_prev = 0;
        p = 0;
        p_prev = p;
        lox1.startRangeContinuous(50);
        lox2.startRangeContinuous(50);
        lox3.startRangeContinuous(50);
        lox4.startRangeContinuous(50);
        while (!lox2.isRangeComplete()) { ; }
        while (!lox1.isRangeComplete()) { ; }
        while (!lox3.isRangeComplete()) { ; }
        while (!lox4.isRangeComplete()) { ; }
        break;
      }
    case 9:
      {
        //stop();
        while (choose == 0 && solve == 0) {
          lox1.stopRangeContinuous();
          lox2.stopRangeContinuous();
          lox3.stopRangeContinuous();
          lox4.stopRangeContinuous();
          scan_button();
          xung[0] = 0;
          xung_i[0] = 0;
          xung_prev = 0;
          position = 1;
          analogWrite(enA, 0);
          analogWrite(enB, 0);
          solve_maze();
          for (int i = 0; i <= m; i++) {
            for (int j = 0; j <= 5; j++) {
              Serial.print(maze[i][j]);
              Serial.print("  ");
            }
            Serial.print("   sensor: ");
            for (int j = 0; j <= 5; j++) {
              Serial.print(sensor[i][j]);
              Serial.print("  ");
            }
            Serial.println();
          }
          Serial.println("Route: ");
          for (int i = 0; i <= route_i; i++) {
            Serial.print(route[i][0]);
            Serial.print("  ");
          }
          delay(200);
          reset = 1;
        }
        solve = 1;
        if (reset == 1) {
          lox1.startRangeContinuous(50);
          lox2.startRangeContinuous(50);
          lox3.startRangeContinuous(50);
          lox4.startRangeContinuous(50);
          while (!lox2.isRangeComplete()) { ; }
          while (!lox1.isRangeComplete()) { ; }
          while (!lox3.isRangeComplete()) { ; }
          reset = 0;
        }
        go_ahead_right();
        if (right_obtacle > 15 || left_obtacle > 15) {
          //display.clearDisplay();
          lox1.stopRangeContinuous();
          lox2.stopRangeContinuous();
          lox3.stopRangeContinuous();
          lox4.stopRangeContinuous();
          recent_position = next_point;
          check_route++;
          solve_maze();
          switch (turn) {
            case 0:
              {
                state = 3;
                break;
              }
            case 1:
              {
                xung[0] = 0;
                display.clearDisplay();
                print_oled("GO GO GO", 0, 20);
                lox1.startRangeContinuous(50);
                lox2.startRangeContinuous(50);
                lox3.startRangeContinuous(50);
                lox4.startRangeContinuous(50);
                while (!lox2.isRangeComplete()) { ; }
                while (!lox1.isRangeComplete()) { ; }
                while (!lox3.isRangeComplete()) { ; }
                while (xung[0] <= 1000) {  //200
                  go_ahead_right();
                }
                break;
              }
            case 2:
              {
                state = 1;
                break;
              }
          }
        }
        lox1.startRangeContinuous(50);
        lox2.startRangeContinuous(50);
        lox3.startRangeContinuous(50);
        while (!lox2.isRangeComplete()) { ; }
        while (!lox1.isRangeComplete()) { ; }
        while (!lox3.isRangeComplete()) { ; }
        break;
      }




    case 4:  //test and measure distance of maze
      {
        balance();
        // caculate_speed();
        // print_oled("Point: ", 0, 10);
        // print_oled(String(xung_i[0]), 40, 10);
        // display.clearDisplay();
        // go_ahead();
        // ahead_obtacle = measure2.RangeMilliMeter / 10;
        // if (ahead_obtacle < 20) {
        //   stop();
        //   delay(5000);
        //   turn_left();
        //   delay(200);
        // }
        // if (ahead_obtacle < 20 && right_obtacle < 10 && left_obtacle < 10) {
        //   stop();
        //   delay(5000);
        //   turn_around();
        //   delay(200);
        // }
        // caculate_speed();
        // p = xung_i[0] / (430 * 1);
        // print_oled("Point: ", 0, 10);
        // print_oled(String(p), 40, 10);
        // print_oled(String(xung_i[0]), 0, 20);
        // if (millis() > time_display + 200) {
        //   display.clearDisplay();
        //   time_display = millis();
        // }


        // int prev_speed1 = output[0];
        // p = xung_i[1]/(467*2);
        // if (p_prev != p){
        //   display.clearDisplay();
        //   //stop();
        //   print_oled("STOP!!!!!",0,0);
        //   //delay(2000);
        //   display.clearDisplay();
        //   p_prev = p;5
        //}
        // caculate_speed();
        // mpu6050.update();
        // Serial.print(mpu6050.getAngleZ());
        // Serial.print(" ");
        // if (time_test + 5000 <= millis()){
        //   turn_left();
        //   time_test = millis();
        // }

        // analogWrite(enA, 200 + 30);
        // analogWrite(enB, 200);
        break;
      }


    case 5:  //Back up state
      {
        //scan_button();
        //int choose_speed = 0;
        //caculate_speed();
        go_ahead();
        //mpu6050.update();
        if (left_obtacle > 25) {
          lox1.stopRangeContinuous();
          lox2.stopRangeContinuous();
          lox3.stopRangeContinuous();
          lox4.stopRangeContinuous();
          while (left_obtacle > 13 && ahead_obtacle > 8) {
            analogWrite(enB, 37);
            analogWrite(enA, 85);
            delay(200);
            analogWrite(enB, 0);
            analogWrite(enA, 0);
            delay(200);
            left_obtacle = lox1.readRange() / 10.0;
            ahead_obtacle = lox2.readRange() / 10.0;
            Serial.println(left_obtacle);
            
          }
          display.clearDisplay();
          print_oled("DONE", 1, 1);
          // stop();
          // delay(200);
          // turn_left();
          // delay(200);
          // if (ahead_obtacle > 15 && ahead_obtacle2 > 15) {
          //   //xung[0] = 0;
          //   //while (xung[0] <= 400) {  //200
          //     analogWrite(enB, analog0 - 4);
          //     analogWrite(enA, analog0);
          //     mpu6050.update();
          //     left_obtacle = lox1.readRange() / 10.0;
          //     right_obtacle = lox3.readRange() / 10.0;
          //     ahead_obtacle = lox2.readRange() / 10.0;
          //     delay(300);
          //   //}
          //   left_obtacle = lox1.readRange() / 10.0;
          //   right_obtacle = lox3.readRange() / 10.0;
          //   ahead_obtacle = lox2.readRange() / 10.0;
            lox1.startRangeContinuous(50);
            lox2.startRangeContinuous(50);
            lox3.startRangeContinuous(50);
            lox4.startRangeContinuous(50);
            // while (!lox2.isRangeComplete()) { Serial.println("Load sensor1"); }
            // while (!lox1.isRangeComplete()) { Serial.println("Load sensor2"); }
          // } else {
          //   lox1.startRangeContinuous(50);
          //   lox2.startRangeContinuous(50);
          //   lox3.startRangeContinuous(50);
          //   lox4.startRangeContinuous(50);
          //   while (!lox2.isRangeComplete()) { Serial.println("Load sensor1"); }
          //   while (!lox1.isRangeComplete()) { Serial.println("Load sensor2"); }
          //   turn_right();
          // }
        } else if (ahead_obtacle < 13 && ahead_obtacle2 < 13) {
          lox1.stopRangeContinuous();
          lox2.stopRangeContinuous();
          lox3.stopRangeContinuous();
          lox4.stopRangeContinuous();
          stop();
          delay(200);
          left_obtacle = lox1.readRange() / 10.0;
          right_obtacle = lox3.readRange() / 10.0;
          ahead_obtacle = lox2.readRange() / 10.0;
          ahead_obtacle2 = lox4.readRange() / 10.0;
          while (ahead_obtacle2 <= 7) {
            ahead_obtacle2 = lox4.readRange() / 10.0;
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);
            analogWrite(enB, 60 - 5);
            analogWrite(enA, 60);
          }
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            digitalWrite(in3, LOW);
            digitalWrite(in4, HIGH);
            analogWrite(enB, 0);
            analogWrite(enA, 0);
          //delay(200);
          if (right_obtacle < 25) {
            //turn_around();
            if (right_obtacle > left_obtacle) {
              while (ahead_obtacle < 20 || ahead_obtacle2 < 20) {
                digitalWrite(in1, HIGH);
                digitalWrite(in2, LOW);
                analogWrite(enB, 60);
                analogWrite(enA, 60);
                // delay(800);
                // analogWrite(enB, 80);
                // analogWrite(enA, 20);
                // delay(800);
                ahead_obtacle = lox2.readRange() / 10.0;
                ahead_obtacle2 = lox4.readRange() / 10.0;
              }
              digitalWrite(in1, LOW);
              digitalWrite(in2, HIGH);
            } else {
              while (ahead_obtacle < 20 || ahead_obtacle2 < 20) {
                digitalWrite(in3, HIGH);
                digitalWrite(in4, LOW);
                analogWrite(enB, 60);
                analogWrite(enA, 60);
                //delay(800);
                ahead_obtacle = lox2.readRange() / 10.0;
                ahead_obtacle2 = lox4.readRange() / 10.0;
              }
              digitalWrite(in3, LOW);
              digitalWrite(in4, HIGH);
            }
            left_obtacle = lox1.readRange() / 10.0;
            right_obtacle = lox3.readRange() / 10.0;
            ahead_obtacle = lox2.readRange() / 10.0;
            delay(200);
          } else {
            while (ahead_obtacle < 22 || ahead_obtacle2 < 22) {
              digitalWrite(in1, HIGH);
              digitalWrite(in2, LOW);
              analogWrite(enB, 60);
              analogWrite(enA, 60);
              ahead_obtacle = lox2.readRange() / 10.0;
              ahead_obtacle2 = lox4.readRange() / 10.0;
            }
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            left_obtacle = lox1.readRange() / 10.0;
            right_obtacle = lox3.readRange() / 10.0;
            ahead_obtacle = lox2.readRange() / 10.0;
            //delay(200);
          }
          lox1.startRangeContinuous(50);
          lox2.startRangeContinuous(50);
          lox3.startRangeContinuous(50);
          lox4.startRangeContinuous(50);
          // while (!lox2.isRangeComplete()) { Serial.println("Load sensor1"); }
          // while (!lox1.isRangeComplete()) { Serial.println("Load sensor2"); }
        }


        if (ahead_obtacle < 15) {  // it will stop at distance 7
          stop();
          delay(1000);
          // ahead_obtacle = lox2.readRange() / 10.0;
          // display.clearDisplay();
          // print_oled(String(ahead_obtacle), 0, 0);
          // delay(5000);
          if (right_obtacle > wall) {
            turn_right();
            //delay(1000);
          }
          if (left_obtacle > wall) {
            turn_left();
            //delay(1000);
          }
          if (right_obtacle < wall && left_obtacle < wall) {
            turn_around();
            //delay(1000);
          }
        }
      }
  }

  //print result
  Serial.print(output_follow);
  Serial.print("  ");
  Serial.print(right_obtacle);
  Serial.print("  ");
  Serial.print(ahead_obtacle);
  Serial.print("  ");
  Serial.print(left_obtacle);
  Serial.print("  ");
  Serial.print(ahead_obtacle2);
  Serial.print("  ");
  Serial.print(xung[0]);
  Serial.print("  ");
  Serial.println();
}

//read encoder

void readEncoder0() {
  int b = digitalRead(enb[0]);
  if (b > 0) {
    posi[0]--;
    xung[0]++;
  } else {
    posi[0]++;
    xung[0]--;
  }
}
void readEncoder1() {
  int b = digitalRead(enb[1]);
  if (b > 0) {
    posi[1]++;
    xung[1]++;
  } else {
    posi[1]--;
    xung[1]++;
  }
}


void print_oled(String text, int x, int y) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x, y);
  display.println(text);
  display.display();
}

void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  digitalWrite(SHT_LOX4, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  delay(10);
  // initing LOX1
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot left VL53L0X"));
    while (1)
      ;
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot ahead_left VL53L0X"));
    while (1)
      ;
  }

  digitalWrite(SHT_LOX3, HIGH);
  delay(10);
  //initing LOX3
  if (!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot right VL53L0X"));
    while (1)
      ;
  }

  digitalWrite(SHT_LOX4, HIGH);
  delay(10);
  //initing LOX4
  if (!lox4.begin(LOX4_ADDRESS)) {
    Serial.println(F("Failed to boot ahead_right VL53L0X"));
    while (1)
      ;
  }
}

void scan_button() {
  button_value = analogRead(button);
  if (button_value >= 420 && button_value <= 500) {
    increase = 1;
  } else {
    increase = 0;
  }

  if (button_value >= 530 && button_value <= 700) {
    stop_button = 1;
  } else {
    stop_button = 0;
  }

  if (button_value >= 1490 && button_value <= 1620) {
    decrease = 1;
  } else {
    decrease = 0;
  }

  if (button_value >= 1780 && button_value <= 1870) {
    choose = 1;
  } else {
    choose = 0;
  }
}

void resetSensor() {
  lox1.stopRangeContinuous();   // Stop continuous mode
  lox1.startRangeContinuous();  // Restart continuous mode
  lox2.stopRangeContinuous();   // Stop continuous mode
  lox2.startRangeContinuous();
  lox3.stopRangeContinuous();  // Stop continuous mode
  lox3.startRangeContinuous();
  lox4.stopRangeContinuous();  // Stop continuous mode
  lox4.startRangeContinuous();
}

void readUltrasonic() {
  unsigned long duration;
  digitalWrite(trig, 0);  // turn off trig pin
  delayMicroseconds(2);
  digitalWrite(trig, 1);  // turn on trig pin
  delayMicroseconds(5); 
  digitalWrite(trig, 0);  // turn off trig pin

  /* Tính toán thời gian */
  // calculate the High logic time at echo pin
  duration = pulseIn(echo, HIGH);
  ahead_obtacle3 = int(duration / 2 / 29.412);
}