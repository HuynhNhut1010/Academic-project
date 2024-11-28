void scan_maze() {
  display.clearDisplay();
  int way = 1;
  //right_obtacle = 0;
  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);
  lox3.rangingTest(&measure3, false);
  left_obtacle = measure1.RangeMilliMeter / 10.0;
  ahead_obtacle = measure2.RangeMilliMeter / 10.0;
  right_obtacle = measure3.RangeMilliMeter / 10.0;
  ahead_obtacle2 = measure4.RangeMilliMeter / 10.0;
  for (int i = 0; i <= 3; i++) {
    sensor[m][i] = i + position;
  }

  for (int ii = 0; ii <= 3; ii++) {
    if (sensor[m][ii] >= 5) {
      sensor[m][ii] = sensor[m][ii] % 4;
    }
    if (sensor[m][ii] <= 0) {
      sensor[m][ii] = 4 + sensor[m][ii];
    }
  }


  Serial.println("Sensor: ");
  for (int i = 0; i <= 3; i++) {
    Serial.print(sensor[m][i]);
    Serial.print("   ");
  }
  Serial.println();
  Serial.println("Maze: ");
  int check_maze[] = { 0, 1 };
  int flag = 0;
  maze[m][0] = maze[m - 1][sensor[m][1]];  //update position

  for (int i = 1; i <= m - 1; i++) {  //check turn_around
    if (maze[m][0] == maze[i][0]) {
      maze[i][5] = maze[i][5] + 1;
      for (int j = 0; j <= 5; j++) {
        maze[m][j] = maze[i][j];
      }
      repeat_position = 1;
      for (int j = 0; j <= 3; j++) {
        sensor[m][j] = sensor[i][j];
      }
    }
  }


  if (repeat_position != 1) {
    for (int i = 1; i <= m - 1; i++) {
      for (int ii = 0; ii <= 1; ii++) {
        if (maze[m][0] == maze[i][sensor[i][check_maze[ii]]]) {
          for (int k = 1; k <= 4; k++) {
            if (maze[i][k] > 0) {
              flag++;
            }
          }
          if (maze[i][0] == 1){
            flag++;
          }
          if (flag - maze[i][5] - 1 == 2) {
              maze[i][sensor[i][0]]++;
              maze[i][sensor[i][1]]++;
              flag = 0;
            } 
            else if (flag - maze[i][5] - 1 == 1) {
              if (maze[i][sensor[i][0]] != 0 && maze[i][sensor[i][0]] == maze[m][0]){    // only update right position
                maze[i][sensor[i][0]]++;
                flag = 0;
              }
              // if (maze[i][sensor[i][1]] != 0){
              //   maze[i][sensor[i][1]]++;
              //   flag = 0;
              // }

            }
          }
        }
      }
    if (left_obtacle < wall) {
      maze[m][sensor[m][2]] = 0;
    } else {
      maze[m][sensor[m][2]] = maze[m][0] + 1;
      way++;
    }


    if (ahead_obtacle < 30 && ahead_obtacle2 <30) {
      maze[m][sensor[m][1]] = 0;
    } else {
      maze[m][sensor[m][1]] = maze[m][0] + way;
      way++;
    }


    if (right_obtacle < wall) {
      maze[m][sensor[m][0]] = 0;
    } else {
      maze[m][sensor[m][0]] = maze[m][0] + way;
    }

    maze[m][sensor[m][3]] = maze[m - 1][0];  //update position behind
    if (dead_end == 0) {
      maze[m][5] = 1;
    } else {
      maze[m][5] = 6;
      dead_end = 0;
    }
  }
  route_i++;
  repeat_position = 0;
}