void analyze_map() {
  int r = 0;
  int a = 1;
  route[0][0] = 0;
  for (int i = 1; i <= route_i; i++) {
    for (int k = 1; k <= 4; k++) {
      if (maze[i][k] > 0) {
        r++;
      }
      if (i==1) {
        r++;
      }
    }
    if (maze[i][5] < r) {
      route[a][0] = maze[i][0];
      if(route[a][0] != route[a - 1][0]){
        a++;
      }
    }
    r = 0;
  }
}
void solve_maze() {
  route[0][0] = 0;
  next_point = route[check_route][0];
  Serial.print("check_route:  ");
  Serial.println(check_route);
  Serial.print("next_point:  ");
  Serial.println(next_point);
  Serial.print("recent_position:  ");
  Serial.println(recent_position);
  Serial.print("position:  ");
  Serial.println(position);
  for (int i = 0; i < route_i; i++) {
    if (recent_position == maze[i][0]) {
      for (int ii = 1; ii <= 4; ii++) {
        if (next_point == maze[i][ii]) {
          for (int iii = 0; iii <= 3; iii++) {
            if (ii == sensor[i][iii]){
              turn = iii;
              Serial.print("turn: ");
              Serial.println(turn);
            }

        }
      }
    }
  }
  }
  display.clearDisplay();
  print_oled("Recent point:", 0, 0);
  print_oled(String(recent_position), 80, 0);
  print_oled("Next point:", 0, 10);
  print_oled(String(next_point), 80, 10);
}