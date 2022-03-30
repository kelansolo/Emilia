switch (state) {
  case 40:
    float stair_width = 20; // cm
    int arm_wait = 10; //s
    int n_stairs = 5;
    int arm_speed = 645;
    int line = 0;

    for (size_t i = 0; i < n_stairs; i++) {
      if (i==1) {
        stair_width = 10
      }
      int line = 0;
      snprintf(lines[line++], MAX_LEN, "edgel=0,vel= 0.2 white=1: dist= %.2f",stair_width);
      snprintf(lines[line++], MAX_LEN, "vel=0:time=1");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-800, vservo=%i :time=%i",arm_speed,arm_wait);
      snprintf(lines[line++], MAX_LEN, "vel=0.2:tilt>0.1");
      snprintf(lines[line++], MAX_LEN, "vel=0:time=1");
      snprintf(lines[line++], MAX_LEN, "vel=0.2:dist=0.07");
      snprintf(lines[line++], MAX_LEN, "vel=0:time=1");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=500, vservo=%i:time=%i", arm_speed, arm_wait);
      snprintf(lines[line++], MAX_LEN, "vel=0.2:tilt<0.15,dist=0.3");
      snprintf(lines[line++], MAX_LEN, "vel=-0.2:dist=%.2f", stair_width);
      snprintf(lines[line++], MAX_LEN, "vel=0:time = 2");
      sendAndActivateSnippet(lines, line);
    }
}
