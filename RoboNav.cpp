#include "RoboHW.h"
#include "RoboNav.h"
#include "RoboMap.h"

float front_dis;
float right_dis;
float left_dis;

Pose robot = { 0, 0, up };

// threshold for the front and side sensors
float side_sensor_th = 15; //cm
float front_sensor_th = 5;//cm

Robostates state = stt_forward;

unsigned long last_print = 0;
unsigned long last_pid = 0;
float steer = 0.0;
bool traversing = false;
int  nodehits = 0;
unsigned long lastsns;
int crnt_node = 0;
int chosen_dir = -1;
float last_l_val = 0;
float last_r_val = 0;

void callibration(){
  long int avg_enc = (labs(enca) + labs(encb))/2;

  if(mvng_frwrd){
    if(read_us() < front_sensor_th && read_us() > 2){
      Serial.print("Real Wall stop at: "); Serial.println(read_us());
      stop();
      mvng_frwrd = false;
      return;
    }
    keepstraight();
    if(millis()-last_pid > 50){
      steer = centering();
      last_pid = millis();
    }
    if(avg_enc < tt){
      wheels(lcs - steer, rcs + steer);
    } else{
      stop();
      mvng_frwrd = false;
      steer = 0;
    }
  } else if(trnng){
    if(tt > 0 && avg_enc < tt){
      wheels(lcs, -rcs);
    }else if(tt < 0 && avg_enc < -tt){
      wheels(-lcs, rcs);
    }else{
      stop();
      trnng = false;
    }
  }
}

void wait_until_done(){
  while(mvng_frwrd || trnng){
    callibration();
    wait_us(100);
  }
}

void turn_right(Pose &r) {
  Serial.println("Action: Turning Right");
  turn(90,0.4);
  wait_until_done();
  r.o = (orientation)((r.o + 1) % 4);
}

void turn_left(Pose &r) {
  Serial.println("Action: Turning Left");
  turn(-90, 0.4);
  wait_until_done();
  r.o = (orientation)((r.o + 3) % 4);
}

void turn_away(Pose &r) {
  Serial.println("Action: Turning Back (180)");
  turn(180, 0.4);
  wait_until_done();
  r.o = (orientation)((r.o + 2) % 4);
}

void avoid_obstacle(){
  if((read_fl_ir() < 10 && read_fr_ir() < 10) || read_us() < 20){
    return;
  }else if(read_fl_ir() < 10){
    turn(30, 0.3);
    wait_until_done();
    move_frwrd(5, b_speed);
    wait_until_done();
    turn(-30, 0.3);
    wait_until_done();
    return;
  }else if(read_fr_ir() < 10){
    turn(-30, 0.3);
    wait_until_done();
    move_frwrd(5, b_speed);
    wait_until_done();
    turn(30, 0.3);
    wait_until_done();
    return;
  }
}

void traverse(Pose &r){
  // to move in a straight line in the centre of a path
  keepstraight(); //to move in a straight line
  float steer = centering();  // to determine how much to steer to be in the centre of the path
  wheels(lcs - steer, rcs + steer); // to move straight in the centre
  //to only sense every  60s
  unsigned long now = millis();
  if(now - lastsns < 60UL){
    return;
  }
  lastsns = now;
  //read all sensors and save them to these variable names
  float f_val = read_us(); //front sensor value
  float r_val = read_r_ir(); //right sensor value
  float l_val = read_l_ir(); //left sensor value
  float fr_val = read_fr_ir();
  float fl_val = read_fl_ir();
  static long prev_enca = 0;
  static long prev_encb = 0;
  static float robotX_mm = 0.0f;
  static float robotY_mm = 0.0f;
  float robotYaw_rad = (robot.o == up ? 0.0f) : (robot.o == rt ? m_pi_2 : (robot.o == dn ? m_pi : 3.0f*m_pi_2))
  long cur_enca = enca;
  long cur_encb = encb;
  long d_ticks_l = cur_enca - prev_enca;
  long d_ticks_r = cur_encb - prev_encb;

  prev_enca = cur_enca
  prev_encb = cur_encb

  if(d_ticks_l == 0 && d_ticks_r == 0){
    float robotX_loc = robotX_mm;
    float robotY_loc = robotY_mm;
    float robotYaw_loc = robotYaw_rad;
  }else{
    float mm_per_tick = dpt * 10f;
    float dl = (float)d_ticks_l * mm_per_tick;
    float dr = (float)d_ticks_r * mm_per_tick;
    float d_centre = 0.5f * (dl + dr);
    float d_theta_deg = (float)(d_ticks_r - d_ticks_l) / (float)tpd;
    float d_theta = d_theta_deg * (m_pi/180.0f);
    float mid_yaw = robotYaw_rad + 0.5f  * d_theta;
    robotX_mm += d_centre * cosf(mid_yaw);
    robotY_mm += d_centre * sinf(mid_yaw);
    robotYaw_rad +=d_theta;
    if(robotYaw_rad  > m_pi){
      robotYaw_rad -= 2.0f * m_pi;
    }else if(robotYaw_rad <= -m_pi){
      robotYaw += 2.0f * m_pi
    }
  }
  float robotX_loc = robotX_mm;
  float robotY_loc = robotY_mm;
  float robotYaw_loc = robotYaw_rad;
  grid_update_from_sensor(robotX_loc, robotY_loc, robotYaw_loc, 120.0f, 0.0f, 0.0f, f_val, 1000.0f);
  grid_update_from_sensor(robotX_loc, robotY_loc, robotYaw_loc, 100.0f, 25.0f, m_pi/3.0f, l_val, 300.0f);
  grid_update_from_sensor(robotX_loc, robotY_loc, robotYaw_loc, 100.0f, -25.0f, -m_pi/3.0f, r_val, 300.0f);
  grid_update_from_sensor(robotX_loc, robotY_loc, robotYaw_loc, 100.0f, 25.0f, m_pi/6.0f, fl_val, 300.0f);
  grid_update_from_sensor(robotX_loc, robotY_loc, robotYaw_loc, 100.0f, -25.0f, -m_pi/6.0f, fr_val, 300.0f);

  //to determine if there are walls withing the thresholds
  bool front = (f_val > front_sensor_th); //for wall in front
  bool right = (r_val > side_sensor_th); //for wall on the right
  bool left = (l_val > side_sensor_th); //for wall on the left
  //detect the presence of a node, 
  bool node_detected = ((!front) && f_val > 2) ||
                       (left && l_val > 2 && avg_enc > 180) || 
                       (right && r_val > 2 && avg_enc > 180);
  
  if(node_detected){
    nodehits++;
  }else{
    nodehits = 0;
  }

  if(nodehits > 2){
    Serial.println(">>> NODE DETECTED <<<");
    stop();
    wait_until_done();
    move_frwrd(22,b_speed);
    wait_until_done();
    traversing = false;
    nodehits = 0;
    return;
  }
}

//wtg : where to go
void wtg(){
  chosen_dir = -1;
  for(int i = 0; i <= 3; i++){
    if(nodes[crnt_node].open[i] == true && nodes[crnt_node].explored[i] == false){
      chosen_dir = i;
      break;
    }
  }  
  if(chosen_dir != -1){
    nodes[crnt_node].explored[chosen_dir] = true;
  }
  if(chosen_dir == -1 && nodes[crnt_node].parent != -1){
      chosen_dir = nodes[crnt_node].parent_dir;
  }
  Serial.print("WTG: Chosen direction index is "); Serial.println(chosen_dir);
}

void fsm(Pose &r){
  Serial.println("--- Entering FSM ---");
  walls(r);
  wtg();
  new_node();
  if(chosen_dir == -1){
    state = stt_stop;
    return;
  }
  int delta = ((chosen_dir - r.o + 4) %4);
  switch (delta){
    case 0: state = stt_forward;    Serial.println("State: Forward"); break;
    case 1: state = stt_turn_right; Serial.println("State: Turn Right"); break;
    case 2: state = stt_turn_back;  Serial.println("State: Turn Back"); break;
    case 3: state = stt_turn_left;  Serial.println("State: Turn Left"); break;
  }
  switch (state){
    case stt_forward:
      depart_node = crnt_node;
      depart_dir = chosen_dir;
      traversing = true;
      break;
    case stt_stop:
      Serial.println("State: Stop (Finished)");
      stop();
      break;
    case stt_turn_right: 
      turn_right(r);
      Serial.println("rstt");
      traversing = true;
      break;
    case stt_turn_left:  
      turn_left(r);
      Serial.println("lstt");
      traversing = true;
      break;
    case stt_turn_back:
      turn_away(r);
      Serial.println("astt");
      traversing = true;
      break;
  }
}
