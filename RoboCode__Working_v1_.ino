//mbed library
#include <mbed.h>
#include <cmath>

//to use the mbed library types without writing mbed::DigitalOut every time.
using namespace mbed;
//pwm control for the left motor (motor A).
PwmOut pwma(P0_27);
//pwm control for motor A.
DigitalOut dira(P0_4);
//direction control for right motor (motor B).
PwmOut pwmb(P1_2);
//direction control for motor B.
DigitalOut dirb(P0_5);
//Encoder pins (Distance control) for motors A and B
InterruptIn tica(P1_11);
InterruptIn ticb(P1_12);

//front ultrasonic sensor pins
DigitalInOut us_front(P0_23);

//timer for us sensor
Timer us_t;


I2C i2c(P0_31, P0_2);


//left/right motor speed variables
float l_speed = 0;
float r_speed = 0;

//left/right encoder count variables
volatile long int enca = 0;
volatile long int encb = 0;

//left/rightmotor directions
int l_dir = +1;
int r_dir = +1;

//distance per encoder tick (cm)
float dpt = 0.048;

//distance moved by each motor
float disa = 0;
float disb = 0;

//base speed
float basespeed = 0.4;

bool mvng_frwrd = false;
bool trnng = false;

//target tick count
long int tt = 0;

float speed = 0.0;

float dis = 0;

float k = 0.025;

float lcs = 0.0;
float rcs = 0.0;

//tickss per degree
float tpd = 3.15;

float dis_wall = 0;


float us_r = 0;


enum orientation {
  up = 0,
  rt = 1,
  dn = 2,
  lt = 3,
};


struct robostate {
  int x;
  int y;
  orientation o;
};

robostate robot = { 0, 0, up };




//function for moving robot at certain speed and direction
void wheels(float l_speed, float r_speed) {

  //left motor direcetion: if speed is positive, move forward
  if (l_speed > 0) {
    dira = 1;
    l_dir = +1;
    //otherwise, move backward
  } else if (l_speed < 0) {
    dira = 0;
    l_dir = -1;
  } else {
    l_dir = 0;
  }
  //same for right motor
  if (r_speed > 0) {
    dirb = 1;
    r_dir = +1;
  } else if (r_speed < 0) {
    dirb = 0;
    r_dir = -1;
  } else {
    r_dir = 0;
  }
  //using the absolute value of motor speeds, rotate at that speed
  pwma.write(fabs(l_speed));
  pwmb.write(fabs(r_speed));
}


void l_cnt() {
  enca = enca + l_dir;
}

void r_cnt() {
  encb = encb + r_dir;
}

void distance() {
  disa = abs(enca) * dpt;
  disb = abs(encb) * dpt;
}

void stop() {
  wheels(0, 0);
}


void move_frwrd(float dis, float spd) {
  enca = 0;
  encb = 0;
  tt = dis / dpt;
  speed = spd;
  mvng_frwrd = true;
}

void move(robostate &r) {
  move_frwrd(10, 0.4);  // DISTANCE VALUE NEEDS CHANGING

  if (r.o == up) {
    r.y = r.y + 1;
  }
  if (r.o == dn) {
    r.y = r.y - 1;
  }
  if (r.o == rt) {
    r.x = r.x + 1;
  }
  if (r.o == lt) {
    r.x = r.x - 1;
  }
}

void keepstraight() {
  //to keep the robot straight
  float diff = abs(enca) - abs(encb);
  lcs = speed - k * diff;
  rcs = speed + k * diff;
  if (lcs < 0) {
    lcs = 0;
  } else if (lcs > 1) {
    lcs = 1;
  } else {
    lcs = lcs;
  }
  if (rcs < 0) {
    rcs = 0;
  } else if (rcs > 1) {
    rcs = 1;
  } else {
    rcs = rcs;
  }
}


void turn(float deg, float spd) {
  enca = 0;
  encb = 0;
  tt = tpd * deg;
  speed = spd;
  trnng = true;
}

void turn_right(robostate &r) {
  turn(90, 0.4);

  r.o = (orientation)((r.o + 1) % 4);
}

void turn_left(robostate &r) {
  turn(-90, 0.4);

  r.o = (orientation)((r.o + 3) % 4);
}

void turn_away(robostate &r) {
  turn(180, 0.4);

  r.o = (orientation)((r.o + 2) % 4);
}



float read_us() {
  us_front.output();
  us_t.reset();
  us_front = 0;
  wait_us(5);
  us_front = 1;
  wait_us(10);
  us_front = 0;
  us_front.input();
  us_t.reset();
  us_t.start();
  while (us_front.read() == 0 && us_t.read_us() < 30000) {}
  if (us_front.read() == 0) {
    return -404;
  }
  us_t.reset();
  us_t.start();
  while (us_front.read() == 1 && us_t.read_us() < 30000) {}
  us_t.stop();
  float pw_us = us_t.read_us();
  us_r = pw_us / 58;
  return us_r;
}


float read_l_ir() {
  const char mux_cmd = 0x02;
  const char mux_addr = 0xEE;
  i2c.write(mux_addr, &mux_cmd, 1);
  char cmd[2];
  cmd[0] = 0x5E;
  cmd[1] = 0x00;
  i2c.write(0x80, cmd, 1);
  wait_us(1000);
  i2c.read(0x80, cmd, 2);
  unsigned int raw_l = (cmd[0] << 4) | (cmd[1] >> 4);
  float l_wall_dis = raw_l / 64.0f;
  return l_wall_dis;
}

float read_r_ir() {
  const char mux_cmd = 0x04;
  const char mux_addr = 0xEE;
  i2c.write(mux_addr, &mux_cmd, 1);
  char cmd[2];
  cmd[0] = 0x5E;
  cmd[1] = 0x00;
  i2c.write(0x80, cmd, 1);
  wait_us(1000);
  i2c.read(0x80, cmd, 2);
  unsigned int raw_r = (cmd[0] << 4) | (cmd[1] >> 4);
  float r_wall_dis = raw_r / 64.0f;
  return r_wall_dis;
}



void setup() {


  Serial.begin(9600);

  //period for pwm
  pwma.period_us(2000);
  pwmb.period_us(2000);
  tica.rise(&l_cnt);
  ticb.rise(&r_cnt);
  i2c.frequency(100000);
}



void loop() {
  long int avg_enc = (labs(enca) + labs(encb)) / 2;

  if (mvng_frwrd) {
    if (avg_enc < tt) {
      keepstraight();
      wheels(lcs, rcs);
    } else {
      stop();
      mvng_frwrd = false;
    }
  } else if (trnng) {
    if (tt > 0 && avg_enc < tt) {
      keepstraight();
      wheels(lcs, -rcs);
    } else if (tt < 0 && avg_enc < labs(tt)) {
      keepstraight();
      wheels(-lcs, rcs);
    } else {
      stop();
      trnng = false;
    }
  }

  if (!mvng_frwrd && !trnng) {
  }
}
