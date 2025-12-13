//mbed library
#include <mbed.h>
#include <cmath>

#include "RoboHW.h"
#include "RoboNav.h"

//to use the mbed library types without writing mbed::DigitalOut every time.
using namespace mbed;

unsigned long last_print = 0;


void callibration(){
  
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
}

char cmd = 0;

void setup() {
  Serial.begin(9600);
  hw_init();
  delay(200);
}



void loop() {
  callibration();


  if(Serial.available() > 0){
    cmd = (char)Serial.read();
  }

  if(!mvng_frwrd && !trnng){
    switch (cmd){
      case 'f': move_frwrd(20, 0.4); break;
      case 'l': turn(-90, 0.4); break;
      case 'r': turn(90, 0.4); break;
      case 'b': turn(180, 0.4); break;
      case 's': stop(); break;
    }
    cmd = 0;
  }

  if (millis() - last_print > 300) {
  last_print = millis();

  Serial.print("enc:");
  Serial.print(enca);
  Serial.print(",");
  Serial.print(encb);

  Serial.print("  us:");
  Serial.print(read_us());

  Serial.print("  L:");
  Serial.print(read_l_ir());
  Serial.print("  R:");
  Serial.print(read_r_ir());

  Serial.print("  mv:");
  Serial.print(mvng_frwrd);
  Serial.print(" tr:");
  Serial.println(trnng);
}
}
