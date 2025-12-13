#include "RoboHW.h"
#include "RoboNav.h"


robostate robot = { 0, 0, up };


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

