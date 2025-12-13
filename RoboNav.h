#pragma once
#include <mbed.h>

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

extern robostate robot;

void move(robostate &r);

void turn_right(robostate &r);
void turn_left(robostate &r);
void turn_away(robostate &r);
