#include "bridge.h"
#include <iostream>

Pixy2 pixy_instance;

int init()
{
  return pixy_instance.init();
}

void set_lamp (int upper, int lower)
{
  pixy_instance.setLamp (upper, lower);
}

int stop() {
  return pixy_instance.m_link.stop();
}

int get_raw_frame(uint8_t **bayerFrame) {
  return pixy_instance.m_link.getRawFrame(bayerFrame);
}