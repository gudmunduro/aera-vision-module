#pragma once
#include <vector>
#include "libpixyusb2.h"
#include <memory>

int init();
void set_lamp(int upper, int lower);
int stop();
int get_raw_frame(uint8_t **bayerFrame);