#include "diffdrive_arduino/wheel.h"

#include <cmath>


Wheel::Wheel(const std::string &wheel_name, int counts_per_rev)
{
  setup(wheel_name, counts_per_rev);
}


void Wheel::setup(const std::string &wheel_name, int counts_per_rev)
{
  name = wheel_name;
  rads_per_count = (2*M_PI)/counts_per_rev;
}

double Wheel::calcEncAngle()
{ 
  enc = enc * 74;
  return enc * rads_per_count;
}