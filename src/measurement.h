#pragma once

#include <algorithm>

struct Observation {
  Observation(size_t count, double cte, double speed, double angle) :
    count(count), cte(cte), speed(speed), angle(angle) {}

  size_t count;
  double cte;
  double speed;
  double angle;
};

struct Control {
  Control(double angle, double throttle) :
    angle(std::max(-1., std::min(1., angle))),
    throttle(std::max(-1e-2, std::min(1., throttle)))
  {}
  
  const double angle;  // value is in range [-1., 1.]
  const double throttle;
};
