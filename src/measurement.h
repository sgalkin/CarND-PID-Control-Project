#pragma once

struct Observation {
  Observation(double cte, double speed, double angle) :
    cte(cte), speed(speed), angle(angle) {}

  const double cte;
  const double speed;
  const double angle;
};

struct Control {
  Control(double angle, double throttle) :
    angle(std::max(-1., std::min(1., angle))),
    throttle(std::max(0., std::min(1., 1. - throttle)))
  {}
  
  const double angle;  // value is in range [-1., 1.]
  const double throttle;
};
