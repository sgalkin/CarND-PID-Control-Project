#include "filter.h"

#include <iostream>
#include <chrono>
#include "measurement.h"

namespace {
  // able to reach with 0.2;0.004;3 at 0.5 constant throttle
  constexpr double distance_threshold = 0.75;
  constexpr size_t distance_grace = 1800;

  constexpr double speed_threshold = 10;
  constexpr size_t speed_threshold_grace = 100;

  constexpr size_t recorder_skip = 50;
  constexpr size_t recorder_dump = 500;
}


Observation DistanceGuard::filter(Observation o) {
  auto now = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(now - current_);
  current_ = std::move(now);
  distance_ += duration.count()*o.speed/3600.0;
  return Observation{
    o.count,
    o.cte / (o.count < distance_grace || distance_ >= distance_threshold),
    o.speed,
    o.angle
  };
}


Observation VelocityGuard::filter(Observation o) {
  ++failed_step_ *= (o.speed < speed_threshold);
  return Observation{
    o.count,
    // scale cte up to infinity if speed threshold is not passed
    // this helps twiddle to finish simulation earlier
    o.cte / (failed_step_ <= speed_threshold_grace * (o.speed > 0)),
    o.speed,
    o.angle
  };
}


Observation Recorder::filter(Observation o) {
  if(o.count > recorder_skip && o.count - recorder_skip < recorder_dump)
    std::cerr << o.count << " " << o.cte << "\n";
  return o;
}
