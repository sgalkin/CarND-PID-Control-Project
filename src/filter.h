#pragma once

#include <chrono>
#include "measurement.h"

struct Filter {
  virtual ~Filter() {}
  virtual Observation filter(Observation o) = 0;
};

class DistanceGuard : public Filter {
public:
  Observation filter(Observation o) override;
private:
  std::chrono::high_resolution_clock::time_point current_{
    std::chrono::high_resolution_clock::now()
  };
  double distance_{0};
};

class VelocityGuard : public Filter {
public:
  Observation filter(Observation o) override;
private:
  size_t failed_step_{0};
};

struct Recorder : public Filter {
  Observation filter(Observation o) override;
};
