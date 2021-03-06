#pragma once

#include <cmath>
#include <string>
#include <memory>
#include "measurement.h"
#include "controller.h"

template<typename Protocol, typename Parser, typename Processor>
class Application {
  static constexpr double brake_threshold = 1.05;
  
public:
  explicit Application(
    std::unique_ptr<Controller> angle,
    std::unique_ptr<Controller> speed)
    : angle_(std::move(angle))
    , speed_(std::move(speed))
  {}

  std::string ProcessMessage(std::string message) {
    return ProcessPayload(Protocol::getPayload(std::move(message)));
  }

private:
  std::string ProcessPayload(std::string payload) {
    if(payload.empty()) {
      return Protocol::formatResponse();
    }
    return ProcessObservation(Protocol::getObservation(std::move(payload), parser_, processor_));
  }
  
  std::string ProcessObservation(Observation observation) {
    Control c{
      angle_->eval(observation.cte),
      speed_->eval(2*(fabs(observation.cte) - brake_threshold))
    };
    return Protocol::formatResponse(c);
  }

private:
  Parser parser_;
  Processor processor_;
  std::unique_ptr<Controller> angle_;
  std::unique_ptr<Controller> speed_;
};
