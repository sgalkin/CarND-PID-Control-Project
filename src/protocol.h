#pragma once

#include <string>
#include "measurement.h"

struct WSProtocol {  
  static std::string getPayload(std::string message);
  static Observation getObservation(std::string payload);
  
  static std::string formatResponse();
  static std::string formatResponse(const Control& control);

  static std::string reset();
};
