#pragma once

#include <string>
#include "json.hpp"
#include "measurement.h"

class DefaultParser {
public:
  Observation parse(const nlohmann::json& object);
  
private:
  size_t count{0};
};

struct WSProtocol {  
  static std::string getPayload(std::string request);

  template<typename Parser, typename Processor>
  static Observation getObservation(std::string message, Parser& parser, Processor& processor) {
    auto j = nlohmann::json::parse(message);
    if (j[0].get<std::string>() != "telemetry") {
      throw std::runtime_error("Unexpected event type");
    }
    return processor.process(parser.parse(j[1]));
  }
  
  static std::string formatResponse();  
  static std::string formatResponse(const Control& control);

  static std::string reset();
};
