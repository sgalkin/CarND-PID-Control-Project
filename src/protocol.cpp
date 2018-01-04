#include "protocol.h"

#include <type_traits>
#include <iterator>
#include <exception>

#include "json.hpp"
//#include "measurement.h"

namespace {
bool checkHeader(const std::string& message) {
  // "42" at the start of the message means there's a websocket message event.
  // The 4 signifies a websocket message. The 2 signifies a websocket event
  return message.length() > 2 && strncmp(message.data(), "42", 2) == 0;
}
}

// for convenience
using json = nlohmann::json;

std::string WSProtocol::getPayload(std::string s) {
  if (!checkHeader(s)) {
    throw std::runtime_error("Unexpected message header");  
  }

  // Checks if the SocketIO event has JSON data.
  // If there is data the JSON object in string format will be returned,
  // else the empty string "" will be returned.
  auto b1 = s.find_first_of("["), b2 = s.find_last_of("]");
  if (s.find("null") == std::string::npos &&
      b1 != std::string::npos &&
      b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

Observation WSProtocol::getObservation(std::string s) {
  auto j = json::parse(s);
  if (j[0].get<std::string>() != "telemetry") {
    throw std::runtime_error("Unexpected event type");
  }
  auto& data = j[1];
  return Observation{
    std::stod(data["cte"].get<std::string>()),
    std::stod(data["speed"].get<std::string>()),
    std::stod(data["steering_angle"].get<std::string>())};
}

std::string WSProtocol::formatResponse() {
  static const std::string manual = "42[\"manual\",{}]";
  return manual;
}

std::string WSProtocol::reset() {
  static const std::string reset = "42[\"reset\",{}]";
  return reset;
}

std::string WSProtocol::formatResponse(const Control& control) {
  json msgJson;
  msgJson["steering_angle"] = control.angle;
  msgJson["throttle"] = control.throttle;
  return "42[\"steer\"," + msgJson.dump() + "]";
}
