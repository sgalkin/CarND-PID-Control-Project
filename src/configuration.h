#pragma once

#include <memory>
#include <uWS/uWS.h>
#include "ProgramOptions.hxx"

#include "protocol.h"
#include "filter.h"
#include "processor.h"
#include "application.h"

template<typename Processor>
using DefaultApplication = ::Application<WSProtocol, DefaultParser, Processor>;


struct Twiddle {
  using Application = DefaultApplication<Processor<VelocityGuard, DistanceGuard>>;
  
  static bool validate(po::parser&) { return true; }
  static std::unique_ptr<Application> create(po::parser&, const uWS::Hub& );
  static void extra(uWS::Hub& h, const std::unique_ptr<Application>&);
};


struct Workload {
  using Application = DefaultApplication<Processor<Recorder>>;

  static bool validate(po::parser& p);
  static std::unique_ptr<Application> create(po::parser& p, const uWS::Hub& h);
  static void extra(const uWS::Hub&, const std::unique_ptr<Application>&) {};
};
