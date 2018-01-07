#include <iostream>
#include <memory>
#include <exception>
#include <uWS/uWS.h>
#include "ProgramOptions.hxx"
#include "configuration.h"

namespace {
po::parser parser() {
  po::parser parser;
  parser["help"].abbreviation('h').description("print this help screen")
    .callback([&parser]{
        std::cout << parser << '\n';
        exit(1);
    });
  
  parser["twiddle"].abbreviation('t').description("enable twiddle mode").type(po::void_)
    .callback([]{
        std::cerr << "Twiddle mode enabled. All other options will be ignored.\n";
    });
    
  parser["sKp"].abbreviation('p').description("steering angle PID Kp").type(po::f32);
  parser["sKi"].abbreviation('i').description("steering angle PID Ki").type(po::f32);
  parser["sKd"].abbreviation('d').description("steering angle PID Kd").type(po::f32);

  parser["tKp"].description("throttle PID Kp").type(po::f32);
  parser["tKi"].description("throttle PID Ki").type(po::f32);
  parser["tKd"].description("throttle PID Kd").type(po::f32);

  parser["tC"].abbreviation('c').description("constant throttle").type(po::f32);

  return parser;
}
  
template<typename Configuration>
int run(po::parser parser) {
  if(!Configuration::validate(parser)) {
    std::cerr << parser << std::endl;
    return -1;
  }
  
  uWS::Hub h;
  std::unique_ptr<typename Configuration::Application> app;

  h.onMessage([&app](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode) {
    try {
      auto message = std::string(data, length);
      auto response = app->ProcessMessage(std::move(message));
      ws.send(response.data(), response.length(), uWS::OpCode::TEXT);
    } catch(std::runtime_error& /*e*/) {
      //std::cerr << "Error while processing message: " << e.what() << std::endl;
    }    
  });

  h.onConnection([&parser, &h, &app](uWS::WebSocket<uWS::SERVER>, uWS::HttpRequest) {
    app = Configuration::create(parser, h);
  });

  h.onDisconnection([&app](uWS::WebSocket<uWS::SERVER>, int, char*, size_t) {
    app.reset();
  });

  constexpr int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  Configuration::extra(h, app);
  h.run();
  return 0;
}
}

int main(int argc, char* argv[]) {
  auto parser = ::parser();
  if(!parser(argc, argv)) {
    std::cerr << parser << "\n";
    return -1;
  }

  if(parser["twiddle"].available())
    return run<Twiddle>(std::move(parser));
  else
    return run<Workload>(std::move(parser));
  
  return 0;
}
