#include <iostream>
#include <memory>
#include <exception>
#include <future>
#include <iterator>
#include <functional>
#include <uWS/uWS.h>

#include "protocol.h"
#include "application.h"
#include "twiddle.h"

namespace {
  using App = Application<WSProtocol>;

  //constexpr double known_best = 500.;
  constexpr size_t skip = 100;
  constexpr size_t count = 3000;

  // Twiddle parameters
  constexpr size_t N = 3;
  constexpr double step = 0.1;
  constexpr double termination_threshold = 1e-2;

  // 3.89635 0 4.2803
  // Fail-safe values from the lesson
  constexpr double Kp = 0.2;
  constexpr double Ki = 0.004;//4;
  constexpr double Kd = 3.;

  constexpr double throttle = 0.5; // 0.3

  using P = Parameters<N>;
  using E = std::tuple<P, double, std::promise<double>>;
}

std::unique_ptr<App> create() {
  std::unique_ptr<Controller> angle{new PID(Kp, Ki, Kd)};
  std::unique_ptr<Controller> speed{new Constant(1. - throttle)};
  return std::unique_ptr<App>(new App{std::move(angle), std::move(speed)});
}

std::unique_ptr<App> create(const P& p, double best, std::promise<double>& score) {
  std::copy(begin(p), end(p), std::ostream_iterator<P::value_type>(std::cerr << "---> ", " "));
  std::unique_ptr<Controller> angle{
    new ScoreThreshold(
      std::unique_ptr<Controller>(new PID(p[0], p[1], p[2])),
      [&score](const ScoreThreshold*, double s) { score.set_value(s); },
      best, std::numeric_limits<double>::infinity(),
      skip, count)
  };
  std::unique_ptr<Controller> speed{
    new Constant(1. - throttle)
//    new PID(p[3], 0, p[4])
  };
  return std::unique_ptr<App>(new App{std::move(angle), std::move(speed)});
}

std::unique_ptr<App> create(E& e) {
  return create(std::get<0>(e), std::get<1>(e), std::get<2>(e));
}

std::unique_ptr<App> create(void* ud) { return ud ? create(*(E*)ud) : create(); }

double evaluate(uWS::Group<uWS::SERVER>& ws, P p, double b) {
  using namespace std::placeholders;
  if(std::any_of(begin(p), end(p), std::bind(std::less<double>(), _1, 0)))
    return std::numeric_limits<double>::infinity();
  
  std::promise<double> pscore;
  auto fscore = pscore.get_future();
  
  std::copy(begin(p), end(p), std::ostream_iterator<double>(std::cerr << "=> ", " "));
  E pb{std::move(p), b, std::move(pscore)};
  ws.setUserData((void*)&pb);
  auto reset = WSProtocol::reset();
  ws.broadcast(reset.data(), reset.length(), uWS::OpCode::TEXT);

  auto s = fscore.get();
  std::cerr << "<= " << s << "\n";
  return s;
}

void twiddle(uWS::Group<uWS::SERVER>& ws) { 
  using namespace std::placeholders;
  std::cerr << "Twiddling " << N << " parameters\n";
  
  auto result = twiddle<P>(
    std::bind(evaluate, std::ref(ws), _1, _2),
    std::bind(threshold<P>, _1, termination_threshold),
    step,
    {{1+Kp, Ki, Kd}});
  
  std::copy(begin(result), end(result),
            std::ostream_iterator<double>(std::cout << "Parameters: ", " "));
  std::cout << std::endl;
}

int main() {
  uWS::Hub h;
  std::unique_ptr<App> app;

  h.onMessage([&app](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode) {
    try {
      auto message = std::string(data, length);
      auto response = app->ProcessMessage(std::move(message));
      ws.send(response.data(), response.length(), uWS::OpCode::TEXT);
    } catch(std::runtime_error& /*e*/) {
      //std::cerr << "Error while processing message: " << e.what() << std::endl;
    }    
  });

  h.onConnection([&h, &app](uWS::WebSocket<uWS::SERVER>, uWS::HttpRequest) {
    app = create(static_cast<uWS::Group<uWS::SERVER>>(h).getUserData());
  });

  h.onDisconnection([&app](uWS::WebSocket<uWS::SERVER>, int, char*, size_t) {
    app.reset();
  });

  constexpr int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  auto dummy = std::async(std::launch::async, [&h] { twiddle(h); });

  h.run();
  return 0;
}
