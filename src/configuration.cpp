#include "configuration.h"

#include <thread>
#include <iostream>
#include <functional>
#include <future>
#include <limits>

#include "controller.h"
#include "twiddle.h"

// Twiddle tunables
namespace {
constexpr size_t measurement_skip = 200;
constexpr size_t measurement_count = 1800;

// Twiddle parameters
constexpr size_t N = 5;
constexpr double twiddle_step = 0.1;
constexpr double termination_threshold = 1e-4;

// Twiddle steering initial state (values from the lesson)
constexpr double Kp = 0.2;
constexpr double Ki = 0.004;
constexpr double Kd = 3.;

// constexpr double throttle = 0.8;
  
using P = Parameters<N>;
using E = std::tuple<P, double, std::promise<double>>;

void evaluate(std::promise<double>&& v, uWS::Group<uWS::SERVER>& ws, P p, double b) {
  // Negative Kp doesn't make sense for the model
  if(p[0] < 0 || p[3] < 0)
    return v.set_value(std::numeric_limits<double>::infinity());

  // Restart simulator
  E pb(std::move(p), b, std::move(v));
  ws.setUserData((void*)&pb);
  auto reset = WSProtocol::reset();
  ws.broadcast(reset.data(), reset.length(), uWS::OpCode::TEXT);
}
}

// Twiddle
std::unique_ptr<Twiddle::Application> Twiddle::create(po::parser&, const uWS::Hub& h) {
  auto& ud = *reinterpret_cast<E*>(static_cast<uWS::Group<uWS::SERVER>>(h).getUserData());

  const auto& p = std::get<0>(ud);
  auto best = std::get<1>(ud);
  auto& score = std::get<2>(ud);

  std::unique_ptr<Controller> angle{
    /*
    new PID(Kp, Ki, Kd)
    */
    new ScoreThreshold(
      std::unique_ptr<Controller>(new PID(p[0], p[1], p[2]))
      , [&score](const ScoreThreshold*, double s) { score.set_value(s); }
      , best, std::numeric_limits<double>::infinity()
      , measurement_skip, measurement_count)
  };
  std::unique_ptr<Controller> speed{
    /*
    new Constant(throttle)
    */
    /*
    new ScoreThreshold(
      std::unique_ptr<Controller>(
    */
      new PID(p[3], 0, p[4])
    /*
      )
      , [&score](const ScoreThreshold*, double s) { score.set_value(s); }
      , best, std::numeric_limits<double>::infinity()
      , measurement_skip, measurement_count)
    */
  };
  return std::unique_ptr<Twiddle::Application>{
    new Twiddle::Application{ std::move(angle), std::move(speed) }
  };
}

void Twiddle::extra(uWS::Hub& h, const std::unique_ptr<Twiddle::Application>&) {
  uWS::Group<uWS::SERVER>& ws = h;
  std::thread([&ws]() {
      std::cout << "Twiddling " << N << " parameters\n";
      auto result = ::twiddle<P>([&ws](P p, double b) {
          std::copy(begin(p), end(p),
                    std::ostream_iterator<double>(std::cerr << "=> ", " "));
          auto s = async(
            [&ws](std::promise<double>&& v, P p, double b) {
              return evaluate(std::move(v), ws, std::move(p), b);
            }, std::move(p), b);
          std::cerr << "<= " << s << "\n";
          return s;
        }   
        , [](const P& p) { return threshold(p, termination_threshold); }
        , twiddle_step
        , {{Kp, Ki, Kd}}
        );  
      std::copy(begin(result), end(result),
                std::ostream_iterator<double>(std::cout << "Parameters: ", " "));
      std::cout << std::endl;
    }).detach();
}


namespace {
std::unique_ptr<Controller> controller(po::parser& args,
                                       std::string p, std::string i, std::string d) {
  double Kp = args[p].get().f32;
  double Ki = args[i].get().f32;
  double Kd = args[d].get().f32;
  std::cerr << "Creating PID controller with "
            << Kp << " " << Ki << " " << Kd << "\n";
  return std::unique_ptr<Controller>{ new PID{Kp, Ki, Kd} };
}

std::unique_ptr<Controller> controller(po::parser& args, std::string c) {
  double Kc = args[c].get().f32;
  std::cerr << "Creating Constant controller with " << Kc << "\n";
  return std::unique_ptr<Controller>{ new Constant{Kc} };
}
}

// Worload
bool Workload::validate(po::parser& p) {
  auto has_steering_pid = p["sKp"].available() && p["sKi"].available() && p["sKd"].available();
  auto has_throttle_pid = p["tKp"].available() && p["tKi"].available() && p["tKp"].available();
  auto has_any_throttle_pid = p["tKp"].available() || p["tKi"].available() || p["tKp"].available();
  auto has_thorttle_const = p["tC"].available();
  auto has_conflitct = has_any_throttle_pid && has_thorttle_const;
  auto has_throttle = (has_thorttle_const || has_throttle_pid) && !has_conflitct;
  if(!has_steering_pid) {
    std::cerr << "Please specify all PID parameters for steering angle controller.\n";
  }
  if(!has_throttle) {
    if(has_conflitct) std::cerr << "Parameters conflict!\n";
    std::cerr << "Please sepcify either contstant throttle "
              << "or all PID parametes for throttle controller.\n";
  }
  return has_steering_pid && has_throttle;
}

std::unique_ptr<Workload::Application> Workload::create(po::parser& p, const uWS::Hub&) {
  return std::unique_ptr<Workload::Application>{
    new Workload::Application(controller(p, "sKp", "sKi", "sKd"),
                              p["tC"].available() ?
                              controller(p, "tC") : controller(p, "tKp", "tKi", "tKd"))
  };                            
}
