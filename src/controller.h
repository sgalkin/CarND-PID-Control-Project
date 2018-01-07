#pragma once

#include <memory>
#include <limits>
#include <functional>

class Controller {
public:
  Controller();
  virtual ~Controller();
  virtual double eval(double input) = 0;
};


class Constant : public Controller {
public:
  explicit Constant(double value);
  double eval(double) override;

private:  
  double value_;
};


class PID : public Controller {
  struct Weight {
    Weight(double Kp, double Ki, double Kd)
      : Kp(Kp), Ki(Ki), Kd(Kd)
    {}
    
    const double Kp;
    const double Ki;
    const double Kd;
  };
  
  struct State {
    State()
      : p_error(0), i_error(0), d_error(0)
    {}
    
    State operator+ (double e);
    State& operator+= (double e);
    double operator* (Weight w);

    double p_error;
    double i_error;
    double d_error;
  };
  
public:
  PID(double Kp, double Ki, double Kd);
  double eval(double cte) override;
  
private:
  bool enable_d{false};
  const Weight weight_;

  State state_;
};


template<typename HostController, typename... Args> 
class CallbackController : public Controller {
public:
  static void Noop(const HostController*, Args...) {};
  using Callback = std::function<void(const HostController*, Args...)>;
  
  explicit CallbackController(Callback cb = Noop)
    : cb_(std::move(cb))
  {}
  
  void invoke(const HostController* host, Args... args) {
    cb_(host, std::move(args)...);
  }

private:
  Callback cb_;
};


class ScoreController
  : public CallbackController<ScoreController, double> {
  using CallbackController<ScoreController, double>::Callback;
public:
  ScoreController(std::unique_ptr<Controller> controller,
                  Callback cb = Noop,
                  size_t skip = 1,
                  size_t pick = std::numeric_limits<size_t>::max());
  double eval(double cte) override;
  
  double score() const;

private:
  std::unique_ptr<Controller> controller_;

  size_t skip_;
  size_t pick_;
  size_t step_;
  double score_;
};


class ScoreThreshold
  : public CallbackController<ScoreThreshold, double> {
  using CallbackController<ScoreThreshold, double>::Callback;
public:
  ScoreThreshold(std::unique_ptr<Controller> controller,
                 Callback cb,
                 double threshold,
                 double replace = std::numeric_limits<double>::infinity(),
                 size_t skip = 1,
                 size_t pick = std::numeric_limits<size_t>::max());
  double eval(double cte) override;

private:
  bool active_{true};
  std::unique_ptr<ScoreController> controller_;
  double threshold_;
  double replace_;
};
