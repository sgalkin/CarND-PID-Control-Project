#include "controller.h"
#include <cassert>
#include <cmath>

Controller::Controller() {}
Controller::~Controller() {}


Constant::Constant(double value) : value_(value) {}
double Constant::eval(double) { return value_; }


PID::State PID::State::operator+ (double e) {
  return *this += e;
}

PID::State& PID::State::operator+= (double e) {
  d_error = e - p_error;
  p_error = e;
  i_error += e;
  return *this;
}

double PID::State::operator* (PID::Weight w) {
  return w.Kp*p_error + w.Ki*i_error + w.Kd*d_error;
}


PID::PID(double Kp, double Ki, double Kd)
  : enable_d(false)
  , weight_{Kp, Ki, Kd}
{}

double PID::eval(double cte) {
  state_ += cte;
  auto o = -(state_ * Weight(weight_.Kp, weight_.Ki, weight_.Kd*enable_d));
  enable_d = true;
  return o;
}


ScoreController::ScoreController(
  std::unique_ptr<Controller> controller, Callback cb, size_t skip, size_t pick)
  : CallbackController(std::move(cb))
  , controller_((assert(controller), std::move(controller)))
  , skip_(skip)
  , pick_(pick)
  , step_(0)
  , score_(0)
{}

double ScoreController::eval(double cte) {
  score_ += (skip_ <= step_ && (assert(step_ >= skip_), step_ - skip_) < pick_) * cte * cte;
  double r = (assert(controller_), controller_)->eval(cte);
  ++step_;
  if(step_ > skip_ && step_ - skip_ == pick_) {
    invoke(this, score());
  }
  return r;
}

double ScoreController::score() const {
  return score_;
}


ScoreThreshold::ScoreThreshold(
  std::unique_ptr<Controller> controller, Callback cb,
  double threshold, double replace,
  size_t skip, size_t pick)
  : CallbackController(std::move(cb))
  , controller_(
    new ScoreController(
      std::move(controller),
      [this](const ScoreController*, double s) {
        active_ &= (invoke(this, s > threshold_ || !std::isfinite(s) ? replace_ : s), false);
      }, skip, pick))
  , threshold_(threshold)
  , replace_(replace)
{}

double ScoreThreshold::eval(double cte) {
  auto r = controller_->eval(cte);
  if(controller_->score() > threshold_ || !std::isfinite(controller_->score()))
    active_ = active_ && (invoke(this, replace_), false);
  return r;
}
