#include "controller.h"
#include <utility>
#include <cassert>

#include <iostream>

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

/*
FallbackController::FallbackController(
  std::unique_ptr<Controller> controller,
  std::unique_ptr<Controller> fallback,
  double threshold,
  Callback cb)
  : CallbackController(std::move(cb))
  , controller_(std::move(controller))
  , fallback_(std::move(fallback))
  , threshold_(threshold)
{}

double FallbackController::eval(double cte) {
  activeFallback_ = activeFallback_ || (cte > threshold_ && (invoke(this), true));
  assert(fallback_);
  assert(controller_);
  auto fb = fallback_->eval(cte);
  return fallback() ? fb : controller_->eval(cte);
}

bool FallbackController::fallback() const {
  return activeFallback_;
}
*/
/*
RecoveryController::RecoveryController(
  Factory factory,
  std::unique_ptr<Controller> recovery,
  double startRecovery,
  double stopRecovery,
  Callback cb)
  : CallbackController(std::move(cb))
  , factory_(std::move(factory))
  , controller_(factory_())
  , recovery_(std::move(recovery))
  , startRecovery_(startRecovery)
  , stopRecovery_(stopRecovery)
{}
  
double RecoveryController::eval(double cte) {
//  std::cerr << "RecoveryController::eval " << cte << std::endl;
  assert(recovery_);
  auto r = recovery_->eval(cte);
  if(recovery()) {
    if(cte < stopRecovery_) {
      reset();
      invoke(this);
    } else {
      return r;
    }
  } else {
    if(cte >= startRecovery_) {
      controller_.reset();
      invoke(this);
      return r;
    }
  }
  assert(controller_);
  return controller_->eval(cte);
}

bool RecoveryController::recovery() const { return !controller_; }
void RecoveryController::reset() {
//  std::cerr << "RecoveryController::reset" << std::endl;
  controller_ = factory_();
  assert(controller_);
};
*/

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
  score_ += (skip_ <= step_ && (assert(step_ > skip_), step_ - skip_) < pick_) * cte * cte;
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
        active_ = active_ && (invoke(this, s > threshold_ ? replace_ : s), false);
      }, skip, pick))
  , threshold_(threshold)
  , replace_(replace)
{}

double ScoreThreshold::eval(double cte) {
  auto r = controller_->eval(cte);
  if(controller_->score() > threshold_)
    active_ = active_ && (invoke(this, replace_), false);
  return r;
}
