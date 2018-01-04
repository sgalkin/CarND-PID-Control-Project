#pragma once

#include <array>
#include <numeric>

namespace detail {
template<typename P>
P fill(typename P::value_type v) {
  P r;
  std::fill(begin(r), end(r), v);
  return r;
}

template<typename P, typename E>
std::tuple<P, P, typename P::value_type>
twiddle(E evaluate, size_t i,
        P candidate, P delta,
        typename P::value_type best,
        typename P::value_type step) {
  static constexpr std::array<int, 2> factor{{-1, 1}};
  for(auto f: factor) {
    candidate[i] += f * delta[i];
    auto c = candidate;
    auto v = evaluate(std::move(c), best);
    if(v < best) {
      delta[i] *= (1 + step);
      return {std::move(candidate), std::move(delta), v};
    }
    candidate[i] -= f * delta[i];
  }
  delta[i] *= (1 - step);
  return {std::move(candidate), std::move(delta), best};
}
}

template<size_t N, typename T = double>
using Parameters = std::array<T, N>;

template<typename P>
bool threshold(const P& delta, double value) {
  return std::accumulate(begin(delta), end(delta),
                         typename P::value_type(0)) < value;
}
#include <iostream>
template<typename P, typename E, typename T>
P twiddle(E evaluate, T terminate, typename P::value_type step,
          P parameters = detail::fill<P>(0.),
          P delta = detail::fill<P>(1.)) {
  auto best{std::numeric_limits<typename P::value_type>::infinity()};
  while(!terminate(delta)) {
    std::cerr << "Sdelta: " << std::accumulate(begin(delta), end(delta), typename P::value_type(0)) << std::endl;
    for(size_t i = 0; i < parameters.size(); ++i) {
      std::tie(parameters, delta, best) =
        detail::twiddle(evaluate, i, std::move(parameters), std::move(delta), best, step);
    }
  }
  return parameters;
}

/*
template<size_t N, typename Control>
class Twiddle {
  using ControlFactory = std::function<Control(const std::array<double, N>&)>
  
  enum { Recovery, Run } stage_;
public:
  Twiddle(ControlFactory factory,
          std::array<double, N> recovery,
          size_t steps,
          double threshold) :
    factory_(factory),
    recovery_(recovery),
    steps_(steps),
    threshold_(threshold) {
    std::fill(begin(parameter_), end(parameter), 0);
    std::fill(begin(delta_), end(delta_), 1);
  }
  
  double operator()(double cte) {
    ++step_;
    cte_ += cte*cte;

    if(step_ >= steps_ || cte_ > 3*threshold) {
      step_ == 0;
      cte_ = cte*cte;
      return (cte_ > treshold) ? stageFailed(cte) : stageCompleted(cte);
    } else {
      retur stagePrgoress(cte); 
    }
  }
  
private:
  double stageFailed(double cte) {
    switch(stage_) {
    case Recovery:
      assert("Unable to recover");
      return cte;
    case Run:
      control_ = factory_(recovery_);
      return stagePrgoress(cte);
    }
  }

  double stageProgress(double cte) {
    return (*control_)(cte);
  }

  double stageCompleted(double cte) {
    control_ = createControl(nextParameter(p_));
    return stageProgress(cte);
  }

  static std::tuple<size_t, int> nextParameter(std::tuple<size_t, int> p) {
    switch(p) {
    case +1:
    case -1:
    case 0:
      return p;
  }

  // State
  size_t step_{0};
  double cte_{0};
  std::tuple<size_t, int> p_{0, +1};

  ControlFactory factory_;
  std::array<double, N> recovery_;

  std::array<double, N> parameters_;
  std::array<double, N> delta_;
  
  const double treshold_;
  const size_t steps_;
};
*/
