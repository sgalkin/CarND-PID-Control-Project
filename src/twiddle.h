#pragma once

#include <array>
#include <numeric>
#include <future>

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
  static constexpr std::array<int, 2> factor{{1, -1}};
  for(auto f: factor) {
    auto c = candidate;
    c[i] += f * delta[i];
    auto v = evaluate(std::move(c), best);
    if(v < best) {
      delta[i] *= (1 + step);
      return std::make_tuple(std::move(c), std::move(delta), v);
    }
  }
  delta[i] *= (1 - step);
  return std::make_tuple(std::move(candidate), std::move(delta), best);
}
}

template<size_t N, typename T = double>
using Parameters = std::array<T, N>;

template<typename P, typename E>
typename P::value_type async(E evaluate, P parameters, typename P::value_type best) {
  std::promise<typename P::value_type> p;
  auto f = p.get_future();
  evaluate(std::move(p), std::move(parameters), best);
  return f.get();
}

template<typename P>
bool threshold(const P& delta, double value) {
  return std::accumulate(begin(delta), end(delta),
                         typename P::value_type(0)) < value;
}

template<typename P, typename E, typename T>
P twiddle(E evaluate, T terminate, typename P::value_type step,
          P parameters = detail::fill<P>(0.),
          P delta = detail::fill<P>(1.)) {
  auto best{std::numeric_limits<typename P::value_type>::infinity()};
  while(!terminate(delta)) {
    for(size_t i = 0; i < parameters.size(); ++i) {
      std::tie(parameters, delta, best) =
        detail::twiddle(evaluate, i, std::move(parameters), std::move(delta), best, step);
    }
  }
  return parameters;
}
