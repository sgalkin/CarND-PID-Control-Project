#include "catch.hpp"

#include <cmath>
#include <functional>

#include "twiddle.h"

using namespace std::placeholders;

TEST_CASE("Twiddle") {
  using P = Parameters<6>;
  double th = 1e-6;
  P e{{1,2,99.65,4,5,-6}};
  P r = twiddle<P>(
    [&e](const P& v, double) {
      return std::inner_product(begin(e), end(e), begin(v),
                                P::value_type(0),
                                std::plus<double>(),
                                [](double a, double b) {
                                  return fabs(a - b);
                                });
    },
    std::bind(&threshold<P>, _1, th), 0.05);
  
  for(size_t i = 0; i < e.size(); ++i)
    REQUIRE(r[i] == Approx(e[i]));
}
