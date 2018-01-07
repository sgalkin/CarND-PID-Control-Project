#pragma once

#include <numeric>
#include <memory>
#include <type_traits>
#include "filter.h"

template<typename... Filters>
class Processor {
public:
  Processor() : filters_{{std::unique_ptr<Filters>{new Filters()}...}} {}
  explicit Processor(std::unique_ptr<Filters>... filters) : filters_{filters...} {}
  
  Observation process(Observation observation) {
    return std::accumulate(std::begin(filters_), std::end(filters_), observation,
                           [](Observation o, std::unique_ptr<Filter>& f) {
                             return f->filter(std::move(o));
                           });
  };
  
private:
  std::array<std::unique_ptr<Filter>, sizeof...(Filters)> filters_;
};

template<>
struct Processor<> {
  Observation process(Observation observation) const { return observation; }
};
