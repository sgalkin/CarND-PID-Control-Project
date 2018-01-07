#include "catch.hpp"
#include "controller.h"

TEST_CASE("Constant") {
  Constant c(42);
  REQUIRE(c.eval(0) == Approx(42));
}

TEST_CASE("PID") {
  SECTION("P") {
    PID c(2, 0, 0);
    REQUIRE(c.eval(10) == Approx(-20));
    REQUIRE(c.eval(20) == Approx(-40));
  }
  SECTION("I") {
    PID c(0, 2, 0);
    REQUIRE(c.eval(10) == Approx(-20));
    REQUIRE(c.eval(20) == Approx(-60));
  }
  SECTION("D") {
    PID c(0, 0, 2);
    REQUIRE(c.eval(10) == Approx(0));
    REQUIRE(c.eval(20) == Approx(-20));
  }
}

TEST_CASE("Score") {
  SECTION("Default") {
    ScoreController s(std::unique_ptr<Controller>(new Constant(42)));
    REQUIRE(Approx(42) == s.eval(1));
    REQUIRE(Approx(0) == s.score());
    REQUIRE(Approx(42) == s.eval(2));
    REQUIRE(Approx(4) == s.score());
    REQUIRE(Approx(42) == s.eval(3));
    REQUIRE(Approx(13) == s.score());
  }
  SECTION("Range") {
    double scr{0};
    bool called{false};
    auto cb = [&scr, &called](const ScoreController*, double score) {
      scr = score;
      called = true;
    };
    ScoreController s(std::unique_ptr<Controller>(new Constant(42)), cb, 10, 10);
    for(int i = 0; i < 10; ++i) {
      REQUIRE(Approx(42) == s.eval(1));
      REQUIRE(Approx(0) == s.score());
      REQUIRE(!called);
    }
    double score = 0;
    for(int i = 0; i < 9; ++i) {
      score += i*i;
      REQUIRE(Approx(42) == s.eval(i));
      REQUIRE(Approx(score) == s.score());
      REQUIRE(!called);
    }
    score += 10*10;
    REQUIRE(Approx(42) == s.eval(10));
    REQUIRE(called);
    REQUIRE(Approx(score) == scr);
    REQUIRE(Approx(score) == s.score());
  }
}

TEST_CASE("ScoreThreshold") {
  SECTION("Unmet") {
    double scr{0};
    bool called{false};
    auto cb = [&scr, &called](const ScoreThreshold*, double score) {
      scr = score;
      called = true;
    };
    ScoreThreshold s(std::unique_ptr<Controller>(new Constant(42)), cb, 10, -1, 0, 1);
    s.eval(1);
    REQUIRE(called);
    REQUIRE(scr == Approx(1));
  }
  SECTION("Met callback") {
    double scr{0};
    bool called{false};
    auto cb = [&scr, &called](const ScoreThreshold*, double score) {
      scr = score;
      called = true;
    };
    ScoreThreshold s(std::unique_ptr<Controller>(new Constant(42)), cb, 10, -1, 0, 1);
    s.eval(11);
    REQUIRE(called);
    REQUIRE(scr == Approx(-1)); 

    called = false;
    s.eval(11);
    REQUIRE(!called);
  }
}
