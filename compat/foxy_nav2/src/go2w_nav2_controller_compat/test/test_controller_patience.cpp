// SPDX-License-Identifier: Apache-2.0
// Deterministic tests of the policy actually called by controller_server.
// No ROS context, networking, SDK or robot is used by this executable.
#include <array>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "go2w_nav2_controller_compat/controller_patience.hpp"

namespace
{
struct PlannerError : std::runtime_error {using std::runtime_error::runtime_error;};
using Command = std::array<double, 6>;

void expect(bool value, const char * message)
{
  if (!value) {throw std::runtime_error(message);}
}

struct Fixture
{
  go2w_foxy_controller::ControllerPatience policy;
  std::int64_t time{1000000000};
  int computes{0}, warnings{0}, zeroes{0};
  bool failing{false}, unexpected_error{false};
  Command valid{{.3, .1, .2, .4, .5, .6}};
  Fixture() {policy.reset(time);}
  Command run(double tolerance = .3)
  {
    return policy.compute<PlannerError>(tolerance,
      [&]() {
        ++computes;
        if (unexpected_error) {throw std::logic_error("not a planner failure");}
        if (failing) {throw PlannerError("temporary controller failure");}
        return valid;
      },
      [&]() {return time;},
      [&]() {++zeroes; return Command{};},
      [&](const char * message) {
        ++warnings;
        expect(std::string(message) == "temporary controller failure", "lost diagnostic");
      });
  }
};

void expectPlannerError(Fixture & fixture, double tolerance, const char * message)
{
  bool failed = false;
  try {fixture.run(tolerance);} catch (const PlannerError & error) {
    failed = true;
    expect(std::string(error.what()) == message, "wrong planner error");
  }
  expect(failed, "expected planner error did not propagate");
}
}  // namespace

int main()
{
  int passed = 0;
  try {
    {
      Fixture f;
      expect(f.run() == f.valid, "valid command changed");
      expect(f.computes == 1 && f.zeroes == 0 && f.warnings == 0, "success side effects");
      ++passed;
    }
    {
      Fixture f; f.failing = true; f.time += 250000000;
      expect(f.run() == Command{}, "transient exception reused a nonzero axis");
      expect(f.zeroes == 1 && f.warnings == 1, "missing zero or warning");
      ++passed;
    }
    {
      Fixture f; f.failing = true; f.time += 300000000;
      expect(f.run() == Command{}, "exact deadline must match Humble strict > comparison");
      ++passed;
    }
    {
      Fixture f; f.failing = true; f.time += 300000001;
      expectPlannerError(f, .3, "Controller patience exceeded");
      ++passed;
    }
    {
      Fixture f; f.failing = true;
      for (int i = 0; i < 6; ++i) {
        f.time += 50000000;
        expect(f.run() == Command{}, "failure inside time allowance");
      }
      ++f.time;
      expectPlannerError(f, .3, "Controller patience exceeded");
      expect(f.computes == 7, "failed attempts extended time allowance");
      ++passed;
    }
    {
      Fixture f; f.time += 200000000;
      expect(f.run() == f.valid, "successful update changed command");
      f.failing = true; f.time += 300000000;
      expect(f.run() == Command{}, "success did not reset last valid command time");
      ++f.time; expectPlannerError(f, .3, "Controller patience exceeded");
      ++passed;
    }
    {
      Fixture f; f.failing = true;
      expectPlannerError(f, 0., "temporary controller failure");
      expect(f.zeroes == 0 && f.warnings == 0, "disabled tolerance changed exception");
      ++passed;
    }
    {
      Fixture f; f.failing = true; f.time += 3600000000000;
      expect(f.run(-1.) == Command{}, "explicit infinite allowance changed");
      ++passed;
    }
    {
      Fixture f; f.unexpected_error = true;
      bool failed = false;
      try {f.run();} catch (const std::logic_error &) {failed = true;}
      expect(failed && f.zeroes == 0 && f.warnings == 0,
        "unexpected exception was treated as a recoverable planner error");
      ++passed;
    }
    {
      Fixture f; f.failing = true; f.time += 900000000;
      f.policy.reset(f.time);  // A distinct newly accepted action, not a replan.
      expect(f.run() == Command{}, "new action inherited an expired allowance");
      ++passed;
    }
    {
      Fixture f; f.failing = true; f.time += 200000000;
      expect(f.run() == Command{}, "initial tolerance");
      expectPlannerError(f, .1, "Controller patience exceeded");
      expect(f.run(.4) == Command{}, "committed tolerance increase was not read");
      ++passed;
    }
    {
      // Match Humble's ROS-clock semantics; independent freshness/watchdog gates
      // still own clock validity and are not weakened by this policy.
      Fixture f; f.failing = true; f.time -= 100000000;
      expect(f.run() == Command{}, "ROS clock rewind compatibility changed");
      ++passed;
    }
    {
      for (double invalid : {-.1, -2., std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()})
      {
        Fixture f; bool failed = false;
        try {f.run(invalid);} catch (const std::invalid_argument &) {failed = true;}
        expect(failed && f.computes == 0, "invalid policy allowed a controller command");
      }
      ++passed;
    }
  } catch (const std::exception & error) {
    std::cerr << "FAILED after " << passed << " cases: " << error.what() << '\n';
    return 1;
  }
  std::cout << "controller_patience: " << passed << " cases passed\n";
  return 0;
}
