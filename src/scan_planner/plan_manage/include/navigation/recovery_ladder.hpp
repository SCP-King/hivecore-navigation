#pragma once
// Bounded recovery ladder: local detour -> global replan -> wait/safe stop.
// Pure logic so the ordering can be tested without ROS or a running planner.
namespace navigation {
struct RecoveryLadder {
  enum Stage { LOCAL = 0, GLOBAL = 1, WAIT = 2 };
  struct Step {
    Stage stage = LOCAL;
    bool request_global = false;  // refresh the global reference now
    bool hold = false;            // zero speed, keep the target, wait
  };
  Stage stage = LOCAL;
  int local_attempts = 0;
  int global_attempts = 0;
  int local_limit = 5;
  int global_limit = 3;

  void configure(int local_max, int global_max) {
    local_limit = local_max > 0 ? local_max : 5;
    global_limit = global_max > 0 ? global_max : 3;
  }
  void reset() { stage = LOCAL; local_attempts = 0; global_attempts = 0; }

  Step noteFailure() {
    Step step;
    if (stage == WAIT) { step.stage = WAIT; step.hold = true; return step; }
    if (stage == GLOBAL) {
      // global_attempts counts the replan that is already in flight; only
      // start another one while the budget still allows it.
      if (global_attempts >= global_limit) {
        stage = WAIT;
        step.stage = WAIT;
        step.hold = true;
        return step;
      }
      ++global_attempts;
      step.stage = GLOBAL;
      step.request_global = true;
      return step;
    }
    if (++local_attempts >= local_limit) {
      stage = GLOBAL;
      local_attempts = 0;
      global_attempts = 1;
      step.stage = GLOBAL;
      step.request_global = true;
      return step;
    }
    step.stage = LOCAL;
    return step;
  }
};
}
