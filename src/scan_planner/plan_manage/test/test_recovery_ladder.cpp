#include <navigation/recovery_ladder.hpp>
#include <cassert>
int main(){
  navigation::RecoveryLadder l;l.configure(3,2);
  // 1-2: keep detouring locally, never touch the global reference.
  auto s=l.noteFailure();assert(s.stage==navigation::RecoveryLadder::LOCAL&&!s.request_global&&!s.hold);
  s=l.noteFailure();assert(s.stage==navigation::RecoveryLadder::LOCAL&&!s.request_global);
  // 3: local budget exhausted -> one global replan is requested.
  s=l.noteFailure();assert(s.stage==navigation::RecoveryLadder::GLOBAL&&s.request_global);
  // 2nd global attempt still retries; 3rd exhausts the ladder into hold.
  s=l.noteFailure();assert(s.stage==navigation::RecoveryLadder::GLOBAL&&s.request_global);
  s=l.noteFailure();assert(s.stage==navigation::RecoveryLadder::WAIT&&s.hold);
  // Holding is idempotent: no extra global refresh, no reset.
  s=l.noteFailure();assert(s.stage==navigation::RecoveryLadder::WAIT&&s.hold&&!s.request_global);
  // A new goal (or operator resume) restarts from the local stage.
  l.reset();s=l.noteFailure();assert(s.stage==navigation::RecoveryLadder::LOCAL);
  // A single-attempt configuration goes straight to global, then hold.
  navigation::RecoveryLadder tight;tight.configure(1,1);
  s=tight.noteFailure();assert(s.stage==navigation::RecoveryLadder::GLOBAL&&s.request_global);
  s=tight.noteFailure();assert(s.stage==navigation::RecoveryLadder::WAIT&&s.hold);
  // Invalid limits fall back instead of disabling recovery.
  navigation::RecoveryLadder bad;bad.configure(0,-1);assert(bad.local_limit==5&&bad.global_limit==3);
}
