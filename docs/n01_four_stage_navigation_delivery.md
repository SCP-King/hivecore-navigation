# N-01 Four-Stage Navigation Delivery

## A. Planning Only

SCAN global/local candidate paths are produced. No actuator output is allowed.
`safety_authorization=NEVER`, `execution_authorized=false`.

## B. Shadow Control

A path follower computes candidate speed/yaw rate from the local candidate path.
It publishes diagnostics only; it has no `/cmd_vel` publisher and no chassis
command path. A conservative CBF-style speed screen uses dynamic obstacle
candidates and a safe radius.

## C. Guarded Low-Speed Motion

Requires explicit operator authorization, verified emergency stop chain,
fresh localization/path, dynamic obstacle supervision, and a cleared test area.
The shadow path follower must first run without actuator output.

## D. Full Navigation

Only after A/B/C acceptance, with continuous safety supervision, speed limits,
watchdogs and operator hold. Planning success alone never authorizes motion.
