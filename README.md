# drake_collsion_filterking-example-brea
A minimal example on parsing collsion_filter_group problems in drake. Also added a customized URDF for 3dof robot. 

The unit test fails on drake version 38:
```
/src/dig/dracula/tests/test_urdf_collision_parsing.cc:35: Failure
Expected equality of these values:
  inspector.GetCollisionCandidates().size()
    Which is: 4
  0
unexpected collison candidate pair = arm1 and  arm3
unexpected collison candidate pair = arm1 and  arm_end_effector
unexpected collison candidate pair = arm2 and  arm_end_effector
unexpected collison candidate pair = object_1 and  object_2
[  FAILED  ] ParseCollisionFilter.ThreeDofRobot (9 ms)
[----------] 1 test from ParseCollisionFilter (9 ms total)

[----------] Global test environment tear-down
[==========] 1 test from 1 test suite ran. (9 ms total)
[  PASSED  ] 0 tests.
[  FAILED  ] 1 test, listed below:
[  FAILED  ] ParseCollisionFilter.ThreeDofRobot

 1 FAILED TEST
```
But passes on drake version 33 (the one we use at the time of drafting this PR):
```
[----------] 1 test from ParseCollisionFilter
[ RUN      ] ParseCollisionFilter.ThreeDofRobot
[       OK ] ParseCollisionFilter.ThreeDofRobot (2 ms)
[----------] 1 test from ParseCollisionFilter (2 ms total)

[----------] Global test environment tear-down
[==========] 1 test from 1 test suite ran. (2 ms total)
[  PASSED  ] 1 test.
```
