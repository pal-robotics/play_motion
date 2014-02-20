^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package play_motion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.3 (2014-02-20)
------------------
* Fix dependencies (add sensor_msgs)
* Add install target for play_motion_helpers lib
* take messages out into new package play_motion_msgs

0.3.2 (2014-02-05)
------------------
* Refactor and add a datatypes.h
* Remove unneeded import
* Fix bug: calling the motion callback more than once

0.3.1 (2013-12-04)
------------------
* Populate velocity data

  * Velocities are populated only for motions that don't specify velocities for at least one waypoint.
  * Good compromise: Smoother than Fritsch-Butland, while still allowing to hold position in equal consecutive waypoints (one of the heuristics we wanted).

0.3.0 (2013-11-28)
------------------
* Add include dirs to unit tests
* Document IsAlreadyThere service
* Add service IsAlreadyThere
* Changed error codes to follow ROS standard. SUCCEEDED should be 1 (so we don't have 0 as a real error code as it's a default value).
* Return SUCCEEDED when everything went fine
* use CATKIN_ENABLE_TESTING in CMakeLists.txt
* catkin-only version now

0.2.0
-----
* rrbot target name already used in other packages
* update catkin version with tests
* fix bug with busy controller
* fix (??) issue with motions ending with an error
* updated sample motion/pose files
* implement error codes

  * SUCCEEDED
  * MOTION_NOT_FOUND
  * CONTROLLER_BUSY
  * INFEASIBLE_REACH_TIME
  * MISSING_CONTROLLER (no unit test yet)
  * TRAJECTORY_ERROR (no unit test yet)
  * GOAL_NOT_REACHED (no unit test yet)
  * OTHER_ERROR (newly added, no unit test yet)

* add tests
* fix bug with invalid service client

0.1.99
------
* now keeps track of available joint controllers
* added controller updater
* fix bugs

0.1.80
------
* goal canceling
* (not tested) simultaneous non-overlapping goals

0.1.0
-----
* play_motion: a simple tool to play pre-recorded motions on a robot
