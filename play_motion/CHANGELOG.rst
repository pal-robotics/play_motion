^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package play_motion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.8 (2019-09-09)
------------------
* Fixes for shadowed variables
* Contributors: Jordan Palacios

0.4.7 (2019-05-22)
------------------
* Merge branch 'melodic_fixes' into 'erbium-devel'
  changes for melodic deprecated MoveGroup (was renamed to MoveGroupInterface)
  See merge request app-tools/play_motion!5
* changes for melodic deprecated MoveGroup (was renamed to MoveGroupInterface)
* Contributors: Sai Kishor Kothakota, Victor Lopez

0.4.6 (2018-09-28)
------------------
* Merge branch 'use-weak-ref' into 'erbium-devel'
  Fix error when changing controllers during a motion
  See merge request app-tools/play_motion!4
* Fix error when changing controllers during a motion
* Contributors: Victor Lopez

0.4.5 (2018-01-11)
------------------
* fixed merge
* 0.4.4
* updated changelog
* added missing joint trajectory controller for test
* added missing position_controllers test
* added missing xacro test depend
* migration to kinetic
* added extra test dependencies
* added gitignore and update new ros_control kinetic compatibility
* Contributors: Hilario Tome, Hilario Tomé

0.4.4 (2017-11-23)
------------------
* Update changelog
* Merge branch 'add-run-motion' into 'cobalt-devel'
  add command line run_motion executables
  See merge request app-tools/play_motion!3
* add command line run_motion executables
* Contributors: Jordi Pages, Victor Lopez

0.4.3 (2016-03-30)
------------------
* Update changelog
* Add play_motion continuous test
  fixes #6967
* Migrate package to format 2, catkin lint fixes
* Contributors: Victor Lopez

0.4.2 (2015-11-17)
------------------
* Update changelog
* Merge branch 'fix_malformed_motion_crash' into 'cobalt-devel'
  Fix Malformed Motion Crash
  As explained in ticket:
  https://redmine/issues/12188
  Trying to make a new movement I mistakenly populated too short an array of positions and I crashed (consistently) play_motion.
  This gave the error (assert):
  ```
  play_motion: /tmp/buildd/play-motion-0.4.1/play_motion/src/play_motion_helpers.cpp:142: void play_motion::populateVelocities(const TrajPoint&, const TrajPoint&, play_motion::TrajPoint&): Assertion `num_joints == point_prev.positions.size() && num_joints == point_next.positions.size()' failed.
  ```
  This fixes it by throwing an exception instead.
* Fix crash when malformed motion was requested
  It was an assert before, now it throws an exception. Added also a test.
* Contributors: Sammy Pfeiffer, Victor Lopez

0.4.1 (2014-11-21)
------------------
* Update changelogs
* Update package maintainer/authors
* Merge pull request #45 from v-lopez/add-depender-exports
  Add missing configuration for depender projects
* Add missing configuration for depender projects
* Merge pull request #44 from bmagyar/convenience_functions
  Convenience functions
* Refactor argument names to convention
* Add function alternatives where NodeHandle defaults to one of play_motion
* Merge pull request #43 from dpinol/debugging_error0
  Log error when error code is 0
* log error when core is 0
* Merge pull request #42 from pal-robotics/min-approach-dur
  Add new optional config parameter.
* Add default values and units to sample config.
* Add new optional config parameter.
  Add new parameter to configure the minimum approach time to use when
  skip_planning = true.
  If we time-parameterize trajectories using MoveIt's built-in methods, we'd
  be able to get rid of this additional parameter, but in the meantime, it
  addresses an important issue.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Bence Magyar, Daniel Pinyol, Paul Mathieu, Víctor López

0.4.0 (2014-04-23)
------------------
* Update changelogs
* Merge pull request #40 from pal-robotics/add-install-rule
  Add install() rule for headers
* Add install() rule for headers
* Template specialization must be marked as inline
  ... if we want to include it in multiple source files
* Merge pull request #29 from pal-robotics/cast-bug
  Allow cast from int to double in xmlrpc helpers
* Fetch time_from_start as a double
  This will trigger the template specialization of the previous commit
  Fixes #28
* Merge pull request #36 from pal-robotics/motion-planning
  MoveIt! integration
* Update README and sample config files
* Simplify how to disable motion planning.
  - Move disable_motion_planning param to play_motion's namespace, instead of
  nesting it inside approach_planner.
  - Update tests to exercise this behavior.
* Make tests build cleanly and pass.
  - Add additional config to force no motion planning in existing tests.
  - Fix compiler warnings.
  - Lift test timeout. Otherwise as the test suite increases we'll need to
  keep on tweaking the value. Timeout errors are highly non-descriptive.
* Allow to disable motion planning altogether.
  - Setting '~/approach_planner/disable_motion_planning=true' will cause
  play_motion to not initialize its motion planning instances.
  - When deployed like this, the node will only accept goals expplicitly
  specifying 'skip_planning=true'
* Merge pull request #32 from pal-robotics/list-motions-srv
  Add service call to query available motions.
* Refactor move_joint file layout.
  - Move guts to src/play_motion/move_joint.py
  - scripts/move_joint is a thin wrapper that catches any unhandled
  exceptions, thus preventing lengthy tracebacks that confuse
  non-developers.
  - Add move_joint to the install target.
* Colorize move_joint output.
* Simplify move_joint logic.
* Fix bug in applying computed reach time.
  A regression was introduced in which the reach time was not propagated to all
  motion waypoints.
* Silence compiler warnings.
* First iteration of move_joint script. Refs #33.
* Don't compute approach time if specified.
  When skip_planning is set to True, the input motion might specify a valid
  time_from_start for the first waypoint. The automatic reach time computation
  is thus done only when the first waypoint contains a zero time_from_start.
* Handle first waypoints with time_from_start == 0.
  Two different scenarios have been addressed when the first waypoint has zero
  time_from_start:
  - Bugfix: If the approach trajectory is null, but non-planning joints move
  a reach time must be computed for the first waypoint.
  - Cosmetic fix: Current state coincides with first waypoint. We make the
  time_from_start be a small, almost zero value to prevent the controllers
  from issuing a warning.
* Add service call to query available motions.
* Isolate move_group async spinner from rest of node
  Without this change, the synchronous spinning of play_motion was compromised.
  Now play_motion preserves its synchronous spin behavior, and move_group
  instances share a separate asynchronously serviced callback queue.
  This changeset can be considered an initial fix for #31: When canceling a goal,
  the cancel request will block until planning completes, but will then be
  serviced correctly. The current planning latencies are small enough to make this
  solution acceptable.
* Make tests build.
  - Adapt test logic to new message layout.
  - Tests currently don't run successfully, as rrbot doesn't yet have a MoveIt!
  configuration.
* Fix crash when approach planner requisites not met
  - If the required rosparam config is absent, print a descriptive error and
  don't crash.
* Make planning optional. Deprecate reach_time.
  - The action message has a new field: skip_planning, used to request for no
  motion planning to take place, case in which the approach time is
  automatically computed from a specified maximum velocity.
  - The above point means that the reach_time parameter is no longer required,
  hence has been removed from the action goal message.
  - A side-effect of this, is that the reach_time is computed by the approach
  planner, and no longer has to be forwarded all the way down to the
  MoveJointGroup instances.
* Silence cppcheck warning.
  - Type qualifiers ignored on function return type [-Wignored-qualifiers]:
  Function expecting const int returned int.
* Log message aesthetics. Caps, better messages.
* Action goal fails when approach computation fails.
  - Fix for bug where goal remained active indefinitely.
* Proper support for setting waypoint vel, acc.
  - Expose acceleration field through PlayMotion and MoveJointGroup.
  - In MoveJointGroup, don't set zero velocity if unspecified. Since we now have
  the populateVelocities method, it's already being taken care of there.
* Refactor approach computation internals.
  Planning group selection has been improved to select groups that:
  - Span at least the joint of the input motion that change between current and
  goal configurations.
  - Span at most all joints of the input motion.
* First prototype of motion planning support.
  - Add dependencies on MoveIt!
  - Add helper class that plans an approach trajectory when needed, and is able to
  reason about which planning groups to use
  - Pending tasks documented as inline TODOs
* Merge pull request #30 from pal-robotics/refactor-check-controllers
  Refactor how controllers are checked.
* Refactor how controllers are checked.
  - Unify in a central place controller checks.
  - Busy controllers are detected at the earliest possible moment.
  - Pave the way for incorporating motion planning. Without this changeset,
  we risked computing approach plans even when play_motion is busy executing
  executing a motion. Not good.
* Allow cast from int to double in xmlrpc helpers
  Fixes #28
* Contributors: Adolfo Rodriguez Tsouroukdissian, Paul Mathieu

0.3.5 (2014-02-25)
------------------
* "0.3.5"
* Update changelogs
* Harmonize doxygen tags
* Merge pull request #21 from pal-robotics/issue-20
  Fix crash with empty motion names. Fixes #20.
* Fix crash with empty motion names. Fixes #20.
* Merge pull request #22 from pal-robotics/doc-fix
  Minor doc fix.
* Minor doc fix.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Paul Mathieu

0.3.4 (2014-02-24)
------------------
* "0.3.4"
* Update changelogs
* Merge pull request #14 from pal-robotics/refactor-popuvel
  Refactor populateVelocities
* Revert intrusive changes to main function.
  - Don't swallow unexpected exceptions. Let the message show on program
  termination.
  - Hide async spinner requirements of the approach planner to its implementation.
* Refactor populateVelocities. Document it.
* Merge pull request #19 from pal-robotics/propagate-status
  Propagate controller action final state
* Propagate controller action state to internal API
  So that a proper message can be displayed, and appropriate
  measures be taken.
  Fixes #15
* Refactor some stuff in play_motion.cpp
  controllerCb had no business inside PlayMotion class
* Refactor and document MoveJointGroup
* Merge pull request #13 from pal-robotics/use-ros-messages
  Use existing msg types for traj points. (#4)
* Replace test_depend with build_depend
  <test_depend/> tags are ignored by almost everybody.
* Use existing msg types for traj points. Refs #4.
  - Move from the custom structs to trajectory_msgs types.
  - Waypoints can now have accelerations.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Paul Mathieu

0.3.3 (2014-02-20)
------------------
* "0.3.3"
* Update changelogs
* Fix dependencies (add sensor_msgs)
  This is hopefully fixing the build on the buildfarm
* Merge pull request #11 from v-lopez/hydro-devel
  Add install target for play_motion_helpers lib
* Add install target for play_motion_helpers lib
* Update is_already_there service with new repo topology
* Merge pull request #10 from pal-robotics/split-msgs
  Split package into play_motion and play_motion_msgs
* Split package into play_motion and play_motion_msgs
  fixes #9
* Contributors: Adolfo Rodriguez Tsouroukdissian, Paul Mathieu, Víctor López

0.3.2 (2014-02-05)
------------------

0.3.1 (2013-12-04 15:48:01 +0100)
---------------------------------

0.3.0 (2013-11-28)
------------------
