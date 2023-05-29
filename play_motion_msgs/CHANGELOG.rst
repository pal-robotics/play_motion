^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package play_motion_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.15 (2023-05-29)
-------------------

0.4.14 (2023-04-24)
-------------------

0.4.13 (2023-04-21)
-------------------

0.4.12 (2023-04-21)
-------------------

0.4.11 (2023-02-13)
-------------------

0.4.10 (2021-09-17)
-------------------

0.4.9 (2021-08-26)
------------------

0.4.8 (2019-09-09)
------------------

0.4.7 (2019-05-22)
------------------

0.4.6 (2018-09-28)
------------------

0.4.5 (2018-01-11)
------------------
* fixed merge
* 0.4.4
* updated changelog
* Contributors: Hilario Tome

0.4.4 (2017-11-23)
------------------
* Update changelog
* Contributors: Victor Lopez

0.4.3 (2016-03-30)
------------------
* Update changelog
* Contributors: Victor Lopez

0.4.2 (2015-11-17)
------------------
* Update changelog
* Contributors: Victor Lopez

0.4.1 (2014-11-21)
------------------
* Update changelogs
* Update package maintainer/authors
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.4.0 (2014-04-23)
------------------
* Update changelogs
* Merge pull request #36 from pal-robotics/motion-planning
  MoveIt! integration
* Merge pull request #32 from pal-robotics/list-motions-srv
  Add service call to query available motions.
* Add service call to query available motions.
* Make planning optional. Deprecate reach_time.
  - The action message has a new field: skip_planning, used to request for no
  motion planning to take place, case in which the approach time is
  automatically computed from a specified maximum velocity.
  - The above point means that the reach_time parameter is no longer required,
  hence has been removed from the action goal message.
  - A side-effect of this, is that the reach_time is computed by the approach
  planner, and no longer has to be forwarded all the way down to the
  MoveJointGroup instances.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Paul Mathieu

0.3.5 (2014-02-25)
------------------
* "0.3.5"
* Update changelogs
* Contributors: Paul Mathieu

0.3.4 (2014-02-24)
------------------
* "0.3.4"
* Update changelogs
* Contributors: Paul Mathieu

0.3.3 (2014-02-20)
------------------
* "0.3.3"
* Update changelogs
* Merge pull request #10 from pal-robotics/split-msgs
  Split package into play_motion and play_motion_msgs
* Split package into play_motion and play_motion_msgs
  fixes #9
* Contributors: Adolfo Rodriguez Tsouroukdissian, Paul Mathieu

0.3.2 (2014-02-05)
------------------

0.3.1 (2013-12-04 15:48:01 +0100)
---------------------------------

0.3.0 (2013-11-28)
------------------
