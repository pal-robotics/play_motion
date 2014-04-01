play_motion
===========

This tool is used to play pre-recorded motions on `ros_control` compliant robots via a simple actionlib interface.
Any robot that implements control via `joint_trajectory_controller` can be put into motion with `play_motion`.

Starting from version 0.4, `play_motion` will by default compute a collision-free trajectory between the robot's
current state and the first motion waypoint. This requires your robot to have a running
[moveit](http://moveit.ros.org/) setup, and to configure `play_motion` to use it. If you're not interested in the
motion planning capability, it's possible to disable it altogether. Refer to the `config/` folder
for configuration examples.

How-to
------

1. Prepare one (or more!) motion. They need to be loaded as rosparams. You will find samples in the `config/`
   folder.

   ```
   rosparam load config/your_motion_file.yaml
   ```

2. Launch play_motion (without motion planning capabilities, for simplicity)

   ```
   rosparam set /play_motion/disable_motion_planning true
   rosrun play_motion play_motion
   ```

3. Fire an actionlib client that publishes on /play_motion action server, enter the name of your motion on the
   `motion_name` field and set `skip_planning=true` to not use motion planning.

That's it!
