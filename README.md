play_motion
===========

This tool is used to play pre-recorded motions on `ros_control` compliant robots via a simple actionlib interface.
Any robot that implements control via `joint_trajectory_controller` can be put into motion with `play_motion`.


How-to
------

1. Prepare one (or more!) motion. They need to be loaded as rosparams. You will find samples in the `config/`
   folder.
 
   ```
   rosparam load config/your_motion_file.yaml
   ```
   
2. Launch play_motion
 
   ```
   rosrun play_motion play_motion
   ```

3. Fire an actionlib client that publishes on /play_motion action server, enter the name of your motion and
   don't forget `reach_time`, which is the time you give your robot to reach the first point of your motion
   from its current configuration.
   
That's it!
