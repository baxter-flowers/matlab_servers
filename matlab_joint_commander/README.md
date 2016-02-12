# Matlab joint commander for Baxter

This package is a server running in background allowing Matlab to send joint commands to the robot.
Although it can be easily adapted to other needs, the behaviour of the server is:
* 1) It receives a joint command from Matlab: `matlab_interface.send([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])` the order being `'s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2'`
* 2) It moves to the requested position (Joint space interpolation of joint values)
* 3) Motion might take several seconds to execute, it is blocking...
* 4) It returns the end effector position `[0.0, 0.0, 0.0]`

## Dependencies
* [Matlab bridge](https://github.com/baxter-flowers/matlab_bridge): Allows Python-Matlab communication through file synchronization
* [Baxter commander](https://github.com/baxter-flowers/baxter_commander): this package allows to display motions in RViz and convert from/to JSON, it could be replaced by `baxter_interface.Limb.move_to_joint_positions` and a custom JSON encoding

## How to use

* 1) Install the dependencies hereabove using `sudo python setup.py install`
* 2) Make sure that in an `ipython` session you are able to `import baxter_commander, matlab_bridge` successfully
* 3) Download matlab_joint_commander in your ROS workspace src directory `~/ros_ws/src/`
* 4) `cd ~/ros_ws && catkin_make`
* 5) `roslaunch baxter_commander commander.launch` in a first terminal
* 6) `rosed matlab_joint_commander joint_commander.py`, set the parameters [`simulation` and `side`](scripts/joint_commander.py#L10-L11) to match your needs
* 7) `rosrun matlab_joint_commander joint_commander.py`
* 8) Check that the server says it's ready to receive commands before sending commands from Matlab
* 9) Set [MatlabInterface.m](https://github.com/baxter-flowers/matlab_bridge/blob/master/matlab_source/MatlabInterface.m) to your Matlab path and send commands:
```
clear;
clc;

bridge = MatlabInterface();

while true
    bridge.send([0, 0, 0, 0, 0, 0, 0])    % 's0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2'
    end_effector = bridge.read()          % Blocking call while motion is executing, returns 'x', 'y', 'z' with respect to 'base'

end
```
