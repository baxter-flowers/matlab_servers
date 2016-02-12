# Matlab servers
Some servers to control Baxter from Matlab

## Dependencies
First install:
* [Matlab bridge](https://github.com/baxter-flowers/matlab_bridge): Allows Python-Matlab communication through file synchronization
* [Baxter commander](https://github.com/baxter-flowers/baxter_commander): Custom overlay to command Baxter
* [Natnet](https://github.com/baxter-flowers/natnet): Natnet frame receiver for Optitrack

## Servers
### matlab_fk
Calls the FK service of the robot and returns the result

### matlab_joint_commander
Commands the joints of the Baxter from Matlab

### matlab_optitrack_reader
Upon a Matlab request, read the optitrack value of a rigid body (e.g. end effector) with respect to another (e.g. world) and returns the relative pose `[x, y, z, qx, qy, qz, qw]` to Matlab.

### Notebook for baxter_commander
To open the notebooks, type in a terminal:
```
cd notebooks
ipython-notebook
```
