# odrive_ros
ROS driver for the [ODrive motor driver](https://odriverobotics.com/)

This is a basic pass at a ROS driver to operate the ODrive as a differential drive. It's Python-based, so not super-fast, but it'll get you started. Maybe in time this will have an optimised C++ version, but I woudn't count on it anytime soon. ;)

Future plans include adding a simulation mode so you can continue to use this driver in a simulation mode within gazebo.

Feedback, issues and pull requests welcome.

## Usage

You will need the main ODrive Python tools installed.

To install:
```sh
git clone https://github.com/madcowswe/ODrive
cd ODrive/tools
sudo pip install monotonic # required for py < 3

# sudo python setup.py install # doesn't work due to weird setup process, so do the following:
python setup.py sdist
sudo pip install dist/odrive-xxx.tar.gz
```

then `rosrun odrive_ros odrive_node` will get you on your way. 

There is a sample launch file showing the use of the wheelbase, tyre circumference, and encoder count parameters too, try `roslaunch odrive_ros odrive.launch` or copy it into your own package and edit the values to correspond to your particular setup.

If you want to test out the driver separate from ROS, you can also install as a regular Python package.

```sh
roscd odrive_ros
sudo pip install -e .
```

If you want to use a bare Python shell, see below for the import which exposes the ODrive with setup and drive functions. You can use either ODriveInterfaceAPI (uses USB) or ODriveInterfaceSerial (requires the /dev/ttyUSBx corresponding to the ODrive as parameter, but not tested recently).

```python
from odrive_ros import odrive_interface
od = odrive_interface.ODriveInterfaceAPI()
od.connect()
od.setup()          # does a calibration
od.engage()         # get ready to drive
od.drive(1000,1000) # drive axis0 and axis1
od.release()        # done
```

You can now (as of v0.5) use this wrapper from within an `odrivetool shell`. Once the shell has launched and connected to your ODrive, run the following, then you can use the commands as if in a bare Python interactive session above, and still get the help provided by odrivetool.

```python
import odrive_interface
od = odrive_interface.ODriveInterfaceAPI(active_odrive=odrv0)
```

## Ros2

example usage using params file
`ros2 run odrive_ros odrive_node --ros-args --params-file params.yaml`

example usage setting parame in cli
`ros2 run odrive_ros odrive_node --ros-args -p "serial_number:=207D3591524B"`

example connect
`ros2 service call /connect_driver std_srvs/srv/Trigger`

example disconnect
`ros2 service call /disconnect_driver std_srvs/srv/Trigger`

example engage
`ros2 service call /engage_motors std_srvs/srv/Trigger`

example release
`ros2 service call /release_motors std_srvs/srv/Trigger`

example sending `cmd_vel` message:
`ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'`
`ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'`

## Acknowledgements

Thanks to the ODrive team for a great little bit of hardware and the active community support. Hope this helps!

- [ODrive homepage](https://odriverobotics.com)
- [ODrive getting started](https://docs.odriverobotics.com)
- [ODrive main repo](https://github.com/madcowswe/ODrive)

Thanks to Simon Birrell for his tutorial on [how to package a Python-based ROS package](http://www.artificialhumancompanions.com/structure-python-based-ros-package/).

