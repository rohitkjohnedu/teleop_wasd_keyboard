# teleop_WASD_keyboard
This is a ROS package which contains a node which listens to keyboard input and publishes a topic which can be used to operate a service robot moving on a plane. This was inspired by teleop_twist_keyboard [1]

## Aim
To make my own teleop node. It works like racing game. WASD controls the speed. Space for braking.

## Requirements
* ROS
* pynput [3]
* jupyter-notebook (for processing data from wasd_tester)

## Operation
To run the node, first run ```roscore``` and don't forget to source the ```setup.bash``` file. Run the node using the following command

```
rosrun teleop_wasd_keyboard keyboard_listener.py
```

The command would be published in the topic ```/wasd_cmd_topic```
## License
Licensed under the [MIT License](LICENSE).

## References
[1] https://github.com/ros-teleop/teleop_twist_keyboard

[2] https://stackoverflow.com/questions/11918999/key-listeners-in-python

[3] https://pypi.org/project/pynput/