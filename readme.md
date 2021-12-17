# Teleop WASD keyboard
This is a ROS package which contains a node which listens to keyboard input and publishes a topic which can be used to operate a service robot moving on a plane. This was inspired by teleop_twist_keyboard [1]

## Aim
To make my own teleop node. It works like racing game. WASD controls the linear movement, qe controls the angular movement. Space is used for braking.

## Requirements
* ROS
* pynput [3]
* jupyter-notebook (for processing data from wasd_tester)

## Operation
* To run the node, first run ```roscore``` and don't forget to source the ```setup.bash``` file. Run the node using the following command

```
rosrun teleop_wasd_keyboard keyboard_listener.py
```

* To shutdown the node use ```CTRL+C```

* The command would be published in the topic ```/wasd_cmd_topic```

* Once the node it running, it will listen to all keyboard inputs, even if the terminal running this node is not selected. 
  * **Tip**: Open a text editor and select it while issuing the command. This was one can keep a record of the commands issued
* Key bindings
  * w: Increase x velocity
  * s: Decrease x velocity
  * a: Increase y velocity
  * d: Decrease y velocity
  * q: Increase z angular velocity
  * e: Decrease z angular velocity
  * space: Brake. Decelerate all speeds to 0.0
* This node can listen to more than one key stroke. 
  * For example is you press w, e, and d, the node will issue commands to increase the x and y component of the velocity as well as reduce the angular velocity along z
## License
Licensed under the [MIT License](LICENSE).

## References
[1] https://github.com/ros-teleop/teleop_twist_keyboard

[2] https://stackoverflow.com/questions/11918999/key-listeners-in-python

[3] https://pypi.org/project/pynput/
