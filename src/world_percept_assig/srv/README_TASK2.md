Process of using the terminal to test the “client” input:

First, open a new terminal and source the package:

source ~/ros/workspaces/SSY236_group8/devel/setup.bash

Run map_generator node which we want to test:

rosrun rosrun world_percept_assig direct_percept_node

Then, type the following command in the terminal:

rosservice call /update_object_list "object_pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
object_name: 'cafe_table'"

cafe_table is one of the example object we found in 'kitchen_chalmers.world'

The result will be returned as 'true' according to the service definition in the .srv file.
