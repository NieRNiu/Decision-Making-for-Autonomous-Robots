

First, source the package:

source ~/ros/workspaces/wor /devel/

Use the following comand:

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


The result will be true according to the service definition in the .srv file.
