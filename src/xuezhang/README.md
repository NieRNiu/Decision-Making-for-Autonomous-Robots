
For this assignment, I work with my groupmate André Pereira,but I mean only task 4. I decide to finish code of task4 by myself after deadline.



# world_percept_assig3

Please provide a ReadMe file with the instructions you use to run your solutions

For example:

To run this package, do:

First indicate where the new gazebo world is located:

To run this package, do:
`source /knowrob_ws/devel/setup.bash`  then  `catkin_make`
 





Task 1:


```bash


### In the termional 1 
source /home/user/exchange/ssy236_yaochen/devel/setup.bash
 `roslaunch world_percept_assig3 reasoning.launch`

### In the termional 2
source /home/user/exchange/ssy236_yaochen/devel/setup.bash
`rosrun world_percept_assig3 map_generator_node`

### In the termional 3
source /home/user/exchange/ssy236_yaochen/devel/setup.bash
`rosservice call /update_object_list "{object_name: 'cafe_table_direct', object_pose: {position: {x: -1.18, y: -1.15, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}"  `
`rosservice call /update_object_list "{object_name: 'table_direct', object_pose: {position: {x: -1.18, y: 1.15, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}"  `
 `rosservice call /get_scene_object_list "{object_name: 'all'}"  `

  rosservice call /get_scene_object_list "{object_name: "table_direct"}"


 output:
obj_found: True
objects: 
  name: 
    - cafe_table_direct
    - table_direct
  pose: 
    - 
      position: 
        x: -1.18
        y: -1.15
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    - 
      position: 
        x: -1.18
        y: 1.15
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  twist: []
message: ''

```

Task 2:
TERMINAL 1:
`rosparam load loadKnowledge.yaml` or we hae some substitution `roslaunch world_percept_assig3 reasoning.launch` 


Terminal 2:

`export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/user/exchange/ssy236_yaochen/src/world_percept_assig3/`

`source /home/user/exchange/ssy236_yaochen/devel/setup.bash`

`roslaunch world_percept_assig3 gazebo_ssy236.launch`


Terminal 3:
then use `rosparam list` to check if we already have parameters loaded into the ROS parameter server.
#then `rosrun world_percept_assig3 reasoning_node /home/user/exchange/ssy236_yaochen/src/world_percept_assig3/config ` 

` rosrun world_percept_assig3 reasoning_node ./src/world_percept_assig3/config` 

output:
[ INFO] [1701124127.451929514]: Wait for the Prolog service...
[ INFO] [1701124127.459432843]: query_file: /home/user/exchange/ssy236_yaochen/src/world_percept_assig3/prolog/queries/savedQueries.txt


Terminal 4:
source /home/user/exchange/ssy236_andperei/devel/setup.bash
rosservice call /assert_knowledge "{object_pose: {position: {x: 1, y: 1, z: 1}, orientation:{x: 0, y: 0, z: 0,w: 1}}, object_name: "Test"}"

Terminal 5:

source /home/user/exchange/ssy236_yaochen/devel/setup.bash

`rosrun key_teleop key_teleop.py`

Terminal 6:
source /home/user/exchange/ssy236_yaochen/devel/setup.bash
`rosrun world_percept_assig3 percept_node`



Task 3:

source /home/user/exchange/ssy236_yaochen/devel/setup.bash
terminal 1: roslaunch world_percept_assig3 reasoning.launch


source /home/user/exchange/ssy236_yaochen/devel/setup.bash
terminal 2: rosrun world_percept_assig3 knowledge_node /home/user/exchange/ssy236_yaochen/src/world_percept_assig3/config

[ INFO] [1701086131.760783627]: Wait for the Prolog service...
[ INFO] [1701086131.771389307]: query_file: /home/user/exchange/ssy236_yaochen/src/world_percept_assig3/prolog/queries/savedQueries.txt
[ INFO] [1701086139.443614065]: query: get_class('Table')
[ INFO] [1701086139.448662030]: query: get_class('Bottle')

terminal 3: rosservice call /load_knowledge "start: 1"




OUTPUT: 
In terminal 1:
[ INFO] [1701086655.605114594]: rosprolog service is running.
New class created: http://www.chalmers.se/ontologies/ssy236Ontology.owl#Table
New class created: http://www.chalmers.se/ontologies/ssy236Ontology.owl#Bottle


In terminal 2:
[ INFO] [1701086664.816480010]: Wait for the Prolog service...
[ INFO] [1701086664.825809131]: query_file: /home/user/exchange/ssy236_yaochen/src/world_percept_assig3/prolog/queries/savedQueries.txt
[ INFO] [1701086669.263375028]: query: get_class('Table')
[ INFO] [1701086669.268468439]: query: get_class('Bottle')

In terminal 3:
confirm: True


Task 4:


 `roslaunch world_percept_assig3 reasoning.launch`

`rosrun world_percept_assig3 tiago_control_node`



To test the code run the following commands:
```bash
#T0
source /home/user/exchange/ssy236_yaochen/devel/setup.bash
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/user/exchange/ssy236_yaochen/src/world_percept_assig3
roslaunch world_percept_assig3 gazebo_ssy236.launch

#T1 - map generator
source /home/user/exchange/ssy236_yaochen/devel/setup.bash
rosrun world_percept_assig3 map_generator_node 

#T2 - percept
source /home/user/exchange/ssy236_yaochen/devel/setup.bash
rosrun world_percept_assig3 percept_node 

#T3 - tiago control
source /home/user/exchange/ssy236_yaochen/devel/setup.bash
rosrun world_percept_assig3 tiago_control_node 

#T4 - testing commands
source /home/user/exchange/ssy236_yaochen/devel/setup.bash
rosservice call /goto_object "{obj: "beer"}"
rosservice call /goto_object "{obj: "bowl"}"
rosservice call /goto_object "{obj: "bookshelf"}"