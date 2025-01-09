# world_percept_assig4

# Team8: Xinzhu Niu, Dian Wang


To run this package, please run the following commands:

First indicate where the new gazebo world is located:

`export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/student/ros/workspaces/SSY236_group8/src/world_percept_assig4/`

`source /home/student/ros/workspaces/SSY236_group8/devel/setup.bash`

Step 1:
`roslaunch world_percept_assig4 reasoning.launch`

Step 2:
`roslaunch world_percept_assig4 gazebo_ssy236.launch`

Step 3:
rosrun world_percept_assig4 reasoning_node ./src/world_percept_assig4/config

Step 4:
rosrun world_percept_assig4 knowledge_node ./src/world_percept_assig4/config

Step 5:
Run the learning node
`rosrun world_percept_assig4 learning_node`

Step 6:
Run the percept node
`rosrun world_percept_assig4 percept_node`

Step 7:
Launch the key_teleop node
`rosrun key_teleop key_teleop.py `

Step 8:
Navigate around in the Gazebo environemnt to collect the position of all the objects.
You can see class and instances have been saved into the "savedQueries" file.

Step 9:
Launch the tiago control node
`rosrun world_percept_assig4 tiago_control_node`

Step 10:
Navigate to the targeted object using a service call
for example: `rosservice call /goto_object "{obj: "bookshelf"}"`
And you can see the robot navigate to the object, together with the trajectory in the Rviz Environment. (Remember to add "Marker" and select topic to be "visualization_marker")

rosservice call /goto_object "{obj: "first_2015_trash_can"}"
