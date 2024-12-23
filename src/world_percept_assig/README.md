# world_percept_assig

Group number: 8
Group member: Dian Wang / Xinzhu Niu

To run this package, do:

First indicate where the new gazebo world is located:

In the docker container do:

`export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/user/exchange/SSY236_group8/src/world_percept_assig/  `

`source /home/user/exchange/SSY236_group8/devel/setup.bash`

`roslaunch world_percept_assig gazebo_ssy236.launch`

Task 1:

Run the TF publisher from Gazebo topic

`rosrun world_percept_assig direct_percept_node`

Task 2:

Run the client node

`rosrun world_percept_assig percept_node`

Task 3:

Launch the service node

`rosrun world_percept_assig map_generator_node`

To use the terminal testing the "client" input, please refer to "README_TASK2.md" file inside "/srv" folder for detailed guidance.
