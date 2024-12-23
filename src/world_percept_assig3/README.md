# world_percept_assig3
# Team 8: Xinzhu Niu, Dian Wang

#Please provide a ReadMe file with the instructions you use to run your solutions

#For example:

To run this package, do:

First indicate where the new gazebo world is located:

In the docker container do:

`export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/user/exchange/SSY236_group8/src/world_percept_assig3/  `

`source /home/user/exchange/SSY236_group8/devel/setup.bash`

`roslaunch world_percept_assig3 gazebo_ssy236.launch`

Task 1:

source ros on your computer:
`source ~/knowrob_noetic/devel/setup.bash`

Task 2:

Make the package, go to the /home/user/exchange/SSY236_group8 folder, and do:
`catkin_make`

Task 3:

Source the package: 
`source /home/user/exchange/SSY236_group8/devel/setup.bash`

Task 4:

Launch the client node

`roslaunch world_percept_assig3 reasoning.launch`

Task 5:

Run the service node

`rosrun rosprolog rosprolog world_percept_assig3`
