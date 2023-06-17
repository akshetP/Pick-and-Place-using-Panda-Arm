# Pick and Place using Panda Arm (Team Project)
This project is a graded part of the module 'Robotic Sensing, Manipulation and Interaction'. It is a team project done by 3 students. The goal of this project is to implement a pick and place task using Panda Arm. The task is divided into 3 parts:
* MoveIt! - Pick and Place at given positions
* Shape detection
* Planning and Execution

The tasks description is detailed [here.](Task_Description.pdf)

## Author
* Hei Yin Wong (heiyin.wong.22@ucl.ac.uk)
* Akshet Patel (akshet.patel.22@ucl.ac.uk)
* Agung Nuza Dwiputra (agung.dwiputra.22@ucl.ac.uk)

## How to build the package
This project is built on ROS 1 using ROS Noetic version. 
To build the project simply use 
`catkin build cw_3_team_1`

## How to run the package
There are 3 task implemented on this code. To launch this package, simply use:
`roslaunch cw1_team_3 run_simulation.launch`

To start one of the 3 task, call the service using `/task 1`, `/task 2` or `/task 3` like below:
`rosservice call /task 1`
`rosservice call /task 2`
`rosservice call /task 3`

## Total time to implement
* Task 1: 6 hours
* Task 2: 10 hours
* Task 3: 20 hours

## Team member contribution
### Task 1 and Task 2:
* Each person solved the tasks independently.
* This was a good practice because it allowed us to compare different approaches.
* After we each solved the tasks, we discussed our implementations and chose the best approach.

### Task 3:
* We worked together to complete Task 3.
* Each person filled in any missing code blocks.
* We tested the code for bugs and edge cases.
* We discussed our findings and optimized the code as needed.

We each contributed equally to the project (33.33% each). We followed a workflow that involved solving Task 1 and Task 2 independently, discussing our implementations, and working together to complete Task 3. We tested the code for bugs and edge cases, and we discussed our findings and optimized the code as needed.

## License 
MIT