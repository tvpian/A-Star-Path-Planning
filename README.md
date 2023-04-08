# ENPM 661 - Project 3 - Phase 2

## Team Members
* Abhijay (UID - 118592619)
* Tharun Puthanveettil (UID - 119069516)

## Dependencies

* python3 interpreter
* Python packages - numpy, time, matplotlib, copy, math, collections, queue, opencv, readline
* ROS - noetic, turtlebot3(needs to be installed), gazebo_ros

## Execution

* **Part 01 - A\* Solver** 
    
    - To execute the program for this problem, navigate to the submission folder and use the following commands
        ```
        cd <path_to_submission>/code/
        python3 a_star_Abhijay_Tharun.py
        ```

    - Once you run the program, enter the start and goal states in the format shown below:
        ```
        Enter start state (x y theta) : 50 100 0
        Enter end state (x y) : 300 180
        Enter clearance (clearance) : 5
        Enter RPM1 and RPM2 (RPM1, RPM2) : 30 60
        ```

    - Each line of input should be in the format x y theta where x, y and theta are the x and y coordinates of the point and theta is the orientation of the robot respectively.

    - The program will not run if only one of the coordinates is entered or if the coordinates are not integers.

    - The program will not run if the clearance and radius are not entered in the format clearance, radius.

* **Part 02 - ROS**

    - Enter the following command to launch the ROS node:
        ```
        roslaunch a_star_solver astar_map.launch
        ```

    - Press the enter key once to display the options for the program.
    
    - Once you run the program, enter the start and goal states in the format shown below:
        ```
        Enter start state (x y theta) : 0 0 0
        Enter end state (x y) : 5 0
        Enter clearance (clearance) : 5
        Enter RPM1 and RPM2 (RPM1, RPM2) : 30 60
        ```
    - The inputs for the coordinates should be with respect to the Gazebo world frame. 

## Results
* **A\* Solver** 
    
    - The outputs is as shown below:
        ```
        Enter start state (x y theta) : 50 100 0
        Enter end state (x y) : 300 180
        Enter clearance (clearance) : 5
        Enter RPM1 and RPM2 (RPM1, RPM2) : 30 60

        Starting search...

        Found path in 1.832632303237915 seconds.
        Distance from state to goal :  3.4211943997592833
        Final node in the path : [299.95249664 181.2539106   65.34      ]
        Number of nodes explored : 1829
        ```
    - The animation video link is [here](https://drive.google.com/drive/folders/19rUXHqT4XGNATDRHBpmevm8oGCOTXusi?usp=sharing).

* **ROS**
    - The outputs is as shown below:
        ```
        Enter start state (x y theta) : 50 100 0
        Enter end state (x y) : 550 100
        Enter clearance (clearance) : 5
        Enter RPM1 and RPM2 (RPM1, RPM2) : 50 100

        Starting search...

        Found path in 0.5591976642608643 seconds.
        Distance from state to goal :  6.4622560884342075
        Final node in the path : [549.42340336 100.48810098 251.1       ]
        Number of nodes explored : 593
        ```
## Help
For any assistance with executing the programs or questions related to them, please reach out to us at abhijay@umd.edu or tvpian@umd.edu.
