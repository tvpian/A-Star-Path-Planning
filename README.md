# ENPM 661 - Project 3 - Phase

## Dependencies

* python3 interpreter
* Python packages - numpy, time, matplotlib, copy, math, collections, queue, opencv, readline

## Execution

* **A\* Solver** 
    
    - To execute the program for this problem, navigate to the submission folder and use the following commands
        ```
        cd <path_to_submission>/code/
        python3 a_star_Abhijay_Tharun.py
        ```

    - Once you run the program, enter the start and goal states in the format shown below:
        ```
        Enter start state (x y theta) : 11 11 0
        Enter end state (x y) : 50 50
        Enter clearance and radius (clearance, radius) : 5 5
        Enter RPM1 and RPM2 (RPM1, RPM2) : 1000 1000
        ```

    - Each line of input should be in the format x y theta where x, y and theta are the x and y coordinates of the point and theta is the orientation of the robot respectively.

    - The program will not run if only one of the coordinates is entered or if the coordinates are not integers.

    - The program will not run if the clearance and radius are not entered in the format clearance, radius.

    - Since the quiver plot in matplotlib is not very efficient, the animation can be played in three ways.

    - The first method is using OpenCV. Please use the function in line 51 of the a_star_Abhijay_Tharun.py file. (Suggested method) To record the animation, please set the record variable to True in the visualizer.record_opencv() function in the a_star_Abhijay_Tharun.py file (line 51).

    - The second method is using matplotlib scatter plot. Please use the function in line 50 of the a_star_Abhijay_Tharun.py file.

    - The third method is using matplotlib quiver plot. Please uncomment lines 99-103 and comment lines 103-104 in the visualizer.py file. (Not recommended)

    - To increase the speed of the animation, change the value of step_size in the visualizer.plot() and visualizer.record_opencv() functions in the a_star_Abhijay_Tharun.py file (lines 50/51).

## Results
* **Dijkstra's Solver** 
    
    - The outputs is as shown below:
        ```
        Enter start state (x y theta) : 125 125 0
        Enter end state (x y theta) : 11 11 0
        Enter clearance and radius (clearance, radius) : 5 5
        Enter RPM1 and RPM2 (RPM1, RPM2) : 1000 1000

        Starting search...

        Found path in 2.6980650424957275 seconds.
        Distance from state to goal :  76193.22465109696
        Final node in the path : [48.76812248 49.25284795 37.09564207]
        Number of nodes explored : 3870
        ```
    - The animation video link is [here](https://drive.google.com/drive/folders/19rUXHqT4XGNATDRHBpmevm8oGCOTXusi?usp=sharing).

## Help
For any assistance with executing the programs or questions related to them, please reach out to us at abhijay@umd.edu or tvpian@umd.edu.
