# PD_Adaptive_Controller_Gazebo
A repo to compare the PD and adaptive controller using turtlebot3 in gazebo.

### Turtlebot3 has 3 models [burger, waffle, waffle_pi].\
### waffle_pi urdf file has been edited in which all sensors has been removed.

![Alt text](https://github.com/rishabhdevyadav/PD_Adaptive_Controller_Gazebo/blob/main/gif/Mocap.gif)

![Alt text](https://github.com/rishabhdevyadav/PD_Adaptive_Controller_Gazebo/blob/main/gif/PD.gif)

![Alt text](https://github.com/rishabhdevyadav/PD_Adaptive_Controller_Gazebo/blob/main/gif/adaptive.gif)


## How To Run

### Terminal 1:-
```bash
export TURTLEBOT3_MODEL="waffle_pi"
roslaunch turtlebot3_gazebo gazebo.launch
```


### Terminal 2:-
```bash
export TURTLEBOT3_MODEL="waffle_pi"
roslaunch turtlebot3_gazebo arena.launch
```


### Terminal 3:-
```bash
export TURTLEBOT3_MODEL="waffle_pi"
roslaunch turtlebot3_example multi_turtlebot3.launch
```

### Terminal 4:-
```bash
roscd turtlebot3_gazebo/scripts
python pid.py
```

OR
### Terminal 4:-
```bash
roscd turtlebot3_gazebo/scripts
python adaptive.py
```

### NOTE
1. arena is custom world having 2 different surface of different friction.\
  URDF file is in turtlebot3_gazebo\urdf\arena.urdf.xacro \
  Friction property is turtlebot3\urdf\arena.gazebo.xacro \
  
2. The infinity shape path is used. To reduce cost of "Search Target Index" a local short path has been used which is small slice of global path.

3. "dt" in PD controller is just a constant value. Instead os using "current_time - previous_time", give a constant value to it.
