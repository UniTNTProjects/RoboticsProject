# Robotics project for University of Trento 

Recognition of lego-like blocks, detection of the blocks and their position in the space, and planning of a path to reach the blocks and bring them to a desired location.

## Authors

Emanuele Ricca, Emanuele Maccacaro, Jacopo Tomelleri

## Project Report

In the [project report](./report.pdf) can be found a detailed description of the project, the details of the implementation and the results obtained.

## Example Video

The following links are videos of the project in action during assignment 1 and 2.

- [Assignment 1](https://youtu.be/OmyCqnIY6ZQ)
- [Assignment 2](https://youtu.be/MqdqDNI8kXU)


## How to run the project

- Clone the locosim enviroment (available at this [link](https://github.com/mfocchi/locosim/tree/659f15fe13895336c0cb11469ef34e747bd84c7f#native-installation-on-linux)) in the ros workspace.

- Clone this repository in the locosim folder.


```bash
git clone https://github.com/UniTNTProjects/RoboticsProject.git

```

- Build the project

```bash
catkin build
```

- Source the new environment

```bash
source devel/setup.bash
```

- Run the simulation

```bash
python3 ./RoboticsProject/ur5_generic.py
```

- Launch the desired assignment (e.g. assignment 1)

```bash
roslaunch robot_projcet simulation_assignment1.launch
```

- Launch the FSM

```bash
rosrun robot_project FSM_Main
```




