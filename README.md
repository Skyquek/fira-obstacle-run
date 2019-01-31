# Obstacle Run

This is the obstacle run event for the FIRA HuroCup. It consists of two modules, the logic and the vision. The vision module is adapted from Assignment 1 with some minor bug fixes. The logic module makes decisions based on patterns it recognizes from the vision.


## Getting Started

### Requirements

This event has the following requirements:

1. OpenCV 3.2+
2. Python 2.7
3. Darwin-OP3 and its framework

### Features

1. Image based visual servoing (IBVS)

### How to run the project

1. Copy the files over to Darwin-OP 3

2. Use the ROS launch file to run the event
```bash
roslaunch obstacle_run obstacle_run.launch
```
3. Press the `mode` button on the Darwin-OP 3 to reset the event.
4. Press the `start` button on the Darwin-OP 3 to start the event.

The vision can be tuned via the GUI. To start, select the desired object to tune using the `Object` trackbar in the `debug` window. Change the average colour space value by selecting a range in the `camera` window. Modify the threshold values via the other three trackbars.

### Configuration

#### Vision module

The vision module has a configuration file under `config/configuration.json`. Use this file to tune the vision based on the environment it is used in.

The configuration file gives the ability to change the setting of the
camera image and the detected features.

Camera:

- camera_index: the index of the video camera device (default: 0)
- resized_frame_height: height of the resized frame, useful
    to reduce processing load (default: 320)

Obstacles:

- min_area: minimum area of contours (default: 5000)
- max_area: maximum area of contours (default: 100000)
- output_colour: line colour of the object in the output frame (default: obstacle colour)
- threshold: value added to calculate the min/max rnage of the colour space (default: [0, 0, 0])
- value: average value of the selected range in the colour space (default: [0, 0, 0])

#### Logic module

The vision module has a configuration file under `config/configuration.json`. Use this file to tune the vision based on the environment it is used in.

The configuration file gives the ability to change the setting of the
camera image and the detected features.


The logic module has two configuration files under `config/`: `action_configurations.json` and `walking_configurations.json`. Use these files to change the action and walking configurations based on the environment.

**action_configurations.json**: This configuration file lists the pages of the crawl motion used in obstacle run.

**walking_configurations.json**: This configuration file lists the walking gaits for each different type of walk.


## Future additions
- [] Simultaneous localization and mapping (SLAM)  
