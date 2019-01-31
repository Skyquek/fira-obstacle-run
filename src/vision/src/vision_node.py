#!/usr/bin/env python

from collections import OrderedDict

import cv2 as cv
import rospy

from cv_mat import CVMat
from obstacle import BoundaryLine, Obstacle
from obstacle_detection import ObstacleDetector
import opencv_gui as gui
from configuration import Configuration


def get_region_colour_space_values_cb(event, x, y, _frame):
    global colour_space_roi, configuration, selected_obstacle

    # Retrieve average values for colour space
    if event == cv.EVENT_LBUTTONDOWN:
        colour_space_roi = (x, y)
    elif event == cv.EVENT_LBUTTONUP:
        roi_mean = cv.mean(_frame[colour_space_roi[1]:y, colour_space_roi[0]:x])
        selected_obstacle.set_colour_space_value(roi_mean[:-1])
        configuration.update(selected_obstacle.name, selected_obstacle.export_configuration())
        colour_space_roi = None


def set_colour_space_threshold_cb(threshold):
    global selected_obstacle, configuration

    # Change colour space threshold for selected object
    selected_obstacle.set_colour_space_threshold(threshold=threshold)
    configuration.update(selected_obstacle.name, selected_obstacle.export_configuration())


def switch_selected_obstacle_cb(value):
    global selected_obstacle, obstacles_list

    # Switch the tuning object
    selected_obstacle = obstacles_list.values()[value]
    colour_space_threshold = selected_obstacle.threshold
    gui.set_colour_space_threshold_trackbar_position(colour_space_threshold)


def track_obstacle(obstacle, obstacle_mat):
    global obstacles_list, output, selected_obstacle

    # Extract obstacle from frame
    obstacle_mat.extract_obstacle(thresh_lb=obstacle.lower_bound, thresh_ub=obstacle.upper_bound)

    # Find the obstacle using bounding rectangle, draw any if found
    positions, position_text, area = obstacle_mat.bounding_rect(min_area=obstacle.min_area, max_area=obstacle.max_area,
                                                                frame_height=FRAME_HEIGHT, frame_width=FRAME_WIDTH)

    # Draw rectangle and publish ROS message
    if positions is not None:
        for tl_pt, br_pt in positions:
            cv.rectangle(output.frame, tl_pt, br_pt, obstacle.output_colour, thickness=2)
    obstacle.publish_msg(position_text, area)

    # Check if the obstacle is selected, show in debug if selected
    if selected_obstacle == obstacles_list[obstacle.name]:
        gui.show_debug_window(obstacle_mat.frame, obstacle.name)


if __name__ == '__main__':
    # Load configurations
    configuration = Configuration(configuration_directory=rospy.get_param('vision_node/configuration_directory'))
    configuration.load()
    colour_space_roi = None

    # Initialize ROS
    rospy.init_node('vision_node')
    rate = rospy.Rate(10)

    # Initialize obstacles/lines and load configurations
    blue_obstacle = Obstacle(name='blue_obstacle', configuration=configuration.config['blue_obstacle'])
    red_obstacle = Obstacle(name='red_obstacle', configuration=configuration.config['red_obstacle'])
    close_yellow_obstacle = Obstacle(name='yellow_obstacle', configuration=configuration.config['yellow_obstacle'])
    obstacles_list = OrderedDict([('blue_obstacle', blue_obstacle), ('red_obstacle', red_obstacle),
                                  ('yellow_obstacle', close_yellow_obstacle)])
    selected_obstacle = blue_obstacle

    # Create GUI and set callbacks
    gui.set_mouse_cb('camera', lambda event, x, y, flags, params:
                     get_region_colour_space_values_cb(event, x, y, original.colour_space_frame))
    gui.create_trackbar(gui.DEBUG_WINDOW_NAME, trackbar_name='Obstacles', default_value=0,
                        max_value=len(obstacles_list.values()) - 1, callback=switch_selected_obstacle_cb)
    gui.create_colour_space_threshold_trackbar(selected_obstacle.threshold, set_colour_space_threshold_cb)

    # Open camera device
    cap = cv.VideoCapture(configuration.config['camera_index'])
    original_height, original_width = cap.read()[1].shape[:2]
    FRAME_HEIGHT = configuration.config['resized_frame_height']
    FRAME_WIDTH = int(round(FRAME_HEIGHT * (original_width / float(original_height))))
    del original_height, original_width

    try:
        while not rospy.is_shutdown():
            ret_code, raw_image = cap.read()

            if raw_image.data:
                original = CVMat(raw_image, height=FRAME_HEIGHT)

                # Create an output frame from a clone of the original
                output = original.clone()

                # Extract obstacle/boundary line from field
                blue_mat = ObstacleDetector(frame=original.frame)
                track_obstacle(obstacle=blue_obstacle, obstacle_mat=blue_mat)
                red_mat = ObstacleDetector(frame=original.frame)
                track_obstacle(obstacle=red_obstacle, obstacle_mat=red_mat)
                close_yellow_mat = ObstacleDetector(frame=original.frame)
                track_obstacle(obstacle=close_yellow_obstacle, obstacle_mat=close_yellow_mat)

                # Display GUI windows
                gui.show('camera', original.frame)
                gui.show('output', output.frame)

            cv.waitKey(1)
    except rospy.ROSInterruptException:
        # Clean up
        gui.teardown()
        cap.release()
