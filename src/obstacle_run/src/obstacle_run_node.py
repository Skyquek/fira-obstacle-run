#!/usr/bin/env python

from enum import Enum
from time import sleep

import rosnode
import rospy
from std_msgs.msg import String

from action_module import ActionModule
from head_control_module import HeadControlModule
from obstacles_pa import ObstaclesWithPA
from vision.msg import PositionArea
from walking_module import WalkingModule


class RobotMode(Enum):
    READY = 0
    RUNNING = 1


class Movements(Enum):
    MARCH = 0
    FORWARD = 1
    BACKWARD = 2
    TURN_LEFT = 3
    TURN_RIGHT = 4
    ACTION = 5


def button_callack(msg):
    global robot_mode, mode_change

    if msg.data == 'start':
        # Start button pressed
        # Send robot to RUNNING mode
        if robot_mode == RobotMode.READY:
            robot_mode = RobotMode.RUNNING
            mode_change = True
        elif robot_mode == RobotMode.RUNNING:
            rospy.logerr('Robot already in running mode.')

    elif msg.data == 'mode':
        # Mode button pressed
        # Send robot to READY mode
        robot_mode = RobotMode.READY
        mode_change = True


def wait_for_node(node_name):
    # Wait for node to start
    while node_name not in rosnode.get_node_names():
        rospy.logwarn('Node is not running: {0}'.format(node_name))
        sleep(1)

    rospy.loginfo('Node is running: {0}'.format(node_name))


def avoid_obstacles():
    global mode_change, action_module, head_control_module, walking_module, obstacles, rate

    walking_module.enable()
    walking_module.march()
    head_control_module.enable()
    head_control_module.look_down()
    sleep(3)

    previous_movement = Movements.MARCH
    current_movement = None
    yellow_prefer_left = True
    num_steps_since_yellow = 0
    num_steps_yellow_turn = 0
    blue_prefer_left = True


    while not rospy.is_shutdown() and not mode_change:

        if obstacles.red.is_on_left or obstacles.red.is_center or obstacles.red.is_on_right:
            # RED is ahead
            num_steps_since_yellow += 1
            num_steps_yellow_turn = 0
            obstacles.yellow.is_last_seen = False
            obstacles.blue.is_last_seen = False


            if obstacles.blue.is_on_left or (obstacles.red.is_on_right and obstacles.red.area < 3000):
                # RED is to the right side
                current_movement = Movements.TURN_RIGHT
            elif obstacles.blue.is_on_right or (obstacles.red.is_on_left and obstacles.red.area < 3000):
                # RED is to the left side
                current_movement = Movements.TURN_LEFT
            else:
                # Red obstacle --> get down and start crawling
                current_movement = Movements.ACTION

        elif obstacles.yellow.is_center:
            # YELLOW is ahead
            num_steps_since_yellow = 0
            obstacles.blue.is_last_seen = False

            if num_steps_yellow_turn == 3:
                # Keeps seeing YELLOW, most likely a sideline
                yellow_prefer_left = not yellow_prefer_left
                obstacles.yellow.is_last_seen = True
                current_movement = Movements.TURN_LEFT if yellow_prefer_left else Movements.TURN_RIGHT
            if not obstacles.yellow.is_last_seen and \
                    obstacles.yellow.is_on_left and not obstacles.yellow.is_on_right:
                # YELLOW on left
                current_movement = Movements.TURN_RIGHT
                yellow_prefer_left = False
            elif not obstacles.yellow.is_last_seen and \
                    not obstacles.yellow.is_on_left and obstacles.yellow.is_on_right:
                # YELLOW on right
                current_movement = Movements.TURN_LEFT
                yellow_prefer_left = True
            else:
                num_steps_yellow_turn += 1

                # Turn based on opposite of preference
                if not obstacles.yellow.is_last_seen:
                    yellow_prefer_left = not yellow_prefer_left
                    obstacles.yellow.is_last_seen = True
                current_movement = Movements.TURN_LEFT if yellow_prefer_left else Movements.TURN_RIGHT

        elif obstacles.blue.is_center:
            # BLUE is ahead
            num_steps_since_yellow += 1
            num_steps_yellow_turn = 0
            obstacles.yellow.is_last_seen = False

            if num_steps_since_yellow <= 10:
                # Based on yellow preference
                current_movement = Movements.TURN_LEFT if yellow_prefer_left else Movements.TURN_RIGHT
            else:
                if not obstacles.blue.is_last_seen and obstacles.blue.is_on_left and not obstacles.blue.is_on_right:
                    # BLUE on left
                    current_movement = Movements.TURN_RIGHT
                    blue_prefer_left = False
                elif not obstacles.blue.is_last_seen and not obstacles.blue.is_on_left and obstacles.blue.is_on_right:
                    # BLUE on right
                    current_movement = Movements.TURN_LEFT
                    blue_prefer_left = True
                else:
                    # Turn based on preference derived from old pattern
                    obstacles.blue.is_last_seen = True
                    current_movement = Movements.TURN_LEFT if blue_prefer_left else Movements.TURN_RIGHT

        else:
            # Path is clear --> proceed
            num_steps_since_yellow += 1
            num_steps_yellow_turn = 0
            obstacles.yellow.is_last_seen = False
            obstacles.blue.is_last_seen = False

            current_movement = Movements.FORWARD

        # Execute movement
        if previous_movement != current_movement:
            if current_movement == Movements.MARCH:
                walking_module.march()
            elif current_movement == Movements.FORWARD:
                walking_module.forward()
            elif current_movement == Movements.BACKWARD:
                walking_module.backward()
            elif current_movement == Movements.TURN_LEFT:
                walking_module.turn_left()
            elif current_movement == Movements.TURN_RIGHT:
                walking_module.turn_right()
            elif current_movement == Movements.ACTION:
                walking_module.backward()
                sleep(5)
                action_module.enable()
                action_module.play_crawl()
                walking_module.enable()
                walking_module.march()
                head_control_module.enable()
                head_control_module.look_down()
                sleep(3)

            previous_movement = current_movement

        rate.sleep()
        

if __name__ == '__main__':

    # Initalize global variables
    robot_mode = RobotMode.READY
    mode_change = False
    obstacles = ObstaclesWithPA()
    SPIN_RATE = 0.5

    # Wait for other nodes to launch
    wait_for_node('/op3_manager')
    wait_for_node('/vision_node')

    # Initialze ROS node
    rospy.init_node('obstacle_run_node')
    rate = rospy.Rate(SPIN_RATE)

    # Initialize OP3 ROS generic subscriber
    button_sub = rospy.Subscriber('/robotis/open_cr/button', String, button_callack)

    # Initialze vision subscribers
    blue_sub = rospy.Subscriber('/vision_node/blue_obstacle', PositionArea,
                                lambda msg: obstacles.blue.update(msg.position, msg.area))
    red_sub = rospy.Subscriber('/vision_node/red_obstacle', PositionArea,
                               lambda msg: obstacles.red.update(msg.position, msg.area))
    yellow_sub = rospy.Subscriber('/vision_node/yellow_obstacle', PositionArea,
                                        lambda msg: obstacles.yellow.update(msg.position, msg.area))

    # Initializing modules
    action_module = ActionModule()
    head_control_module = HeadControlModule()
    walking_module = WalkingModule()

    # Enabling walking module
    sleep(10)
    rospy.loginfo('Press start button to begin.')

    while not rospy.is_shutdown():
        if mode_change:
            mode_change = False

            if robot_mode == RobotMode.READY:
                rospy.loginfo('Resetting OP3. Press start button to begin.')

                walking_module.enable()
                walking_module.stop()
                head_control_module.enable()
                head_control_module.look_down()
            
            elif robot_mode == RobotMode.RUNNING:
                rospy.loginfo('Starting obstacle run event.')

                avoid_obstacles()

        rate.sleep()



