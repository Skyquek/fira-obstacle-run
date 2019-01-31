import numpy as np

import rospy
from vision.msg import PositionArea


class Obstacle(object):

    def __init__(self, name, configuration):
        self.pub = rospy.Publisher('/vision_node/' + name, PositionArea, queue_size=1)

        self.name = name

        self.threshold = np.array(configuration['threshold'])
        self.value = np.array(configuration['value'])
        self.lower_bound, self.upper_bound = self.get_colour_space_bounds()

        self.min_area = configuration['min_area']
        self.max_area = configuration['max_area']

        self.output_colour = configuration['output_colour']

    def get_colour_space_bounds(self):
        threshold_lower_bound = []
        threshold_upper_bound = []

        for index in range(0, len(self.value)):
            threshold_lower_bound += [self.round_int(self.value[index] - self.threshold[index])]
            threshold_upper_bound += [self.round_int(self.value[index] + self.threshold[index])]

        return np.array(threshold_lower_bound), np.array(threshold_upper_bound)

    def set_colour_space_value(self, values):
        self.value = np.array([self.round_int(value) for value in values])
        self.lower_bound, self.upper_bound = self.get_colour_space_bounds()

    def set_colour_space_threshold(self, threshold):
        self.threshold = np.array([self.round_int(value) if value >= 0 else self.threshold[index]
                                   for index, value in enumerate(threshold)])
        self.lower_bound, self.upper_bound = self.get_colour_space_bounds()

    def export_configuration(self):
        return {
            'threshold': self.threshold, 'value': self.value,
            'min_area': self.min_area, 'max_area': self.max_area,
            'output_colour': self.output_colour
        }

    def publish_msg(self, position, area):
        message = PositionArea()
        message.position = position
        message.area = area
        self.pub.publish(message)

    @staticmethod
    def round_int(num):
        return max(0, int(round(num)))


class BoundaryLine(Obstacle):

    def __init__(self, configuration):
        Obstacle.__init__(self, name='boundary_line', configuration=configuration)
