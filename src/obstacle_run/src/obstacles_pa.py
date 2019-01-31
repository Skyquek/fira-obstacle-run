
class ObstaclePositionArea(object):

    def __init__(self):
        self.is_on_left = False
        self.is_on_right = False
        self.is_center = False
        self.is_far = False
        self.is_close = False
        self.area = -1
        self.is_last_seen = False

    def update(self, position, area):
        self.is_far = 'far' in position
        self.is_close = 'close' in position
        self.is_on_left = 'left' in position
        self.is_center = 'center' in position
        self.is_on_right = 'right' in position
        self.area = area


class ObstaclesWithPA(object):

    def __init__(self):
        self.blue = ObstaclePositionArea()
        self.red = ObstaclePositionArea()
        self.yellow = ObstaclePositionArea()
