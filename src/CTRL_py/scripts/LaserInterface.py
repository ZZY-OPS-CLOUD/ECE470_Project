import rospy
from sensor_msgs.msg import LaserScan


class LaserInterface:

    def __init__(self):
        self.scanned_data = []
        self.intensities = []

        laserSub = rospy.Subscriber('/project_ctrl/laser/scan', LaserScan, self.__laser_callback)

    def __laser_callback(self, msg):
        self.scanned_data = msg.ranges
        self.intensities = msg.intensities
