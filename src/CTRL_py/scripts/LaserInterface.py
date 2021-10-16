import rospy
from sensor_msgs.msg import LaserScan


class LaserInterface:

    def __init__(self):
        """initialize the laser interface. Add subscriber.
        """
        self.scanned_data = []
        self.intensities = []

        self.laserSub = rospy.Subscriber('/project_ctrl/laser/scan', LaserScan, self.__laser_callback)

    def __laser_callback(self, msg):
        """Callback function for process the feedback from laser scanner
        """
        self.scanned_data = msg.ranges
        self.intensities = msg.intensities
