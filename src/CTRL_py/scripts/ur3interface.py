from ur3_driver.msg import command
from ur3_driver.msg import position
from ur3_driver.msg import gripper_input
import rospy
import rospkg


class UR3Interface:
    """
    @brief init the interface, including register the callback functions and initialize variables.
    """

    def __init__(self, SPINRATE_=20):
        self.angle_feedback = [0, 0, 0, 0, 0, 0]
        self.gripper_feedback = False
        self.target_reached = True

        self.target_angle = [0, 0, 0, 0, 0, 0]
        self.target_sprayer_state = False
        self.target_speed = 0.0
        self.target_acceleration = 0.0

        self.cmd = rospy.Publisher('ur3/command', command, queue_size=10)
        self.position = rospy.Subscriber('ur3/position', position, self.__pos_callback)
        self.gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, self.__gripper_callback)
        self.SPINRATE = SPINRATE_

    """
    @brief callback function for 6 joints' angle feedback
    """

    def __pos_callback(self, msg):
        for i in range(0, 6):
            self.angle_feedback[i] = msg.position[i]

    """
    @brief callback function for laser's feedback
    """

    # TODO: modified the callback function for laser

    def __gripper_callback(self, msg):
        self.gripper_feedback = msg.DIGIN

    """
    @brief Set gripper of UR3
    @param gripper_state_ (True/False) whether the gripper is on or off
    """

    def set_sprayer(self, sprayer_state_):
        self.target_sprayer_state = sprayer_state_

    def set_angle(self, target_angle_, speed_, acceleration_):
        # for i in range(0, 6):
        #     if abs(self.target_angle[i] - self.angle_feedback[i]) > 0.0005:
        #         return False
        for i in range(0, 6):
            self.target_angle[i] = target_angle_[i]
        if speed_ <= 0.0:
            return

        self.target_speed = speed_
        self.target_acceleration = acceleration_
        self.__publish_variables()
        self.target_reached = False
        spin_count = 0
        while not self.target_reached:
            self.target_reached = (abs(self.target_angle[0] - self.angle_feedback[0]) < 0.0005) and \
                                  (abs(self.target_angle[1] - self.angle_feedback[1]) < 0.0005) and \
                                  (abs(self.target_angle[2] - self.angle_feedback[2]) < 0.0005) and \
                                  (abs(self.target_angle[3] - self.angle_feedback[3]) < 0.0005) and \
                                  (abs(self.target_angle[4] - self.angle_feedback[4]) < 0.0005) and \
                                  (abs(self.target_angle[5] - self.angle_feedback[5]) < 0.0005)

            if spin_count > self.SPINRATE * 5:
                self.__publish_variables()
                spin_count = 0
            spin_count = spin_count + 1


    def __publish_variables(self):
        ur3_msg = command()
        ur3_msg.destination = self.target_angle
        ur3_msg.v = self.target_speed
        ur3_msg.a = self.target_acceleration
        ur3_msg.io_0 = self.target_sprayer_state
        self.cmd.publish(ur3_msg)
