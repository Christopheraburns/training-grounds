import numpy as np
import rospy
from sensor_msgs.msg import LaserScan


class LidarReader(object):
    def __init__(self):
        rospy.Subscriber("/scan", LaserScan,self.scan_callback)
        self.ranges = None


    def scan_callback(self, data):
        self.ranges = data.ranges
        self.do_funky_math(self.ranges)


    def do_funky_math(self, ranges):
        TRAINING_IMAGE_SIZE = 360
        LIDAR_SCAN_MAX_DISTANCE = 5.0
        size = len(ranges)
        x = np.linspace(0, size -1, TRAINING_IMAGE_SIZE)

        #print("LinSpace command: {}".format(x))

        xp = np.arange(size)
        #print("ARange command: {}".format(xp))

        interp = np.interp(x, xp, ranges)
        #print("Interpolation: {}".format(interp))

        state = np.clip(interp, 0, LIDAR_SCAN_MAX_DISTANCE)
        #print("State = {} ".format(state))

        state[np.isnan(state)] = LIDAR_SCAN_MAX_DISTANCE

        min_distance = np.amin(state)

        print("Min distance: {}".format(min_distance))

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('lidar_reader', anonymous=True)
    reader = LidarReader()
    reader.run()