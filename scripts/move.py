#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

class MoveRobot():

    def __init__(self):
        self.robot_vel_publisher = rospy.Publisher('/cmd/vel', Twist, queue_size=1)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(10)  #10hz


    def publish_once_in_cmd_vel(self, cmd):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems it is not a big deal
        """
        while not self.ctrl_c:
            connections = self.robot_vel_publisher.get_num_connections()
            if connections > 0:
                self.robot_vel_publisher.publish(cmd)
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        self.stop_robot()
        self.ctrl_c = True


    def stop_robot(self):
        rospy.loginfo("Shutting down")
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel(cmd)

    def move_x_time(self, moving_time, linear_speed=0.2, angular_speed=0.2):
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed

        rospy.loginfo('Moving Forwards')
        self.publish_once_in_cmd_vel(cmd)
        time.sleep(moving_time)
        self.stop_robot()
        rospy.loginfo(" Finished moving forwards")

    def move_square(self):

        i = 0
        while not self.ctrl_c and i < 4:
            # Move Forward
            self.move_x_time(moving_time=2.0, linear_speed=0.2, angular_speed=0.0)
            #stop
            self.move_x_time(moving_time=4.0, linear_speed=0.0, angular_speed=0.0)
            # Turn
            self.move_x_time(moving_time=3.5, linear_speed=0.0, angular_speed=0.2)
            # stop
            self.move_x_time(moving_time=0.1, linear_speed=0.0, angular_speed=0.0)

            i += 1
        rospy.loginfo("###### Finished moving in a square")

    def move_one_meter(self):
        #Move Forwards
        self.move_x_time(moving_time=2.3, linear_speed=0.2, angular_speed=0.0)
        #Stop
        self.move_x_time(moving_time=4.0, linear_speed=0.0, angular_speed=0.0)
        #Move Forward
        self.move_x_time(moving_time=2.3, linear_speed=0.2, angular_speed=0.0)
        # Stop
        self.move_x_time(moving_time=4.0, linear_speed=0.0, angular_speed=0.0)

    def turn_90_degrees(self):
        # Turn
        self.move_x_time(moving_time=4.0, linear_speed=0.0, angular_speed=0.2)
        # Stop
        self.move_x_time(moving_time=0.1, linear_speed=0.0, angular_speed=0.0)


    def movement_distance_test(self):
        # MOVE in Y_AXIS
        rospy.loginfo("Moving in Y AXIS")
        self.move_one_meter()
        # TURN 90 degrees
        rospy.loginfo("Turnin 90 degrees")
        self.turn_90_degrees()
        # Move in X Axis
        rospy.loginfo("Moving in X Axis")
        self.move_one_meter()

if __name__ == '__main__':
    rospy.init_node('move_robot_test', anonymous=True)
    moveRobo_object = MoveRobot()
    try:
        moveRobo_object.movement_distance_test()
    except rospy.ROSInterruptException:
        pass