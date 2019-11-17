#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64

"""
Example Odom topic message      [nav_msgs/Odometry]:

header:                         std_msgs/header
  seq: 120809                       seq = uint32
  stamp:                            stamp = time
    secs: 1208                  
    nsecs: 100000000
  frame_id: "world"                 string
child_frame_id: "body"          string
pose:                           geometry_msgs/PoseWithCovariance
  pose:                             geometry_msgs/Pose
    position:                           geometry_msgs/Point
      x: -2.35328879445                     float64
      y: -4.02063844631                     float64
      z: 0.643086149032                     float64
    orientation:                        geometry_msgs/Quaternion
      x: -0.0680610638694                   float64  
      y: -0.0170079127095                   float64
      z: 0.83977815523                      float64
      w: -0.538378187234                    float64
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
twist:                          geometry_msgs/TwistWithCovariance
  twist:                            geometry_msgs/Twist
    linear:                             geometry_msgs/Vector3
      x: 0.000834973012654                  float64
      y: 0.000723337796847                  float64
      z: 0.00265678314809                   float64
    angular:                            geometry_msgs/Vector3
      x: 0.00337688536237                   float64
      y: -0.0075091074736                   float64
      z: 0.00272538454178                   float64
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---

Pose = position in the world of the robot at that point in time
Twist = direction of movement and acceleration of the robot at that point in time
"""

CHECKPOINT_X = 8.5
CHECKPOINT_Y = 8.5

class MovementDetector(object):
    def __init__(self):
        self.distance_travelled = Float64()
        self.distance_travelled.data = 0.0
        self.distance_to_checkpoint = Float64()
        self.distance_to_checkpoint.data = 0.0
        self.current_position = Float64()
        self.current_position.data = 0.0
        self._current_position = Point()
        self._current_position.x = 0
        self._current_position.y = 0

        self.current_position_pub = rospy.Publisher('/current_position', Point, queue_size=1)
        self.distance_moved_pub = rospy.Publisher('/distance_travelled', Float64, queue_size=1)
        self.distance_to_checkpoint_pub = rospy.Publisher('/distance_to_checkpoint', Float64, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/robot_bumper", ContactsState, self.contact_callback)

        self.get_init_position()

    def get_init_position(self):
        data_odom = None
        while data_odom is None:
            try:
                data_odom = rospy.wait_for_message("/odom", Odometry, timeout=1)
            except:
                rospy.loginfo("Current robot odom not ready yet..")
        self._current_position.x = data_odom.pose.pose.position.x
        self._current_position.y = data_odom.pose.pose.position.y

    def contact_callback(self, data):
        states = data.states
        if len(states) > 0:
            print(states[0].collision1_name)

    def odom_callback(self, msg):
        NewPosition = msg.pose.pose.position

        try:
            # Publish the position
            self.current_position_pub.publish(NewPosition)

            #Is the new position different than the old pos?
            x_diff = abs(NewPosition.x - self._current_position.x)
            y_diff = abs(NewPosition.y - self._current_position.y)

            if x_diff > .01 or y_diff > .01:
                print(NewPosition)
                # Robot moved enough to update
                dist = math.hypot(NewPosition.x - self._current_position.x, NewPosition.y - self._current_position.y)
                self.distance_travelled.data += dist
                if self.distance_travelled.data < 0.000001:
                    aux = Float64
                    aux.data = 0.0
                    self.distance_moved_pub.publish(aux)
                else:
                    self.distance_moved_pub.publish(self.distance_travelled)

                # Calculate distance to check point

                # math.sqrt(((x-x1)**2) + (y-y1)**2))
                self.distance_to_checkpoint.data = abs(math.sqrt(((NewPosition.x - 8.5)**2) +
                                                   (NewPosition.y - 8.5)**2))

                self.distance_to_checkpoint_pub.publish(self.distance_to_checkpoint.data)

                # Don't update current position if not a significant movement - call it "eventual consistency"
                self.updatecurrent_position(NewPosition)
        except Exception as err:
            print("Error in Odom_Callback: {}".format(err))

    def updatecurrent_position(self, new_position):
        self._current_position.x = new_position.x
        self._current_position.y = new_position.y

    def print_pos(self, position, dist):
        rospy.loginfo("Distance:" + str(dist) + " [X,Y]=["+str(position.x)+", "+str(position.y)+"]")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('move_robot_test', anonymous=True)
    moveRobo = MovementDetector()
    try:
        moveRobo.run()
    except rospy.ROSInterruptException:
        pass