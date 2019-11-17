
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


class RoverReset():
    def rover_reset(self):
        rospy.wait_for_service('gazebo/set_model_state')
        self.gazebo_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Put the Rover at the initial position
        modelState = ModelState()
        modelState.pose.position.x = -8.5
        modelState.pose.position.y = -8.5
        modelState.pose.position.z = 0.65
        modelState.pose.orientation.x = 0.0174910797633
        modelState.pose.orientation.y = 0.0178586654381
        modelState.pose.orientation.z = -0.699497425461
        modelState.pose.orientation.w = 0.71419778911
        modelState.twist.linear.x = 0
        modelState.twist.linear.y = 0
        modelState.twist.linear.z = 0
        modelState.twist.angular.x = 0
        modelState.twist.angular.y = 0
        modelState.twist.angular.z = 0
        modelState.model_name = 'rover'

        self.gazebo_model_state_service(modelState)

if __name__ == '__main__':
    reset = RoverReset()
    reset.rover_reset()

