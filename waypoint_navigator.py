import rospy
import json
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion

class WaypointsNavigator:
    def __init__(self):
        self.waypoints = self.load_waypoints('/home/shrey/catkin_ws/src/Tasks/aruco_marker_teleop/waypoints.json')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server")
        self.navigate_waypoints()

    def load_waypoints(self, filename):
        with open(filename, 'r') as file:
            data = json.load(file)
            return data['waypoints']

    def navigate_waypoints(self):
        for waypoint in self.waypoints:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = waypoint['pose']['position']['x']
            goal.target_pose.pose.position.y = waypoint['pose']['position']['y']
            goal.target_pose.pose.position.z = waypoint['pose']['position']['z']

            # Set a default orientation if not provided
            goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)

            rospy.loginfo(f"Navigating to waypoint {waypoint['id']}")
            self.client.send_goal(goal)
            finished_within_time = self.client.wait_for_result(rospy.Duration(60))  # Wait for 60 seconds

            if not finished_within_time:
                self.client.cancel_goal()
                rospy.logwarn(f"Timed out reaching waypoint {waypoint['id']}")
            else:
                state = self.client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo(f"Reached waypoint {waypoint['id']}")
                else:
                    rospy.logwarn(f"Failed to reach waypoint {waypoint['id']} with state {state}")

if __name__ == '__main__':
    rospy.init_node('waypoint_navigation')
    try:
        navigator = WaypointsNavigator()
    except rospy.ROSInterruptException:
        rospy.logerr("Navigation interrupted.")
