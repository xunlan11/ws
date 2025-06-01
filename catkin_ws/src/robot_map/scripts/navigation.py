import rospy
import math
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

ACTION_MOVE_BASE = 'move_base'
FRAME_MAP = 'map'
FRAME_BASE = 'base_frame'

class NavToPoint:
    def __init__(self):
        rospy.on_shutdown(self.clean_up)
        self.move_base = actionlib.SimpleActionClient(ACTION_MOVE_BASE, MoveBaseAction)
        rospy.loginfo('Waiting for move_base action server ...')
        self.move_base.wait_for_server(rospy.Duration(120))
        rospy.loginfo('Connected to move_base server')
        self.map_robot_pose = [0, 0, 0]
        self.move_base_running = False
        self.blocking = True
        rospy.loginfo('Ready to go ...')
        rospy.sleep(1)
    
    def goto(self, target, blocking=True):
        self.blocking = blocking
        yaw = target[2] / 180.0 * math.pi
        quaternion = quaternion_from_euler(0.0, 0.0, yaw)
        target_pose = Pose(Point(target[0], target[1], target[2]), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
    
        self.goal = MoveBaseGoal()
        rospy.loginfo('Starting navigation test...')
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo('Going to the target...')
        rospy.sleep(2)
        self.goal.target_pose.pose = target_pose
        self.move_base.send_goal(self.goal)
        if blocking:
            waiting = self.move_base.wait_for_result()
            self.move_base_running = False
            if not waiting:
                rospy.logerr('Action server not available!')
            else:
                rospy.loginfo('Navigation result: %s' % self.move_base.get_result())
    def clean_up(self):
        rospy.loginfo('Shutting down navigation ...')

if __name__ == '__main__':
    rospy.init_node('nav_to_point')
    nav = NavToPoint()
    nav.goto([3, 0, 90])