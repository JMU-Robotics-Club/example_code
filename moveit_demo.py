"""Demo of using moveit_commander to control the turtlebot arm. 

Some of this code is from: 
/opt/ros/indigo/lib/turtlebot_arm_moveit_demos/pick_and_place.py

Refer to that file for a more extensive example. 

First: 

$ roslaunch turtlebot_arm_block_manipulation block_manipulation_moveit.launch

Then: 

Usage: 

python moveit_demo.py X_GOAL Y_GOAL Z_GOAL PITCH_GOAL YAW_GOAL

Positions are in meters from the base of the arm, angles are in
radians.

"""


import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation
from moveit_msgs.msg import MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

def make_goal(x, y, z, pitch, yaw):
    goal = PoseStamped()
    goal.header.frame_id = 'arm_base_link'
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z
    new_quat = quaternion_from_euler(0, pitch, yaw)
    goal.pose.orientation.x = new_quat[0]
    goal.pose.orientation.y = new_quat[1]
    goal.pose.orientation.z = new_quat[2]
    goal.pose.orientation.w = new_quat[3]
    return goal


def main():
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    pitch_goal = float(sys.argv[4])
    yaw_goal = float(sys.argv[5])
    

    rospy.init_node('moveit_demo')

    # Initialize the arm. 
    arm = MoveGroupCommander('arm')
    arm.set_goal_position_tolerance(0.01)
    arm.set_goal_orientation_tolerance(0.05)

    # Allow replanning to increase the odds of a solution
    arm.allow_replanning(True)
    
    # Move the arm to the resting state
    # arm.set_named_target('resting')
    # pl = arm.plan()
    # rospy.sleep(1)
    # arm.execute(pl)

    # Move the arm to the goal. 
    goal = make_goal(x, y, z, pitch_goal, yaw_goal)

    arm.set_pose_target(goal)
    pl = arm.plan()
    arm.execute(pl)
    rospy.sleep(1)
    cur_pose = arm.get_current_pose()
    print cur_pose

if __name__ == "__main__":
    main()
