#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

class MyRobot:

    def __init__(self):
        rospy.init_node('move_to_coordinates', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("Arm_group")

    def move_to_coordinates(self, x, y, z, roll, pitch, yaw):
        # Convert Euler angles to quaternion
        quaternion = quaternion_from_euler(roll, pitch, yaw)

        # Define target position and orientation
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.x = quaternion[0]
        target_pose.orientation.y = quaternion[1]
        target_pose.orientation.z = quaternion[2]
        target_pose.orientation.w = quaternion[3]

        # Set target pose
        self.group.set_pose_target(target_pose)

        # Plan and execute motion
        plan = self.group.go(wait=True)
        #self._group.stop()
        #self._group.clear_pose_targets()

        rospy.loginfo("Moved to coordinates (x={}, y={}, z={}) with orientation (roll={}, pitch={}, yaw={})".format(x, y, z, roll, pitch, yaw))

    def print_end_effector_pose(self):
        current_pose = self.group.get_current_pose().pose
        rospy.loginfo("End effector pose: {}".format(current_pose))

def main():
    arm = MyRobot()

    if len(sys.argv) != 5:
        rospy.logerr("Usage: rosrun package_name script_name.py x y z roll pitch yaw")
        return

    target_x = float(sys.argv[1])
    target_y = float(sys.argv[2])
    target_z = float(sys.argv[3])
    roll = float(sys.argv[4])
    pitch = float(sys.argv[5])
    yaw = float(sys.argv[6])

    arm.move_to_coordinates(target_x, target_y, target_z, roll, pitch, yaw)
    arm.print_end_effector_pose()

if __name__ == '__main__':
    main()
