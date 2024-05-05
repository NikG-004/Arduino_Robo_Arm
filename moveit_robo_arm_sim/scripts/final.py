#! /usr/bin/env python3

# Include the necessary libraries
import rospy
import sys
import copy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MyRobot:

    # Default Constructor
    def __init__(self):
        # Initialize the rospy node
        rospy.init_node('node_print_pose', anonymous=True)

        # Instantiate a RobotCommander object. This object is the outer-level interface to the robot
        self._robot = moveit_commander.RobotCommander()
        # Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
        self._scene = moveit_commander.PlanningSceneInterface()
        
        # Define the movegoup for the robotic 
        # Replace this value with your robot's planning group name that you had set in MoveIt Setup Assistant
        self._planning_group = "Arm_group"
        # Instantiate a MoveGroupCommander Object. This Object is an interface to the one group of joints. this interface can be used to plan and execute the motions on the robotic arm
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        
        # We create a DisplayTrajectory publisher which is used later to publish trajectories for RViz to visualize
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        
        # Get the planning frame, end effector link and the robot group names
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        # Print the info
        rospy.loginfo('\033[95m' + "Planning Frame: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[95m' + " >>> MyRobot initialization is done." + '\033[0m')

    def move_to_coordinate(self, x, y, z):
        # Create a pose target
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z

        # Set the target pose for the end-effector
        self._group.set_pose_target(pose_target)

        # Set the planning time to allow for longer planning
        self._group.set_planning_time(5)  # Adjust the planning time as needed
        
        try:
            # Plan and execute the motion
            plan = self._group.go(wait=True)
            self._group.stop()
            self._group.clear_pose_targets()

            # Print the status of the motion
            rospy.loginfo('\033[32m' + "Motion to ({}, {}, {}) completed.".format(x, y, z) + '\033[0m')
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr("MoveIt Error: {}".format(str(e)))
            rospy.logerr('\033[31m' + "Failed to execute motion to ({}, {}, {}).".format(x, y, z) + '\033[0m')

    def print_end_effector_pose(self):
        # Get the current position of end effector link
        pose_values = self._group.get_current_pose().pose

        # Convert Quaternion to Euler (Roll, Pitch, Yaw)
        q_x = pose_values.orientation.x
        q_y = pose_values.orientation.y
        q_z = pose_values.orientation.z
        q_w = pose_values.orientation.w

        # Store the quaternion position values in list
        quaternion_list = [q_x, q_y, q_z, q_w]
        # Convert the quaternion values to roll, pitch and yaw
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)

        # Print the values
        rospy.loginfo('\033[32m' + 
                                "\n" + "End-Effector ({}) Pose: \n\n".format(self._eef_link) + 
                                "x: {}\n".format(pose_values.position.x) +  "y: {}\n".format(pose_values.position.y) +    "z: {}\n\n".format(pose_values.position.z) + 
                                "roll: {}\n".format(roll) + "pitch: {}\n".format(pitch) + "yaw: {}\n".format(yaw) +
                                '\033[0m')

    def print_joint_angle_values(self):
        # Get the values of all the joint of the arm
        list_joint_values = self._group.get_current_joint_values()

        # Print the values
        rospy.loginfo('\033[32m' + "\nArm Joint Values: \n\n" +
                      "base_joint: {}\n".format(math.degrees(list_joint_values[0])) +
                      "joint_1: {}\n".format(math.degrees(list_joint_values[1])) +
                      "joint_2: {}\n".format(math.degrees(list_joint_values[2])) +
                      "joint_3: {}\n".format(math.degrees(list_joint_values[3])) +
                      '\033[0m')

    # Class Destructor
    def __del__(self):
        # When the actions are finished, shut down the moveit commander
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[95m' + "Object of class MyRobot Deleted." + '\033[0m')

def main():
    # Create a new arm object from the MyRobot class
    arm = MyRobot()

    # Check if the number of command-line arguments is correct
    if len(sys.argv) != 4:
        print("Usage: rosrun package_name script_name.py x y z")
        return

    # Extract x, y, and z coordinates from command-line arguments
    target_x = float(sys.argv[1])
    target_y = float(sys.argv[2])
    target_z = float(sys.argv[3])

    # Move the end-effector to the specified coordinates
    arm.move_to_coordinate(target_x, target_y, target_z)

    # Print end-effector pose and joint angle values
    arm.print_end_effector_pose()
    arm.print_joint_angle_values()

    del arm

if __name__ == '__main__':
    main()
