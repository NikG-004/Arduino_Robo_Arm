import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from math import pi
import serial

class MyRobot:
    def __init__(self, Group_Name):
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('predefined_pose', anonymous=True)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._planning_group = Group_Name
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        rospy.loginfo('\033[95m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[95m' + " >>> MyRobot initialization is done." + '\033[0m')

    def set_pose(self, arg_pose_name):
        rospy.loginfo('\033[32m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan_success, plan, planning_time, error_code = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[32m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[95m' + "Object of class MyRobot Deleted." + '\033[0m')

def main():
    arm = MyRobot("Arm_group")
    #hand =  MyRobot("hand")

    # Open serial port
    serial_port = "/dev/ttyUSB0"  # Adjust this based on your Arduino's port
    baud_rate = 57600  # Match this with your Arduino's baud rate
    timeout = 1  # Adjust timeout as needed
    
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=timeout)
        print(f"Reading data from {serial_port}...")
        while not rospy.is_shutdown():
            if ser.in_waiting > 0:
                # Decode using 'utf-8' encoding with error handling
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                print("Received:", data)
                # Example: Assuming data format is "pose_name"
                # You can modify this part based on your actual data format
                pose_name = data.strip()  # Extracting pose name from received data
                if pose_name == "straight":
                    arm.set_pose(pose_name)
                elif pose_name == "place":
                    arm.set_pose(pose_name)
                elif pose_name == "pic":
                    arm.set_pose("pick")
                elif pose_name == "shoulder":
                    arm.set_pose(pose_name)
                elif pose_name == "bent":
                    arm.set_pose(pose_name)
                elif pose_name == "rotate":
                    arm.set_pose(pose_name)
                elif pose_name == "base":
                    arm.set_pose(pose_name)
                elif pose_name == "elbow":
                    arm.set_pose(pose_name)
                elif pose_name == "wrist":
                    arm.set_pose(pose_name)
    except serial.SerialException as e:
        print("Serial port error:", e)
    finally:
        if ser.is_open:
            ser.close()

    del arm
    #del hand

if __name__ == '__main__':
    main()
