#!/usr/bin/env python3
import rospy
import actionlib
from franka_gripper.msg import GraspAction, GraspGoal, GraspEpsilon

def grasp_client(width=0.04, speed=0.1, force=10.0):
    rospy.init_node("grasp_client")

    client = actionlib.SimpleActionClient("/franka_gripper/grasp", GraspAction)
    rospy.loginfo("Waiting for /franka_gripper/grasp action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to /franka_gripper/grasp server.")

    goal = GraspGoal()
    goal.width = width  # target opening width [m]
    goal.speed = speed  # grasping speed [m/s]
    goal.force = force  # grasping force [N]
    goal.epsilon = GraspEpsilon(inner=0.1, outer=0.1)

    client.send_goal(goal)
    client.wait_for_result()

    result = client.get_result()
    rospy.loginfo(f"Grasp result: {result}")
    return result

if __name__ == "__main__":
    try:
        grasp_client(width=0.06)
    except rospy.ROSInterruptException:
        pass
