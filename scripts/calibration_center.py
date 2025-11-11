#!/usr/bin/env python3
import yaml
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander

def main():
    rospy.init_node("center_calibration", anonymous=True)

    group_name   = rospy.get_param("~group_name", "arm")
    eef_link     = rospy.get_param("~eef_link", "pen")
    frame        = rospy.get_param("~frame", "table_volume")
    x            = float(rospy.get_param("~x", 0.0))
    y            = float(rospy.get_param("~y", 0.0))
    z            = float(rospy.get_param("~z", 0.05))
    out_yaml     = rospy.get_param("~out_yaml", "/home/hri25-group1/center_joints.yaml")

    group = MoveGroupCommander(group_name)
    group.set_end_effector_link(eef_link)
    try:
        group._g.start_state_monitor(1.0)
    except Exception:
        pass

    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    # same orientation you used elsewhere
    pose.pose.orientation.x = 1.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0
    pose.header.stamp = rospy.Time.now()

    group.set_joint_value_target(pose)
    ok = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    if not ok:
        rospy.logerr("Move failed. No joints written.")
        return

    joints = list(group.get_current_joint_values())

    with open(out_yaml, "w") as f:
        yaml.safe_dump(joints, f, sort_keys=False, default_flow_style=False)
    print(out_yaml)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
