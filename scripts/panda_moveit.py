import rospy
from geometry_msgs.msg import PoseStamped
import moveit_commander
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
import tf2_ros
import yaml
import os 
class MovePanda:
    def __init__(self) -> None:
        rospy.init_node("falcon_moveit")
        self.setup=True
        self.commander = moveit_commander.MoveGroupCommander("arm")
        self.commander._g.start_state_monitor(1.0)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pos_sub = rospy.Subscriber("panda_target", PointStamped, self.move_cb, queue_size=1)
        self.commander.set_end_effector_link("pen")
        self.center=1
        yaml_path='/home/hri25-group1/center_joints.yaml'
        with open(yaml_path, "r") as file:
            self.joints = yaml.safe_load(file)


    def move_cb(self,msg):

        pose=PoseStamped()
        pose.pose.orientation.x=1
        pose.pose.orientation.y=0
        pose.pose.orientation.z=0
        pose.pose.orientation.w=0
        pose.header.frame_id="table_top"
        pose=self.tf_buffer.transform(
                    pose,"table_volume",rospy.Duration(0.1)
                )
            
        pose.pose.position.x=msg.point.x
        pose.pose.position.y=msg.point.y
        pose.pose.position.z=msg.point.z
        pose.header.frame_id=msg.header.frame_id
        pose.header.stamp=rospy.Time.now()
        self.go_to_pos_once(target_positions=pose)
        rospy.loginfo("Command sent to panda")
        pose_msg = self.commander.get_current_pose("pen")

        # Convert current end-effector point to table_top frame
        point_stamped = PointStamped()
        point_stamped.header = pose_msg.header
        point_stamped.point = pose_msg.pose.position
        pose_in_table_top = self.tf_buffer.transform(point_stamped, "table_top", rospy.Duration(0.1))
        rospy.loginfo(str(point_stamped.point))

        if pose_in_table_top.point.z <= 0.005 and not self.setup:
            rospy.sleep(1)
            rospy.loginfo("Going back to center")
            # # Create a small upward move in table_volume frame
            # lift_pose = PoseStamped()
            # lift_pose.header.frame_id = "table_top"
            # lift_pose.pose.orientation.x = 1
            # lift_pose.pose.orientation.y = 0
            # lift_pose.pose.orientation.z = 0
            # lift_pose.pose.orientation.w = 0
            # lift_pose.pose.position.x = pose_in_table_top.point.x
            # lift_pose.pose.position.y = pose_in_table_top.point.y
            # lift_pose.pose.position.z = pose_in_table_top.point.z + 0.05

            # # Transform to table_volume frame
            # lift_pose = self.tf_buffer.transform(lift_pose, "table_volume", rospy.Duration(0.1))
            # lift_pose.header.stamp = rospy.Time.now()
            # self.go_to_pos_once(target_positions=lift_pose)

            # # Move to volume center
            # center_pose = PoseStamped()
            # center_pose.header.frame_id = "table_volume"
            # center_pose.pose.orientation = lift_pose.pose.orientation
            # center_pose.pose.position.x = 0.0
            # center_pose.pose.position.y = 0.0
            # center_pose.pose.position.z = 0.02
            # center_pose.header.stamp = rospy.Time.now()
            # self.go_to_pos_once(target_positions=center_pose)
            # joint_goal = [0.8912, 0.1106, 0.3726, -2.1895, -0.0542, 2.2927, 2.0831]
            self.go_to_pos_once(self.joints)
        self.setup=False


    def go_to_pos_once(self, target_positions: PoseStamped) -> None:
        self.commander.set_joint_value_target(target_positions)
        self.commander.go(wait=True)
        self.commander.stop()
        self.commander.clear_pose_targets()




if __name__ == "__main__":
    MovePanda()
    rospy.spin()
