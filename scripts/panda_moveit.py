import rospy
from geometry_msgs.msg import PoseStamped
import moveit_commander
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
import tf2_ros


class MovePanda:
    def __init__(self) -> None:
        rospy.init_node("falcon_moveit")
        self.commander = moveit_commander.MoveGroupCommander("arm")
        self.commander._g.start_state_monitor(1.0)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pos_sub = rospy.Subscriber("panda_target", PointStamped, self.move_cb, queue_size=1)

    def move_cb(self,msg):
        pose=PoseStamped()
        
        pose.pose.orientation.x=1
        pose.pose.orientation.y=0
        pose.pose.orientation.z=0
        pose.pose.orientation.w=0
        pose.header.frame_id="falcon_volume"
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

    def go_to_pos_once(self, target_positions: PoseStamped) -> None:
        self.commander.set_joint_value_target(target_positions)
        self.commander.go(wait=True)
        self.commander.stop()
        self.commander.clear_pose_targets()




if __name__ == "__main__":
    MovePanda()
    rospy.spin()
