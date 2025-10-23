import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
import moveit_commander
import tf2_ros
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs


class controller:
    def __init__(self) -> None:
        rospy.init_node("falcon_absoulte_controller")
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pos_sub = rospy.Subscriber("falcon/joy", Joy, self.joy_cb, queue_size=1)
        self.pose_pub = rospy.Publisher("mapped_falcon_target", PoseStamped, queue_size=1)
        self.commander = moveit_commander.MoveGroupCommander("arm")
        self.scale = 5
        self.bounds = {
            "x_max" :0.05 * self.scale,
            "x_min" :-0.05 * self.scale,
            "y_max":0.05 * self.scale,
            "y_min":-0.05 * self.scale,
            "z_max":0.05 * self.scale,
            "z_min":-0.05 * self.scale,
        }

    def joy_cb(self, msg: Joy) -> None:
        pos = msg.axes
        transformed_point = self.project_to_falcon_volume(pos=pos)
        pose = PoseStamped()
        transformed_scaled_point_x = transformed_point.point.x * self.scale
        transformed_scaled_point_y = transformed_point.point.y * self.scale
        transformed_scaled_point_z = transformed_point.point.z * self.scale

        pose.pose.position.x =max(min(transformed_scaled_point_x,self.bounds['x_max']),self.bounds['x_min'])
        pose.pose.position.y = max(min(transformed_scaled_point_y,self.bounds['y_max']),self.bounds['y_min'])
        pose.pose.position.z = max(min(transformed_scaled_point_z,self.bounds['z_max']),self.bounds['z_min'])

        #pose.pose.position.x = transformed_point.point.x
        #pose.pose.position.y = transformed_point.point.y
        #pose.pose.position.z = transformed_point.point.z
        pose.pose.orientation.x = 1
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 0
        pose.header.frame_id = "panda_table_volume"
        self.pose_pub.publish(pose)
        if msg.buttons[2]:
            self.go_to_pos_once(pose)
            rospy.loginfo('Command Sent')


    def go_to_pos_once(self, target_positions: PoseStamped) -> None:
        self.commander.set_joint_value_target(target_positions)
        self.commander.go(wait=True)
        self.commander.stop()
        self.commander.clear_pose_targets()

    def project_to_falcon_volume(self, pos: list) -> PointStamped:
        point = PointStamped()
        point.point.x = pos[0]
        point.point.y = pos[1]
        point.point.z = pos[2]
        point.header.frame_id = "falcon_front"
        try:
            transformed_point = self.tf_buffer.transform(
                point, "falcon_volume",rospy.Duration(0.1)
            )
            return transformed_point
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logerr(e)
            raise(RuntimeError)



if __name__ == "__main__":
    controller()
    rospy.spin()
