import rospy
from sensor_msgs.msg import Joy
import tf2_ros
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs

class Mapper:
    def __init__(self) -> None:
        rospy.init_node("falcon_absoulte_controller")
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pos_sub = rospy.Subscriber("falcon/joy", Joy, self.joy_cb, queue_size=1)
        self.visual_pub = rospy.Publisher("mapped_falcon_target_visual", PointStamped, queue_size=1)
        self.target_pub=rospy.Publisher("panda_target", PointStamped, queue_size=1)
        self.scale = 5
        self.bounds = {
            "x_max" :0.05 * self.scale,
            "x_min" :-0.05 * self.scale,
            "y_max":0.05 * self.scale,
            "y_min":-0.05 * self.scale,
            "z_max":0.05 * self.scale,
            "z_min":-0.05 * self.scale,
        }
        self.last_button_state=0

    def joy_cb(self, msg: Joy) -> None:
        pos = msg.axes
        transformed_point = self.project_to_falcon_volume(pos=pos)
        target_point = PointStamped()
        transformed_scaled_point_x = transformed_point.point.x * self.scale
        transformed_scaled_point_y = transformed_point.point.y * self.scale
        transformed_scaled_point_z = transformed_point.point.z * self.scale

        target_point.point.x= max(min(transformed_scaled_point_x,self.bounds['x_max']),self.bounds['x_min'])
        target_point.point.y = max(min(transformed_scaled_point_y,self.bounds['y_max']),self.bounds['y_min'])
        target_point.point.z = max(min(transformed_scaled_point_z,self.bounds['z_max']),self.bounds['z_min'])

        target_point.header.frame_id = "panda_table_volume"
        self.visual_pub.publish(target_point)
        if msg.buttons[2] and not self.last_button_state:
            self.target_pub.publish(target_point)
            rospy.loginfo('Target Sent')
        self.last_button_state=msg.buttons[2]


    def project_to_falcon_volume(self, pos: list) -> PointStamped:
        point = PointStamped()
        point.point.x = pos[0]
        point.point.y = pos[1]
        point.point.z = pos[2]
        point.header.frame_id = "falcon_front"
        try:
            transformed_point = self.tf_buffer.transform(
                point,"falcon_volume",rospy.Duration(0.1)
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
    Mapper()
    rospy.spin()
