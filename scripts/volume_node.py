import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf_conversions
import math
from falcon_controller.srv import ModeChange,ModeChangeResponse
from std_msgs.msg import Bool


class VolumeNode():
    def __init__(self):
        rospy.init_node("falcon_volume_pub")
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.mode_service=rospy.Service('mode_change',ModeChange,handler=self.srv_cb)
        self.publish_static_box(
            parent="falcon_front",
            child="falcon_volume",
            x=0.0, y=0.0, z=0.12,
            roll=180.0, pitch=-90.0, yaw=90.0
        )

    def srv_cb(self,request):
        succes=Bool()
        try:
            self.publish_static_box(
                parent="falcon_front",
                child="falcon_volume",
                x=0.0, y=0.0, z=0.12,
                roll=request.orientation[0].data, pitch=request.orientation[1].data, yaw=request.orientation[2].data
            )
            succes.data=True
        except:
            succes.data=False
        return ModeChangeResponse(succes)


    def publish_static_box(self, parent, child, x, y, z, roll, pitch, yaw):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = child

        # translation
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        # orientation
        quat = tf_conversions.transformations.quaternion_from_euler(
            math.radians(roll), math.radians(pitch), math.radians(yaw)
        )
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.broadcaster.sendTransform(t)
        rospy.loginfo(f"Static transform published: {parent} → {child}")


if __name__ == "__main__":
    VolumeNode()
    rospy.spin()

