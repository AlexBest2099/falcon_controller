import rospy
from sensor_msgs.msg import Joy
import tf2_ros
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
from falcon_controller.srv import ModeChange,ModeChangeResponse,ModeChangeRequest
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
import math
import tf.transformations as tft
from tf2_msgs.msg import TFMessage

class Mapper:
    def __init__(self) -> None:
        rospy.init_node("falcon_absolute_controller")
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.visual_pub = rospy.Publisher("~mapped_falcon_target_visual", PointStamped, queue_size=1)
        self.marker_pub=rospy.Publisher("~marker",Marker,queue_size=10,latch=True)
        self.target_pub=rospy.Publisher("panda_target", PointStamped, queue_size=1)
        rospy.sleep(0.5)
        self.mode_service=rospy.Service('~mode_change',ModeChange,handler=self.srv_cb)
        self.last_button_state=0
        req=ModeChangeRequest()
        req.origin.pose.position.x=0.0
        req.origin.pose.position.y=0.2
        req.origin.pose.position.z=0.168
        req.origin.pose.orientation.w=1
        req.origin.header.frame_id='table_top'
        req.extends.x=0.5
        req.extends.y=0.5
        req.extends.z=0.3

        
        self.FALCON_VOLUME_EXTENDS= [0.043, 0.1, 0.05]
        self.t_falcon_volume = TransformStamped()
        self.t_falcon_volume .header.stamp = rospy.Time.now()
        self.t_falcon_volume .header.frame_id = "falcon_front"
        self.t_falcon_volume .child_frame_id = "falcon_volume"

        # origin xyz="0 0 0.12"
        self.t_falcon_volume .transform.translation.x = 0.0
        self.t_falcon_volume .transform.translation.y = 0.0
        self.t_falcon_volume .transform.translation.z = 0.12

        # rpy="${tau/2} ${-tau/4} ${tau/4}"  →  (π/2, −π/4, π/4)
        q = tft.quaternion_from_euler(math.tau/2 , -math.tau / 4, math.tau / 4)
        self.t_falcon_volume .transform.rotation.x = q[0]
        self.t_falcon_volume .transform.rotation.y = q[1]
        self.t_falcon_volume .transform.rotation.z = q[2]
        self.t_falcon_volume .transform.rotation.w = q[3]
        # publish the transform
        self.srv_cb(req)
        # now create matching marker
        m = Marker()
        m.header.frame_id ='falcon_volume'
        m.header.stamp = rospy.Time.now()
        m.ns = "falcon_volume"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0

        # box size from URDF
        m.scale.x = self.FALCON_VOLUME_EXTENDS[0]
        m.scale.y = self.FALCON_VOLUME_EXTENDS[1]
        m.scale.z = self.FALCON_VOLUME_EXTENDS[2]

        # color rgba="0.0 0.0 1.0 0.1"
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 1.0
        m.color.a = 0.1
        self.marker_pub.publish(m)
        self.pos_sub = rospy.Subscriber("falcon/joy", Joy, self.joy_cb, queue_size=1)
        
    def joy_cb(self, msg: Joy) -> None:
        pos = msg.axes
        transformed_point = self.project_to_falcon_volume(pos=pos)
        
        self.visual_pub.publish(transformed_point)
        if msg.buttons[2] and not self.last_button_state:                
            self.target_pub.publish(transformed_point)
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

            # normalize in falcon volume
            
            transformed_point.point.x/= self.FALCON_VOLUME_EXTENDS[0]/2
            transformed_point.point.y/= self.FALCON_VOLUME_EXTENDS[1]/2
            transformed_point.point.z/= self.FALCON_VOLUME_EXTENDS[2]/2

            # clamp
            transformed_point.point.x= max(min(transformed_point.point.x,1),-1)
            transformed_point.point.y= max(min(transformed_point.point.y,1),-1)
            transformed_point.point.z= max(min(transformed_point.point.z,1),-1)

            # scale to target volume
            transformed_point.header.frame_id = "table_volume"
            transformed_point.point.x=transformed_point.point.x*self.extends.x/2
            transformed_point.point.y=transformed_point.point.y*self.extends.y/2
            transformed_point.point.z=transformed_point.point.z*self.extends.z/2

            return transformed_point
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logerr(e)
            raise(RuntimeError)

    def srv_cb(self,request):
        success=False
        try:
            self.origin=request.origin
            self.extends=request.extends
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self.origin.header.frame_id
            t.child_frame_id = 'table_volume'
            t.transform.translation= self.origin.pose.position
            t.transform.rotation=self.origin.pose.orientation
            published_transforms=[t, self.t_falcon_volume]
            self.broadcaster.pub_tf.publish(TFMessage(published_transforms))
            m=Marker()
            m.header.frame_id=t.child_frame_id
            m.type=m.CUBE
            m.pose.orientation.w=1
            m.ns='volume'
            m.scale=self.extends
            m.color.b=0.8
            m.color.a=0.3
            self.marker_pub.publish(m)
            success=True 
        except:
            success=False
        return ModeChangeResponse(success)


if __name__ == "__main__":
    Mapper()
    rospy.spin()
