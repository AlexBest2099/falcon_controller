import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class falcon_visual:
    def __init__(self)->None:
        rospy.init_node("falcon_visualizer")
        self.pos_sub=rospy.Subscriber('/falcon/joy',Joy,self.joy_cb,queue_size=10)
        self.pose_pub=rospy.Publisher('falcon_pose',Marker,queue_size=10)
        self.timer=rospy.Timer(rospy.Duration(0.2),self.timer_cb)
        self.pos=[]
        self.cnt=0
    
    def joy_cb(self,msg)->None:
        self.pos=msg.axes

    def timer_cb(self,ev)->None:
        msg=Marker()
        msg.id=self.cnt
        self.cnt+=1
        msg.type=2
        msg.action=0
        msg.header.frame_id='falcon_front'
        msg.pose.position.x=self.pos[0]
        msg.pose.position.y=self.pos[1]
        msg.pose.position.z=self.pos[2]
        msg.pose.orientation.w=1
        msg.scale.x=0.01
        msg.scale.y=0.01
        msg.scale.z=0.01
        msg.color.r=1.0
        msg.color.a=1.0
        self.pose_pub.publish(msg)



if __name__=='__main__':
    falcon_visual()
    rospy.spin()





