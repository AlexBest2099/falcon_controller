import rospy
import moveit_commander
import sys
from geometry_msgs.msg import PoseStamped
#moveit_commander move group
class moveit_node:
    def __init__(self):
        rospy.init_node('movelt_node')
        self.commander=moveit_commander.MoveGroupCommander('arm')
        #self.commander.set_joint_value_target()

    def go_to_joints_once(self, target_joint_positions):
        self.commander.set_joint_value_target(target_joint_positions)
        self.commander.go(wait=True)   
        self.commander.stop()
        self.commander.clear_pose_targets()

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    node = moveit_node()

    order = node.commander.get_active_joints()
    curr  = node.commander.get_current_joint_values()
    #print("Order:", order)
    #print("Zipped current:", list(zip(order, curr)))


    current = node.commander.get_current_joint_values()
    node.go_to_joints_once(current)
    target=PoseStamped()
    target.pose.position.x=0
    target.pose.position.y=0.1
    target.pose.position.z=0.4
    target.header.frame_id="table_top"
    target.pose.orientation.x=1
    target.pose.orientation.y=0
    target.pose.orientation.z=0
    target.pose.orientation.w=0
    pub = rospy.Publisher('expected_pos', PoseStamped, queue_size=10)
    while True:
        pub.publish(target)
    # target = [0.1, 0.000, 0.616, -1.976, 0.216, 2.788, 1.915]  
    # node.go_to_joints_once(target)