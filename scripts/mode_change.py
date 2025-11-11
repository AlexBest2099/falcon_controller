#!/usr/bin/env python3
import yaml
import rospy
from geometry_msgs.msg import PoseStamped, Vector3
from falcon_controller.srv import ModeChange, ModeChangeRequest

def build_request(calib, mode):
    frame  = calib.get('frame', 'table_top')
    center = calib['volume']['center']
    ori    = calib[mode]['orientation']
    ext    = calib[mode]['extends']

    req = ModeChangeRequest()
    req.origin = PoseStamped()
    req.origin.header.frame_id = frame
    req.origin.header.stamp = rospy.Time.now()

    req.origin.pose.position.x = float(center['x'])
    req.origin.pose.position.y = float(center['y'])
    req.origin.pose.position.z = float(center['z'])

    req.origin.pose.orientation.x = float(ori.get('x', 0.0))
    req.origin.pose.orientation.y = float(ori.get('y', 0.0))
    req.origin.pose.orientation.z = float(ori.get('z', 0.0))
    req.origin.pose.orientation.w = float(ori.get('w', 1.0))

    req.extends = Vector3()
    req.extends.x = float(ext['x'])
    req.extends.y = float(ext['y'])
    req.extends.z = float(ext['z'])
    return req

def main():
    rospy.init_node("mode_change_from_yaml", anonymous=True)

    yaml_path = rospy.get_param("~yaml_path", "/home/hri25-group1/volume_data.yaml")
    mode      = rospy.get_param("~mode", "upright").lower()   # 'upright' or 'flip'
    service   = "/panda_mapped/mode_change"

    if mode not in ("upright", "flip"):
        rospy.logerr("~mode must be 'upright' or 'flip'")
        return

    with open(yaml_path, "r") as f:
        calib = yaml.safe_load(f)

    req = build_request(calib, mode)

    rospy.wait_for_service(service)
    proxy = rospy.ServiceProxy(service, ModeChange)
    resp = proxy(req)
    rospy.loginfo(f"response: {resp}")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
