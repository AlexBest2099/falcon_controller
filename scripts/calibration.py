#!/usr/bin/env python3
import rospy
import yaml
import tf2_ros
from geometry_msgs.msg import TransformStamped

class VolumeRecorderTF:
    def __init__(self):
        rospy.init_node("volume_recorder_tf")

        self.target_frame = rospy.get_param("~target_frame", "table_top")
        self.eef_frame    = rospy.get_param("~eef_frame", "pen")  # set to your EE frame if different
        self.out_file     = rospy.get_param("~out_file", "/home/hri25-group1/volume_data.yaml")
        # Set to None to use z_max from sampled points
        self.fixed_top_z  = rospy.get_param("~fixed_top_z", 0.149)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.points = []  # list of (x, y, z) in target_frame

    def get_eef_in_table(self):
        tf: TransformStamped = self.tf_buffer.lookup_transform(
            self.target_frame, self.eef_frame, rospy.Time(0), rospy.Duration(0.5)
        )
        t = tf.transform.translation
        return (t.x, t.y, t.z)

    def save_point(self):
        try:
            p = self.get_eef_in_table()
            self.points.append(p)
            print(f"Saved #{len(self.points)} @ {self.target_frame}: "
                  f"({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")
        except Exception as e:
            print(f"Save failed: {e}")

    def compute_and_write(self):
        if len(self.points) < 4:
            print("Need 4 points.")
            return

        xs, ys, zs = zip(*self.points[:4])
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        z_min, z_max = min(zs), max(zs)

        cx = (x_max + x_min) / 2.0
        cy = (y_max + y_min) / 2.0
        cz = float(self.fixed_top_z) if self.fixed_top_z is not None else z_max

        dx = x_max - x_min
        dy = y_max - y_min
        dz = 0.3  

        extends_upright = {"x": dx, "y": dy, "z": dz}
        extends_flip    = {"x": dz, "y": dy, "z": dx}  # rotate ±90° about Y → swap x↔z

        data = {
            "frame": self.target_frame,
            "points_used": [{"x": xs[i], "y": ys[i], "z": zs[i]} for i in range(4)],
            "volume": {
                "center": {"x": cx, "y": cy, "z": cz}
            },
            "upright": {
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                "extends": extends_upright
            },
            "flip": {
                "orientation": {"x": 0.0, "y": -0.707, "z": 0.0, "w": 0.707},
                "extends": extends_flip
            }
        }

        try:
            with open(self.out_file, "w") as f:
                yaml.safe_dump(data, f, sort_keys=False)
            print(f"Wrote {self.out_file}")
        except Exception as e:
            print(f"Write failed: {e}")

def main():
    vr = VolumeRecorderTF()
    print("s = save current EEF pose (TF), q = write YAML and quit, p = print count")
    while not rospy.is_shutdown():
        cmd = input("> ").strip().lower()
        if cmd == "s":
            vr.save_point()
            if len(vr.points) == 4:
                print("Collected 4 points. Computing and writing now.")
                vr.compute_and_write()
        elif cmd == "p":
            print(f"{len(vr.points)} point(s) saved.")
        elif cmd == "q":
            vr.compute_and_write()
            break
        else:
            print("Use s, p, or q.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
