#!bin/bash 

ID=$1
DATE=$(date +%Y-%m-%d_%H-%M-%S)
rosbag record -O ~/ros/src/falcon_controller/bags/experiment_p${ID}_${DATE} \
/falcon/measured_cp \
/falcon/joy \
/falcon/grip_state \
/joint_states \
/franka_state_controller/franka_states \
/position_joint_trajectory_controller/state \
/move_group/status \
/trajectory_execution_event \
/tf \
/tf_static \
/panda_target 

# First three are Teleop input topics

# Next three are Robot state Feedback topics

# Next two are MoveIt trajectory topics

# Next two are Transform tree topics for potential reconstruction

# Last topic is the topic our node publishes to

