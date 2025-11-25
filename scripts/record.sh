#!/bin/bash

echo -n "enter participant id: "
read participant_id

condition=${1:-unknown}

rosbag record -o ~/ros/src/falcon_controller/bags/experiment_participant_${participant_id}_condition_${condition} \
/falcon/measured_cp \
/falcon/joy \
/falcon/grip_state \
/joint_states \
/franka_state_controller/franka_states \
/position_joint_trajectory_controller/state \
/move_group/goal \
/move_group/result \
/tf \
/tf_static \
/panda_target \
/panda_mapped/marker \
/panda_mapped/mapped_falcon_target_visual \
/franka_state_controller/F_ext \
/frame_pose \

# First three are Teleop input topics

# Next three are Robot state Feedback topics

# Next two are MoveIt trajectory topics

# Next two are Transform tree topics for potential reconstruction

# Last topic is the topic our node publishes to

