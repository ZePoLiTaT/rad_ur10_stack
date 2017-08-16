# rad_ur10_stack
Everything needed to get the UR10 arm running

Basic Motion:

  For Rviz Simulation:

    roslaunch ur10_gripper_moveit_config demo.launch
    rosrun ur10_gripper ur10_gripper_server
    rqt

  Actual Robot

    roslaunch ur_modern_driver ur10_ros_control.launch limited:=true robot_ip:=<ROBOT_IP>
    roslaunch ur10_gripper_moveit_config move_group.launch
    rosrun ur10_gripper ur10_gripper_server
    rqt
    rviz
