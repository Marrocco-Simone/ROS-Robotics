####tutorial from https://roboticscasual.com/ros-tutorial-control-the-ur5-robot-with-ros_control-tuning-a-pid-controller/
####comment or uncomment the lines you need

####remove catkin workspace if it exists

rm -r catkin_ws -f;

####start the catkin workspace

mkdir catkin_ws;
cd catkin_ws;
mkdir src;
cd src;
git clone https://github.com/dairal/ur5-joint-position-control.git;
git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git;
git clone https://github.com/dairal/ur5-tcp-position-control.git;

    ####files for gripper

    #git clone https://github.com/filesmuggler/robotiq.git;
    #cd universal_robot/ur_description/urdf;
    #wget https://raw.githubusercontent.com/utecrobotics/ur5/master/ur5_description/urdf/ur5_robotiq85_gripper.urdf.xacro;
    ####sed -i 'LINENUMBERs/.*/replacement-line/' file.txt #remember to escape special characters with \
    #sed -i '4s/.*/  <xacro:include filename=\"$(find ur_description)\/urdf\/ur5_joint_limited_robot.urdf.xacro\" \/>/' ur5_robotiq85_gripper.urdf.xacro;
    #cd ../../../robotiq/robotiq_description/urdf;
    #sed -i '9s/.*/  \<hardwareInterface\>hardware_interface\/EffortJointInterface\<\/hardwareInterface\>/' robotiq_85_gripper.transmission.xacro;
    #sed -i '14s/.*/  \<hardwareInterface\>hardware_interface\/EffortJointInterface\<\/hardwareInterface\>/' robotiq_85_gripper.transmission.xacro;
    #cd ../../../universal_robot/ur_description/urdf;
    #sed -i '5s/.*/  \<xacro:arg name=\"transmission_hw_interface\" default=\"hardware_interface\/EffortJointInterface\"\/>/' ur5_joint_limited_robot.urdf.xacro
    #cd ../../..;

cd ..;
###curl -sSL http://get.gazebosim.org | sh; ####not necessary if you have already gazebo
sudo apt-get install ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers;
rosdep update;
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src;
cd ..;

####source the project

cd catkin_ws;
catkin_make;
source devel/setup.bash;
cd ..;

####start gazebo in another terminal

gnome-terminal -- roslaunch gazebo_ros empty_world.launch;

    ####spawn useless robot (useful for testing)

    #gnome-terminal -- rosrun gazebo_ros spawn_model -file `rospack find ur5-joint-position-control`/urdf/ur5_jnt_pos_ctrl.urdf -urdf -x 0 -y 0 -z 0.1 -model ur5;
    #gnome-terminal -- roslaunch ur5-joint-position-control ur5_joint_position_control.launch;

####spawn useful robot, pause simulation first

sleep 10;
printf "\n\n\nYou may want to click Scene in the top left of Gazebo and then uncheck Shadows for better visibility\n";
read -p "Press the pause button in the lower bar to pause the simulation. Then press Enter: ";
read -p "Confirm you paused it: ";
gnome-terminal -- rosrun gazebo_ros spawn_model -file `rospack find ur5-joint-position-control`/urdf/ur5_jnt_pos_ctrl.urdf -urdf -x 0 -y 0 -z 0.1 -model ur5 -J shoulder_lift_joint -1.5 -J elbow_joint 1.0;
gnome-terminal -- roslaunch ur5-joint-position-control ur5_joint_position_control.launch;
printf "You can now unpause the simulation\n";
read -p "Press Enter to continue";

    ####lets move the robot

    #gnome-terminal -- rosrun rqt_gui rqt_gui;
    #printf "Go to Plugins>Topics>Message Publisher and choose a joint, like /wrist_1_joint_position_controller/command\n";
    #printf "Set rate to 100 and expand, then on data>expression set the value of the joint\n";
    #read -p "Press Enter to continue";
    #printf "Go to Plugins>Visualization>Plot to see how the link is reacting to the changes\n";
    #printf "Add /wrist_1_joint_position_controller/state/process_value (the measured value) and /wrist_1_joint_position_controller/command/data (the actual movement)\n";
    #read -p "Press Enter to continue";
    #printf "Go to Plugins>Configuration>Dynamic Reconfigure, choose the link and click on his pid\n";
    #printf "Increase p to reach the desired position quicker and more accurately but increase the oscillations\n";
    #printf "Increase d to reduce overshoot greatly but reduce velocity\n";
    #printf "Play with the values to find a good combo\n";
    #read -p "Press Enter to continue";

####lets move it in a easier way. use only increments of 10^-2 for the xyz axis!

gnome-terminal -- rosrun ur5-tcp-position-control  tcp_position_controller;
