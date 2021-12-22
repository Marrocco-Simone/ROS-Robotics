####tutorial from https://roboticscasual.com/ros-tutorial-control-the-ur5-robot-with-ros_control-tuning-a-pid-controller/
####comment or uncomment the lines you need

####remove catkin workspace if it exists

#rm -r catkin_ws -f;

####start the catkin workspace

#mkdir catkin_ws;
#cd catkin_ws;
#mkdir src;
#cd src;
#git clone https://github.com/dairal/ur5-joint-position-control.git;
#git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git;
#git clone https://github.com/dairal/ur5-tcp-position-control.git;
#git clone https://github.com/filesmuggler/robotiq.git;
#cd ..;
####curl -sSL http://get.gazebosim.org | sh; ####not necessary if you have already gazebo
#sudo apt-get install ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers;
#rosdep update;
#rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src;
#cd ..;

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

####download and modify files for Moveit

#cd catkin_ws/src/universal_robot/ur_description/urdf;
#wget https://raw.githubusercontent.com/utecrobotics/ur5/master/ur5_description/urdf/ur5_robotiq85_gripper.urdf.xacro;
###sed -i 'LINENUMBERs/.*/replacement-line/' file.txt #remember to escape special characters with \
#sed -i '4s/.*/  <xacro:include filename=\"$(find ur_description)\/urdf\/ur5_joint_limited_robot.urdf.xacro\" \/>/' ur5_robotiq85_gripper.urdf.xacro;
#cd ../../../robotiq/robotiq_description/urdf;
#sed -i '9s/.*/  \<hardwareInterface\>hardware_interface\/EffortJointInterface\<\/hardwareInterface\>/' robotiq_85_gripper.transmission.xacro;
#sed -i '14s/.*/  \<hardwareInterface\>hardware_interface\/EffortJointInterface\<\/hardwareInterface\>/' robotiq_85_gripper.transmission.xacro;
#cd ../../../universal_robot/ur_description/urdf;
#sed -i '5s/.*/  \<xacro:arg name=\"transmission_hw_interface\" default=\"hardware_interface\/EffortJointInterface\"\/>/' ur5_joint_limited_robot.urdf.xacro
#cd ../../../../..;
#cd catkin_ws;
#catkin_make;
#source devel/setup.bash;
#cd ..;

    ####setup Moveit the hard way

    #gnome-terminal -- roslaunch moveit_setup_assistant setup_assistant.launch;

    #printf "Choose Create New Moveit and load /universal_robot/ur_description/urdf/ur5_robotiq85_gripper.urdf.xacro\n";
    #read -p "Press Enter to continue";
    #printf "Go on Self Collision and click Generate Collision Matrix\n";
    #read -p "Press Enter to continue";
    #printf "Go on Virtual Joint and set up as this: virtual_joint | base_link | world | fixed\n";0
    #read -p "Press Enter to continue";
    #printf "Go on Planning Groups, click Add Group, then enter ur5_arm as Group Name and choose kdl_kinematics_plugin/KDLKinematicsPlugin as Kinematic Solver. Then click Add Kin. Chain. Expand all the first arrows until you see ee_link and set it as Tip Link, and base_link as Base Link, then Save\n";
    #printf "Now click Add Group again, Group Name = gripper, no Kin. Solver and click Add Joint. Select the robotiq_85_left_knuckle_joint and click the > arrow, then save\n";
    #read -p "Press Enter to continue";
    #printf "Go on Robot Poses and click add pose. Name= home, group name ur5_arm, values: \nshoulder_pan_joint 0.0 \nshoulder_lift_joint -1.5447 \nelbow_joint 1.5447 \nwrist_1_joint -1.5794 \nwrist_2_joint -1.5794 \nwrist_3_joint 0.0\n";
    #printf "Add two more poses, group name gripper, with names open and close and values 0.1 and 0.3\n";
    #read -p "Press Enter to continue";
    #printf "Go to End Effectors, add one with \nname: robotiq_gripper, \nend effector group: gripper, \nparent link: ee_link, \nparent group: ur5_arm\n";
    #read -p "Press Enter to continue";
    #printf "Go on Passive Joints, select all the robotiq_85 file except the left_knukle_joint (we did it before) and move them with >. They should be five\n";
    #read -p "Press Enter to continue";
    #printf "Go on Ros Control and click Auto Add Follow[...]\n";
    #read -p "Press Enter to continue";
    #printf "Nothing to do on Simulation and 3D Perception.\nOn Author, just fill the data with anything\n";
    #read -p "Press Enter to continue";
    #printf "In Configuration FIles, save the configuration package inside the src folder of the catkin space with name ur5_gripper_moveit_config (but also do a backup somewhere else). Insert a new folder name that will be created\n";
    #read -p "Press Enter to continue";

    #printf "please modify the file config/ros_controllers.yaml with the text here: https://github.com/dairal/ur5_gripper_moveit_config/blob/main/config/ros_controllers.yaml\n";
    #read -p "Press Enter to continue";
    #cd catkin_ws/src/ur5_gripper_moveit_config/launch;
    #sed -i '9s/.*/ output="screen" args="ur5_arm_controller gripper_controller joint_state_controller"\/>/' ros_controllers.launch;
    #sed -i '9s/.*/ <node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" args="-urdf -param robot_description -model robot -x 0 -y 0 -z 0.1"/' gazebo.launch;
    #cd ../../../..;

####setup Moveit donwloaded lmao

#cd catkin_ws/src;
#git clone https://github.com/dairal/ur5_gripper_moveit_config;
#cd ../..;

####prepare file for pick and place - no obstacles

#cd catkin_ws;
#cd src;
#sudo apt-get install ros-noetic-rviz-visual-tools;
#sudo apt-get install ros-noetic-moveit-visual-tools;
#git clone https://github.com/dairal/ur5_simple_pick_and_place;
#cd ur5_gripper_moveit_config/launch;
#sed -i '9s/.*/ \t<arg name="world_name" default="$(find ur5_simple_pick_and_place)\/world\/simple_pick_and_place"\/>/' gazebo.launch;
#sed -i '18s/.*/ \t<node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" args="-urdf -param robot_description -model robot -x 0 -y 0 -z 1.21"/' gazebo.launch;
#cd ../../../..;

    ####prepare file for pick and place with obstacles

    #cd catkin_ws;
    #cd src;
    #sudo apt-get install ros-noetic-rviz-visual-tools;
    #sudo apt-get install ros-noetic-moveit-visual-tools;
    #git clone https://github.com/dairal/ur5_simple_pick_and_place;
    #cd ur5_gripper_moveit_config/launch;
    #sed -i '9s/.*/ \t<arg name="world_name" default="$(find ur5_simple_pick_and_place)\/world\/simple_pick_and_place_collision"\/>/' gazebo.launch;
    #sed -i '18s/.*/ \t<node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" args="-urdf -param robot_description -model robot -x 0 -y 0 -z 1.21"/' gazebo.launch;
    #cd ../../../..;

####source catkin 
#rosdep update;
#rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src;
#cd catkin_ws;
#catkin_make;
#source devel/setup.bash;
#cd ..;

####open Moveit

#gnome-terminal -- roslaunch ur5_gripper_moveit_config demo_gazebo.launch;
#sleep 10;
    #rosrun ur5_simple_pick_and_place pick_and_place; ####no collision
#rosrun ur5_simple_pick_and_place pick_and_place_collision;