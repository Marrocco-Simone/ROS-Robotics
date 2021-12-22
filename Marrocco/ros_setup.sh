#update system
sudo apt update;
sudo apt upgrade;

#download components
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list';
sudo apt install curl;
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -;
sudo apt remove ros-noetic-desktop-full;
sudo apt install ros-noetic-desktop-full;
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc;
source ~/.bashrc;
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential;
sudo apt-get install ros-$ROS_DISTRO-rviz;
sudo apt install ros-noetic-moveit;
sudo rosdep init;
rosdep update;

#create workspace
rm -r catkin_ws -f;
mkdir catkin_ws;
cd catkin_ws;
mkdir src;
catkin_make;
source devel/setup.bash;
sudo apt-get install ros-noetic-ros-tutorials;

#test
gnome-terminal -- roscore;
sleep 10;
rosnode list;
gnome-terminal -- rosrun turtlesim turtlesim_node __name:=my_turtle;
sleep 10;
gnome-terminal -- rosrun turtlesim turtle_teleop_key;
