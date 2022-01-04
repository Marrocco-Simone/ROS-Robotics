#include "ros/ros.h"
#include <iostream>
#include "dynamic_reconfigure/ConfigDescription.h"
#include "dynamic_reconfigure/Config.h"
#include "control_msgs/JointControllerState.h"

void paramDesc_callback(const dynamic_reconfigure::ConfigDescription::ConstPtr& msg) {
    //ROS_INFO();
    //std::cout<<"\nparamDesc:\n"<<*msg;
}
void paramUpd_callback(const dynamic_reconfigure::Config::ConstPtr& msg) {
    //ROS_INFO();
    //std::cout<<"\nparamUpd:\n"<<*msg;
}
void state_callback(const control_msgs::JointControllerState::ConstPtr& msg) {
    //ROS_INFO();
    //std::cout<<"\nstate:\n"<<*msg;
    printf("current angle: %f\tprocessing: %f\n",msg->set_point,msg->process_value);
}

int main(int argc,char **argv) {
    ros::init(argc,argv,"demo_topic_subscriber");
    ros::NodeHandle node_obj;
    ros::Subscriber paramDesc_sub = node_obj.subscribe("/elbow_joint_position_controller/pid/parameter_descriptions",10,paramDesc_callback);
    ros::Subscriber paramUpd_sub = node_obj.subscribe("/elbow_joint_position_controller/pid/parameter_updates",10,paramUpd_callback);
    ros::Subscriber state_sub = node_obj.subscribe("/elbow_joint_position_controller/state",10,state_callback);
    ros::spin();
    return 0;
}