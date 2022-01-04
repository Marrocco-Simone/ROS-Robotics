/* #include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <unistd.h> //usleep

int main(int argc, char **argv){
    //modificare con nome finale file
    ros::init(argc,argv,"demo_topic_publisher");
    ros::NodeHandle node_obj;
    ros::Publisher elbow_pub = node_obj.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command",10);
    ros::Publisher shoulderPan_pub = node_obj.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command",10);
    ros::Rate loop_rate(10);
    float x = 0.0;

    while(ros::ok()){
        std_msgs::Float64 msg;
        msg.data = x;
        ROS_INFO("%f", msg.data);
        elbow_pub.publish(msg);
        shoulderPan_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        x += 0.1;
        if(x > 3) x = -3;
        usleep(100000);
    }

    return 0;
} */

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include "control_msgs/JointControllerState.h"
#include <unistd.h> //usleep

float elbowAngle, shoulderPanAngle;

void elbow_callback(const control_msgs::JointControllerState::ConstPtr& msg) {
    elbowAngle = msg->set_point;
}
void shoulderPan_callback(const control_msgs::JointControllerState::ConstPtr& msg) {
    shoulderPanAngle = msg->set_point;
}

int main(int argc, char **argv){
    //modificare con nome finale file
    ros::init(argc,argv,"demo_topic_publisher");
    ros::NodeHandle node_obj;
    ros::Publisher elbow_pub = node_obj.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command",10);
    ros::Subscriber elbow_sub = node_obj.subscribe("/elbow_joint_position_controller/state",10,elbow_callback);
    ros::Publisher shoulderPan_pub = node_obj.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command",10);
    ros::Subscriber shoulderPan_sub = node_obj.subscribe("/shoulder_pan_joint_position_controller/state",10,shoulderPan_callback);
    ros::Rate loop_rate(10);

    while(ros::ok()){
        std_msgs::Float64 elbowMsg, shoulderPanMsg;
        
        elbowMsg.data = elbowAngle;
        elbowMsg.data = elbowMsg.data<3 ? elbowMsg.data + 0.1 : -3.0;
        
        shoulderPanMsg.data = shoulderPanAngle;
        shoulderPanMsg.data = shoulderPanMsg.data<3 ? shoulderPanMsg.data + 0.1 : -3.0;
        
        elbow_pub.publish(elbowMsg);
        shoulderPan_pub.publish(shoulderPanMsg);

        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO("\nelbow: %f %f\nshoulderSpan: %f %f", elbowMsg.data, elbowAngle, shoulderPanMsg.data, shoulderPanAngle);
        usleep(100000);
    }

    return 0;
}