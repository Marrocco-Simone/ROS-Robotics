//TODO: aggiungere gripper al robot e iscriversi ai suoi topics
//TODO: dividere in headers diversi
//TODO: scrivere a mano la libreria kdl

//constants
#define nJoints 6
#define refresh 100
#define string std::string

//ros
#include <ros/ros.h>
#include <ros/package.h>

//messages
#include <std_msgs/Float64.h>
    #define Message std_msgs::Float64
#include <std_msgs/String.h> //possibly useless

//kdl libraries
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
    #define ForwardKinematicPos_Solver KDL::ChainFkSolverPos_recursive
#include <kdl/chainiksolverpos_nr.hpp>
    #define InverseKinematicPos_Solver KDL::ChainIkSolverPos_NR
#include <kdl/chainiksolvervel_pinv.hpp>
    #define InverseKinematicVel_Solver KDL::ChainIkSolverVel_pinv
#include <kdl/frames.hpp>
    #define Coordinate KDL::Frame
    #define CoordinateVector KDL::Vector
#include <kdl/jntarray.hpp>
    #define JointArray KDL::JntArray





//some callback functions for the nodeHandler subscriber
#include <control_msgs/JointControllerState.h>
#define JointState control_msgs::JointControllerState::ConstPtr
void get_shoulderPan_jointPos(const JointState &ctrl_msg){
    jointStart(0) = ctrl_msg->process_value;
}
void get_shoulderLift_jointPos(const JointState &ctrl_msg){
    jointStart(1) = ctrl_msg->process_value;
}
void get_Elbow_jointPos(const JointState &ctrl_msg){
    jointStart(2) = ctrl_msg->process_value;
}
void get_Wrist1_jointPos(const JointState &ctrl_msg){
    jointStart(3) = ctrl_msg->process_value;
}
void get_Wrist2_jointPos(const JointState &ctrl_msg){
    jointStart(4) = ctrl_msg->process_value;
}
void get_Wrist3_jointPos(const JointState &ctrl_msg){
    jointStart(5) = ctrl_msg->process_value;
}

////////////////////////////////////////////////////////////
float compute_path(double singleJointStart, double singleJointGoal, float t, float t_max) {
	return((singleJointGoal - singleJointStart) * (t/t_max) + singleJointStart);
}

//here the AI should put the next move coordinates
void get_goal_relative_coordinates(float *x, float *y, float *z){
    char dummy; std::cout<<"move the robot"; std::cin>>dummy;
    *x=0.1;
    *y=0.1;
    *z=0.1;
}

CoordinateVector get_goal(Coordinate coordStart){
    float x,y,z;
    get_goal_relative_coordinates(&x,&y,&z);

    CoordinateVector v (
        coordStart.p(0) + x,
        coordStart.p(1) + y,
        coordStart.p(2) + z
    );

    return v;
}

JointArray jointStart(nJoints);

int main(int argc, char **argv){
    string urdf_path = ros::package::getPath("ur5-joint-position-control");
    if(urdf_path.empty()) ROS_ERROR("ur5-joint-position-control package path was not found");
    urdf_path += "/urdf/ur5_jnt_pos_ctrl.urdf";
    ros::init(argc, argv, "tcp_control");

	ros::NodeHandle nodeHandler;

	ros::Rate loop_rate(refresh);

    //Create subscribers for all joint states
	ros::Subscriber shoulderPan_sub = nodeHandler.subscribe("/shoulder_pan_joint_position_controller/state", 1000, get_shoulderPan_jointPos);
	ros::Subscriber shoulderLift_sub = nodeHandler.subscribe("/shoulder_lift_joint_position_controller/state", 1000, get_shoulderLift_jointPos);
	ros::Subscriber elbow_sub = nodeHandler.subscribe("/elbow_joint_position_controller/state", 1000, get_Elbow_jointPos);
	ros::Subscriber wrist1_sub = nodeHandler.subscribe("/wrist_1_joint_position_controller/state", 1000, get_Wrist1_jointPos);
	ros::Subscriber wrist2_sub = nodeHandler.subscribe("/wrist_2_joint_position_controller/state", 1000, get_Wrist2_jointPos);
	ros::Subscriber wrist3_sub = nodeHandler.subscribe("/wrist_3_joint_position_controller/state", 1000, get_Wrist3_jointPos);

    //Create publishers to send position commands to all joints
	ros::Publisher commander[nJoints]; 
	commander[0] = nodeHandler.advertise<Message>("/shoulder_pan_joint_position_controller/command", 1000);
	commander[1] = nodeHandler.advertise<Message>("/shoulder_lift_joint_position_controller/command", 1000);
	commander[2] = nodeHandler.advertise<Message>("/elbow_joint_position_controller/command", 1000);
	commander[3] = nodeHandler.advertise<Message>("/wrist_1_joint_position_controller/command", 1000);
	commander[4] = nodeHandler.advertise<Message>("/wrist_2_joint_position_controller/command", 1000);
	commander[5] = nodeHandler.advertise<Message>("/wrist_3_joint_position_controller/command", 1000);

    //Parse urdf model and generate KDL tree
    KDL::Tree tree;
    if(!kdl_parser::treeFromFile(urdf_path,tree)) ROS_ERROR("Failed to construct kdl tree");

    //Generate a kinematic chain from the robot base to its tcp
    KDL::Chain chain;
    tree.getChain("base_link","wrist_3_link",chain);

    //Create solvers
    ForwardKinematicPos_Solver fkPos_solver(chain);
    InverseKinematicVel_Solver ikVel_solver(chain,0.0001,1000);
    InverseKinematicPos_Solver ikPos_solver(chain,fkPos_solver,ikVel_solver,1000);

    //Make sure we have received proper joint angles already
	ros::spinOnce();    loop_rate.sleep();
    ros::spinOnce();    loop_rate.sleep();

    while (ros::ok()) {

        //Compute current cartesian position
        Coordinate coordStart;
        fkPos_solver.JntToCart(jointStart, coordStart);

        ROS_INFO("Current tcp Position/Twist KDL:");		
		ROS_INFO("Position: %f %f %f", coordStart.p(0), coordStart.p(1), coordStart.p(2));		
		ROS_INFO("Orientation: %f %f %f", coordStart.M(0,0), coordStart.M(1,0), coordStart.M(2,0));

        //get user input
        float t_max = 3.0;
        Coordinate coordGoal(coordStart.M,get_goal(coordStart));

        //Compute inverse kinematics
        JointArray jointGoal(nJoints);
        ikPos_solver.CartToJnt(jointStart, coordGoal, jointGoal);

        for(float t=0.0; t<t_max; t+=1/refresh){

            Message newPosition[nJoints];
            //Compute next position step for all joints
			for(int i=0; i<nJoints; i++) {
                newPosition[i].data = compute_path(jointStart(i), jointGoal(i), t, t_max);
                commander[i].publish(newPosition[i]);
            }

            ros::spinOnce();    loop_rate.sleep();
        }
    }

    return 0;
}