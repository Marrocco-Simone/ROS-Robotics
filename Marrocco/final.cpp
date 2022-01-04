#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>
#define PI 3.14159265358979323846
#include "control_msgs/JointControllerState.h"
#include <unistd.h> //usleep
#include <Eigen/Dense>

/* 
    /shoulder_pan_joint_position_controller
    /shoulder_lift_joint_position_controller
    /elbow_joint_position_controller
    /wrist_1_joint_position_controller
    /wrist_2_joint_position_controller
    /wrist_3_joint_position_controller 
*/
const int n = 6;

//lenghts of joints
//a[0] = a0 in DH table - so the index must be the the one in the kinematic pdf
float a[]   = {
    0,
    0,
    -0.42500,
    -0.39225,
    0,
    0
};
//d[0] = d1 in DH table - so the index must be -1 the one in the kinematic pdf
float d[]   = {
    0.0891589,
    0,
    0,
    0.10915,
    0.09456,
    0.0823
};
//alpha[0] = alpha0 in DH table - so the index must be the the one in the kinematic pdf
float alpha[] = {0,PI/2,0,0,PI/2,-PI/2};

//current and goal angle configuration
float current_angle[n], goal_angle[n];

//callback functions to get the current angle
void shoulderPan_callback(const control_msgs::JointControllerState::ConstPtr& msg)  { current_angle[0] = msg->set_point; }
void shoulderLift_callback(const control_msgs::JointControllerState::ConstPtr& msg) { current_angle[1] = msg->set_point; }
void elbow_callback(const control_msgs::JointControllerState::ConstPtr& msg)        { current_angle[2] = msg->set_point; }
void wrist1_callback(const control_msgs::JointControllerState::ConstPtr& msg)       { current_angle[3] = msg->set_point; }
void wrist2_callback(const control_msgs::JointControllerState::ConstPtr& msg)       { current_angle[4] = msg->set_point; }
void wrist3_callback(const control_msgs::JointControllerState::ConstPtr& msg)       { current_angle[5] = msg->set_point; }

float *pointsDistance(float *start, float *finish){
    float v[3];
    for(int i=0;i<3;i++){
        v[i] = start[i] - finish[i];
        if(v[i]<0) v[i] *= -1;
    }
    return v;
}

Eigen::VectorXf arrayToVector(float *a, int dim){
    Eigen::VectorXf v(dim);
    for(int i=0;i<dim;i++) v(i) = a[i];
    return v;
}

float *vectorToArray(Eigen::Vector3f v, int dim){
    float a[dim];
    for(int i=0;i<dim;i++) a[i] = v(i);
    return a;
}

//transformation matrix between two near frames calculator using the DH parameters
Eigen::Matrix4f transMatrix(int alpha,float a,float d,float teta){
    Eigen::Matrix4f T;

    T(0,0)=cos(teta);
    T(0,1)=sin(teta) * -1;
    T(0,2)=0;
    T(0,3)=a;

    T(1,0)=sin(teta) * cos(alpha);
    T(1,1)=cos(teta) * cos(alpha);
    T(1,2)=sin(alpha) * -1;
    T(1,3)=d * sin(alpha) * -1;

    T(2,0)=sin(teta) * sin(alpha);
    T(2,1)=cos(teta) * sin(alpha);
    T(2,2)=cos(alpha);
    T(2,3)=d * cos(alpha);

    T(3,0)=0;
    T(3,1)=0;
    T(3,2)=0;
    T(3,3)=1;

    return T;
}

//return the origin coordinate of the last frame from the first frame by taking the specific vector from T
float* getCoordFinalFrame(Eigen::Matrix4f T){
    //the position of the base of the last frame in the first frame
    Eigen::Vector3f current_pos_M = T.block(0,3,3,1);

    float *current_pos = vectorToArray(current_pos_M,3);
    return current_pos;
}

/*
    rotation matrix is the first 3x3. Every column is the coordinates on the first frame of the axis of the last frame. 
    Ex: first column is the position in the start frame of (x,y,z)=(1,0,0) in the last frame if the origins would be the same
        If not, you need to add the origins distance
*/
Eigen::Matrix3f getRotationMatrix(Eigen::Matrix4f T){
    Eigen::Matrix3f rotation_M = T.block(0,0,3,3);
    return rotation_M;
}

/* 
    returns the transformation matrix from frame 6 to frame 0
    input: the array of angles of each joint
    logic:
        first we set up the DH parameters table using different arrays
        then we calculate the single transformation matrixes from the i frame to the i+1 frame
        we multiply all of them to get the transformation matrix from frame 6 to frame 0
*/
Eigen::Matrix4f fwdKin(float *teta){
    //transformation matrix: T[i] = T from i to i+1
    Eigen::Matrix4f T[n];
    for(int i=0;i<n;i++){
        T[i] = transMatrix(alpha[i],a[i],d[i],teta[i]);
    }

    //transformation matrix from frame 6 to frame 0
    Eigen::Matrix4f T06 = T[0];
    for(int i=1;i<n;i++) T06 = T06 * T[i];
    return T06;
}

/* 
    Converts a point on frame 6 to a point on frame 0 via the formula 
        p0 = rotMat06 * p6 + distance06
    input: 
        the array of position of the goal given locally on frame 6 by the camera
        the transformation matrix from frame 6 to frame 0
*/
float *getCoordGoalFrame6(float *relative_goal_pos, Eigen::Matrix4f T06){
    Eigen::Vector3f relative_goal_pos_M = arrayToVector(relative_goal_pos,3);
    float *frame6_pos = getCoordFinalFrame(T06);
    Eigen::Matrix3f rotation_M = getRotationMatrix(T06);

    //calculate distance between base and the frame 6 origin position
    float base[3] = {0,0,0};
    float *frameDistance = pointsDistance(base, frame6_pos);
    Eigen::Vector3f frameDistance_M = arrayToVector(frameDistance,3);

    //formula
    Eigen::Vector3f goal_pos_M = rotation_M * relative_goal_pos_M + frameDistance_M;

    float *goal_pos = vectorToArray(goal_pos_M,3);
    return goal_pos;
}

/*
    calculate set of angle based on final end effector (aka new frame 6 origin) coordinates and orientation
    input:
        the coordinates of the goal point from frame 0
        the rotation matrix of the coordinates of the frame 6 axis from frame 0
        the current angle configuration
    logic:
        first we create a new T06 transformation matrix based on our goal position and orientation
        then we calculate the necessary components for the inverse kinematic angles
        specifics on the kinematic pdf
*/
float *invKin(float *frame6_pos, Eigen::Matrix3f frame6_rotMatrix){
    //goal transformation matrix
    Eigen::Matrix4f T06;
    T06.block(0,3,3,1) = arrayToVector(frame6_pos,3);
    T06.block(0,0,3,3) = frame6_rotMatrix;
        T06(3,0) = 0;
        T06(3,1) = 0;
        T06(3,2) = 0;
        T06(3,3) = 1;
    
    //equations
    float teta[n];

        //needed for teta[0]
        Eigen::Vector4f frame5_pos_M_converter;
            frame5_pos_M_converter(0) = 0;
            frame5_pos_M_converter(1) = 0;
            frame5_pos_M_converter(2) = d[5] * -1;
            frame5_pos_M_converter(3) = 1;
        Eigen::Vector4f frame5_pos_M = T06 * frame5_pos_M_converter;
        //it contains the three coordinatees. frame5_pos[3] = 1
        float *frame5_pos = vectorToArray(frame5_pos_M,4);

        //possibile solution, with a {- acos()}: shoulder being "left" or "right"
        teta[0] = PI/2 + atan2(frame5_pos[1],frame5_pos[0]) + 
            acos( d[3] / sqrt( frame5_pos[1]*frame5_pos[1] + frame5_pos[0]*frame5_pos[0] ) );

        //possibile solution, with a {- acos()}: wrist being "up" or "down"
        teta[4] = acos(     (frame6_pos[0]*sin(teta[0]) - frame6_pos[1]*cos(teta[0]) - d[3]) / d[5] );

        //needed for teta[5]
        Eigen::Vector3f X06 = T06.block(0,0,3,1);
        float *X06v = vectorToArray(X06,3);
        Eigen::Vector3f Y06 = T06.block(0,1,3,1);
        float *Y06v = vectorToArray(Y06,3);

        //if teta[4] = 0 or 2PI we got an error: how to fix it? can the joint even move there?
        //this is the gripper orientation angle: it should be moved differently, based on object-to-pick orientation
        //should we give it a default value and then modify it based on camera AI?
        if(sin(teta[4]) != 0)
        teta[5] = atan2(    ( Y06v[1]*cos(teta[0]) - X06v[1]*sin(teta[0]) ) / sin(teta[4]), 
                            ( X06v[0]*sin(teta[0]) - Y06v[0]*cos(teta[0]) ) / sin(teta[4])  );
        else
        teta[5] = 0;

        //needed for teta[2] and teta[1]
            //A=X*B --> X=A*B.inverse()
            //A=B*X --> X=B.inverse()*A
        Eigen::Matrix4f T01 = transMatrix(alpha[0],a[0],d[0],teta[0]);
        Eigen::Matrix4f T45 = transMatrix(alpha[4],a[4],d[4],teta[4]);
        Eigen::Matrix4f T56 = transMatrix(alpha[5],a[5],d[5],teta[5]);
        Eigen::Matrix4f T46 = T45 * T56;
        Eigen::Matrix4f T14 = T01.inverse() * T06 * T46.inverse();

        float *p14 = getCoordFinalFrame(T14);
        float p14xy_distance = sqrt(p14[0]*p14[0] + p14[2]*p14[2]);

        //possible solution, with a {- acos()}: elbow being "up" and "down" 
        teta[2] = acos( (p14xy_distance*p14xy_distance - a[2]*a[2] - a[3]*a[3]) / 2*a[2]*a[3] );

        teta[1] = atan2(p14[2]*-1, p14[0]*-1) - asin(a[3] * -1 * sin(teta[2]) / p14xy_distance);

        //needed for teta[3]
        Eigen::Matrix4f T12 = transMatrix(alpha[1],a[1],d[1],teta[1]);
        Eigen::Matrix4f T23 = transMatrix(alpha[2],a[2],d[2],teta[2]);
        Eigen::Matrix4f T13 = T12 * T23;
        Eigen::Matrix4f T34 = T13.inverse() * T14;

        Eigen::Vector3f X34 = T34.block(0,0,3,1);
        float *X34v = vectorToArray(X34,3);

        teta[3] = atan2(X34v[2], X34v[1]);

    return teta;
}

/* 
    modifies goal to the angle configuration needed to reach the goal point
    input: the coordinates of the point to reach (may change the format, but we need a float array of them in the end)
    logic:
        first we get the coordinates from frame 0 of end effector and goal
*/
void move(float x, float y, float z){
    //save input in float array format
    float relative_goal_pos[3] = {x,y,z};

    //current is highly unstable, so we save it locally
    float t_current_angle[n];
    for(int i=0;i<n;i++) t_current_angle[i] = current_angle[i];

    //calculate forward kinematics to get the coordinates of the end effector and the goal from the frame 0, on the base
    Eigen::Matrix4f T06 = fwdKin(t_current_angle);
    float *current_pos = getCoordFinalFrame(T06);
    float *goal_pos = getCoordGoalFrame6(relative_goal_pos, T06);

    //inverse kinematics to find the new angles for each joint
    Eigen::Matrix3f goal_rotMatrix = getRotationMatrix(T06); //this needs to be the desired orientation of the end effector
    float *t_goal_angle = invKin(goal_pos, goal_rotMatrix);

    //publish changes
    for(int i=0;i<n;i++) goal_angle[i] = t_goal_angle[i];
}

int main(int argc, char **argv){
    ros::init(argc,argv,"modificareconnomefileinpacchetto");
    ros::NodeHandle node;
    ros::Publisher pub[n];
        pub[0] = node.advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command",10);
        pub[1] = node.advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command",10);
        pub[2] = node.advertise<std_msgs::Float64>("/elbow_joint_position_controller/command",10);
        pub[3] = node.advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command",10);
        pub[4] = node.advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command",10);
        pub[5] = node.advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command",10);
    ros::Subscriber sub[n];
        sub[0] = node.subscribe("/shoulder_pan_joint_position_controller/state",10,shoulderPan_callback);
        sub[1] = node.subscribe("/shoulder_lift_joint_position_controller/state",10,shoulderLift_callback);
        sub[2] = node.subscribe("/elbow_joint_position_controller/state",10,elbow_callback);
        sub[3] = node.subscribe("/wrist_1_joint_position_controller/state",10,wrist1_callback);
        sub[4] = node.subscribe("/wrist_2_joint_position_controller/state",10,wrist2_callback);
        sub[5] = node.subscribe("/wrist_3_joint_position_controller/state",10,wrist3_callback);
    ros::Rate loop_rate(10);

    while(ros::ok()){
        move(0, 0.1, 0);

        //publish and change angle configuration
        std_msgs::Float64 msg;

        for(int i=0;i<n;i++){
            msg.data = goal_angle[i];
            pub[i].publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}