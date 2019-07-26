#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif


// ROS inludes
#include <ros/ros.h>
#include <ros/package.h>

// ROS msgs
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "tf2_msgs/TFMessage.h"
#include "stabiliplus/polytope.h"
#include "stabiliplus/plane.h"

// Stabiliplus includes
#include "robot.h"
#include "stability_polytope.h"

// other includes
#include <chrono>

void updateRobotContacts_callback(const tf2_msgs::TFMessage::ConstPtr& msg, Robot & robot)
{
    // Hard code the configuration from the rsdf file
    //Left hand
    double LH_phi(0.0), LH_theta(-M_PI/2), LH_psi(0.0);
    double LH_c1(std::cos(LH_phi)), LH_c2(std::cos(LH_theta)), LH_c3(std::cos(LH_psi));
    double LH_s1(std::sin(LH_phi)), LH_s2(std::sin(LH_theta)), LH_s3(std::sin(LH_psi));
    Eigen::Matrix4d LHand_origin;

    LHand_origin << c2*c3, -c2*s3, s2, 0.211,
                    c1*s3 + c3*s1*s2, c1*c3 - s1*s2*s3, -c2*s1, 0.0,
                    s1*s3 - c1*c3*s2, c3*s1 + c1*s2*s3, c1*c2, 0.0,
                    0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4d LHand_th1 = LHand_origin * Eigen::Vector4d([0.026, 0.026, 0.0, 1.0])
    Eigen::Matrix4d LHand_th2 = LHand_origin * Eigen::Vector4d([0.026, -0.026, 0.0, 1.0])
    Eigen::Matrix4d LHand_th3 = LHand_origin * Eigen::Vector4d([-0.026, -0.026, 0.0, 1.0])
    Eigen::Matrix4d LHand_th4 = LHand_origin * Eigen::Vector4d([-0.026, 0.026, 0.0, 1.0])

    //Right hand
    double RH_phi(0.0), RH_theta(-M_PI/2), RH_psi(0.0);
    double RH_c1(std::cos(RH_phi)), RH_c2(std::cos(RH_theta)), RH_c3(std::cos(RH_psi));
    double RH_s1(std::sin(RH_phi)), RH_s2(std::sin(RH_theta)), RH_s3(std::sin(RH_psi));
    Eigen::Matrix4d RHand_origin;

    RHand_origin << c2*c3, -c2*s3, s2, 0.211,
                    c1*s3 + c3*s1*s2, c1*c3 - s1*s2*s3, -c2*s1, 0.0,
                    s1*s3 - c1*c3*s2, c3*s1 + c1*s2*s3, c1*c2, 0.0,
                    0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4d RHand_th1 = RHand_origin * Eigen::Vector4d([0.026, 0.026, 0.0, 1.0])
    Eigen::Matrix4d RHand_th2 = RHand_origin * Eigen::Vector4d([0.026, -0.026, 0.0, 1.0])
    Eigen::Matrix4d RHand_th3 = RHand_origin * Eigen::Vector4d([-0.026, -0.026, 0.0, 1.0])
    Eigen::Matrix4d RHand_th4 = RHand_origin * Eigen::Vector4d([-0.026, 0.026, 0.0, 1.0])

    //Left foot
    double LF_phi(0.0), LF_theta(0.0), LF_psi(0.0);
    double LF_c1(std::cos(LF_phi)), LF_c2(std::cos(LF_theta)), LF_c3(std::cos(LF_psi));
    double LF_s1(std::sin(LF_phi)), LF_s2(std::sin(LF_theta)), LF_s3(std::sin(LF_psi));
    Eigen::Matrix4d LFoot_origin;

    LFoot_origin << c2*c3, -c2*s3, s2, 0.0,
                    c1*s3 + c3*s1*s2, c1*c3 - s1*s2*s3, -c2*s1, 0.0,
                    s1*s3 - c1*c3*s2, c3*s1 + c1*s2*s3, c1*c2, -0.101,
                    0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4d LFoot_th1 = LFoot_origin * Eigen::Vector4d([0.100, 0.065, 0.0, 1.0])
    Eigen::Matrix4d LFoot_th2 = LFoot_origin * Eigen::Vector4d([0.100, -0.045, 0.0, 1.0])
    Eigen::Matrix4d LFoot_th3 = LFoot_origin * Eigen::Vector4d([-0.060, -0.045, 0.0, 1.0])
    Eigen::Matrix4d LFoot_th4 = LFoot_origin * Eigen::Vector4d([-0.060, 0.065, 0.0, 1.0])

    //Right Foot
    double RF_phi(0.0), RF_theta(-M_PI/2), RF_psi(0.0);
    double RF_c1(std::cos(RF_phi)), RF_c2(std::cos(RF_theta)), RF_c3(std::cos(RF_psi));
    double RF_s1(std::sin(RF_phi)), RF_s2(std::sin(RF_theta)), RF_s3(std::sin(LH_psi));
    Eigen::Matrix4d RFoot_origin;

    RFoot_origin << c2*c3, -c2*s3, s2, 0.211,
                    c1*s3 + c3*s1*s2, c1*c3 - s1*s2*s3, -c2*s1, 0.0,
                    s1*s3 - c1*c3*s2, c3*s1 + c1*s2*s3, c1*c2, 0.0,
                    0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4d RFoot_th1 = RFoot_origin * Eigen::Vector4d([0.100, 0.045, 0.0, 1.0])
    Eigen::Matrix4d RFoot_th2 = RFoot_origin * Eigen::Vector4d([0.100, -0.065, 0.0, 1.0])
    Eigen::Matrix4d RFoot_th3 = RFoot_origin * Eigen::Vector4d([-0.060, -0.065, 0.0, 1.0])
    Eigen::Matrix4d RFoot_th4 = RFoot_origin * Eigen::Vector4d([-0.060, 0.045, 0.0, 1.0])
}

int main(int argc, char *argv[])
{
    // init ros node
    ros::init(argc, argv, "stabiliplus_node");
    ros::NodeHandle n;

    ros::Publisher currentPolytopePublisher = n.advertise<stabiliplus::polytope>("current_stability_polytope", 10);
    ros::Subscriber currentRobotPosition = n.subscribe("/tf", 1, updateRobotContacts_callback)
    ros::Rate loop_rate(10);

    // init robot
    std::string stabiliplus_path = ros::package::getPath("stabiliplus");
    std::string file_robot = stabiliplus_path + "/robots/robot_8.xml";
    Robot robot(file_robot, 16);

    int seq = 0;
    while (ros::ok())
    {
        // create and compute polytope:
        auto start = std::chrono::high_resolution_clock::now();
        std::shared_ptr<StabilityPolytope> polytope(new StabilityPolytope(robot));

        polytope->buildStabilityProblem();
        polytope->projectionStabilityPolyhedron();

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);

        auto faceNormals = polytope->get_innerFaceNormals();
        auto faceOffsets = polytope->get_innerFaceOffsets();


        // Publishing the result in ROS

        stabiliplus::polytope polytope_msg;
        // filling the header
        polytope_msg.header.seq=seq;
        polytope_msg.header.stamp = ros::Time::now();
        // filling the planes
        for (int i=0; i<polytope->get_numberOfFaces(); i++)
        {
            stabiliplus::plane plane_msg;
            plane_msg.normal.x = faceNormals[i][0];
            plane_msg.normal.y = faceNormals[i][1];
            plane_msg.normal.z = faceNormals[i][2];
            plane_msg.offset = faceOffsets[i];

            polytope_msg.planes.push_back(plane_msg);
        }

        currentPolytopePublisher.publish(polytope_msg);
        ros::spinOnce();

        loop_rate.sleep();

        seq++;

    }



    return 0;
}
