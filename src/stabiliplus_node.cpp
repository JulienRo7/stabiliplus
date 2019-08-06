#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif


// ROS inludes
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>


// ROS msgs
#include <std_msgs/Float64.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>

#include "stabiliplus/polytope.h"
#include "stabiliplus/plane.h"
#include "stabiliplus/contact.h"
#include "stabiliplus/contactSet.h"

// Stabiliplus includes
#include "robot.h"
#include "stability_polytope.h"

// other includes
#include <chrono>
#include <cmath>


class RosRobot : public Robot
{
public:

    RosRobot(std::string const& robot_file_name, int numFrictionSides, tf2_ros::Buffer* tfBuffer):
     Robot::Robot(robot_file_name, numFrictionSides),
     m_tfBuffer(tfBuffer)
    {
        // Hard code the configuration from the rsdf file
        //Left hand
        Eigen::Matrix4d LHand_origin = Eigen::Matrix4d::Zero();
        LHand_origin.topLeftCorner<3,3>() = Euler2RotMat(0.0, -M_PI/2, 0.0);
        LHand_origin.rightCols<1>()<< 0.211, 0.0, 0.0, 1.0;

        Eigen::Matrix4d p1 = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d p2 = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d p3 = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d p4 = Eigen::Matrix4d::Identity();

        p1.rightCols<1>().head<3>() << 0.026, 0.026, 0.0;
        p2.rightCols<1>().head<3>() << 0.026, -0.026, 0.0;
        p3.rightCols<1>().head<3>() << -0.026, -0.026, 0.0;
        p4.rightCols<1>().head<3>() << -0.026, 0.026, 0.0;

        LHand_th1 = LHand_origin * p1;
        LHand_th2 = LHand_origin * p2;
        LHand_th3 = LHand_origin * p3;
        LHand_th4 = LHand_origin * p4;

        //Right hand
        Eigen::Matrix4d RHand_origin = Eigen::Matrix4d::Zero();
        RHand_origin.topLeftCorner<3,3>() = Euler2RotMat(0.0, -M_PI/2, 0.0);
        RHand_origin.rightCols<1>()<< 0.211, 0.0, 0.0, 1.0;

        RHand_th1 = RHand_origin * p1;
        RHand_th2 = RHand_origin * p2;
        RHand_th3 = RHand_origin * p3;
        RHand_th4 = RHand_origin * p4;

        //Left foot
        Eigen::Matrix4d LFoot_origin = Eigen::Matrix4d::Zero();
        LFoot_origin.topLeftCorner<3,3>() = Euler2RotMat(0.0, 0.0, 0.0);
        LFoot_origin.rightCols<1>()<< 0.0, 0.0, -0.101, 1.0;

        p1.rightCols<1>().head<3>() << 0.100, 0.065, 0.0;
        p2.rightCols<1>().head<3>() << 0.100, -0.045, 0.0;
        p3.rightCols<1>().head<3>() << -0.060, -0.045, 0.0;
        p4.rightCols<1>().head<3>() << -0.060, 0.065, 0.0;
        LFoot_th1 = LFoot_origin * p1;
        LFoot_th2 = LFoot_origin * p2;
        LFoot_th3 = LFoot_origin * p3;
        LFoot_th4 = LFoot_origin * p4;

        //Right Foot
        Eigen::Matrix4d RFoot_origin = Eigen::Matrix4d::Zero();
        RFoot_origin.topLeftCorner<3,3>() = Euler2RotMat(0.0, 0.0, 0.0);
        RFoot_origin.rightCols<1>()<< 0.0, 0.0, -0.101, 1.0;

        RFoot_th1 = RFoot_origin * p1;
        RFoot_th2 = RFoot_origin * p2;
        RFoot_th3 = RFoot_origin * p3;
        RFoot_th4 = RFoot_origin * p4;
    };

    ~RosRobot()
    {

    };

    void updateRobotContacts()
    {
        updateHomogeneousTransforms();
        if (hasContactNamed("LHand_1"))
        {
            set_contact(get_contactIndexFromName("LHand_1"), m_LArm_WRY*LHand_th1);
        }
        if (hasContactNamed("LHand_2"))
        {
            set_contact(get_contactIndexFromName("LHand_2"), m_LArm_WRY*LHand_th2);
        }
        if (hasContactNamed("LHand_3"))
        {
            set_contact(get_contactIndexFromName("LHand_3"), m_LArm_WRY*LHand_th3);
        }
        if (hasContactNamed("LHand_4"))
        {
            set_contact(get_contactIndexFromName("LHand_4"), m_LArm_WRY*LHand_th4);
        }

        if (hasContactNamed("RHand_1"))
        {
            set_contact(get_contactIndexFromName("RHand_1"), m_RArm_WRY*RHand_th1);
        }
        if (hasContactNamed("RHand_2"))
        {
            set_contact(get_contactIndexFromName("RHand_2"), m_RArm_WRY*RHand_th2);
        }
        if (hasContactNamed("RHand_3"))
        {
            set_contact(get_contactIndexFromName("RHand_3"), m_RArm_WRY*RHand_th3);
        }
        if (hasContactNamed("RHand_4"))
        {
            set_contact(get_contactIndexFromName("RHand_4"), m_RArm_WRY*RHand_th4);
        }

        if (hasContactNamed("LFoot_1"))
        {
            set_contact(get_contactIndexFromName("LFoot_1"), m_LLeg_3X*LFoot_th1);
        }
        if (hasContactNamed("LFoot_2"))
        {
            set_contact(get_contactIndexFromName("LFoot_2"), m_LLeg_3X*LFoot_th2);
        }
        if (hasContactNamed("LFoot_3"))
        {
            set_contact(get_contactIndexFromName("LFoot_3"), m_LLeg_3X*LFoot_th3);
        }
        if (hasContactNamed("LFoot_4"))
        {
            set_contact(get_contactIndexFromName("LFoot_4"), m_LLeg_3X*LFoot_th4);
        }

        if (hasContactNamed("RFoot_1"))
        {
            set_contact(get_contactIndexFromName("RFoot_1"), m_RLeg_3X*RFoot_th1);
        }
        if (hasContactNamed("RFoot_2"))
        {
            set_contact(get_contactIndexFromName("RFoot_2"), m_RLeg_3X*RFoot_th2);
        }
        if (hasContactNamed("RFoot_3"))
        {
            set_contact(get_contactIndexFromName("RFoot_3"), m_RLeg_3X*RFoot_th3);
        }
        if (hasContactNamed("RFoot_4"))
        {
            set_contact(get_contactIndexFromName("RFoot_4"), m_RLeg_3X*RFoot_th4);
        }
    };

    void updateRobotContacts_callback(const stabiliplus::contactSet::ConstPtr& msg)
    {
        std::vector<std::string> currentContactPoints, previousContactPoints;
        for (auto contact: msg->contactSet)
        {
            for (auto name: get_currentContactNames(contact))
            {
                currentContactPoints.push_back(name);
            }
        }
        previousContactPoints = get_contactNames();

        std::vector<std::string> newContacts, oldContacts;
        // finding contacts to add

        for (auto contact: currentContactPoints)
        {
            if (find(previousContactPoints.begin(), previousContactPoints.end(), contact)==previousContactPoints.end())
            {
                newContacts.push_back(contact);
            }
        }
        //finding contacts to remove
        for (auto contact: previousContactPoints)
        {
            // std::cout << contact << '\n';
            if (find(currentContactPoints.begin(), currentContactPoints.end(), contact)==currentContactPoints.end())
            {
                oldContacts.push_back(contact);
            }
        }

        // std::cout << "new contacts: " << '\n';
        for (auto name: newContacts)
        {
            // std::cout << name << '\n';
            addContact(name);
        }
        // std::cout << "old contacts: " << '\n';
        for (auto name: oldContacts)
        {
            // std::cout << name << '\n';
            removeContact(name);
        }
    };

    std::vector<std::string> get_currentContactNames(stabiliplus::contact contact) const
    {
        std::vector<std::string> contactNames;

        if (contact.r1Surface == "LeftFoot" && contact.r2Surface == "AllGround")
        {
            contactNames.push_back("LFoot_1");
            contactNames.push_back("LFoot_2");
            contactNames.push_back("LFoot_3");
            contactNames.push_back("LFoot_4");
        }
        else if (contact.r1Surface == "RightFoot" && contact.r2Surface == "AllGround")
        {
            contactNames.push_back("RFoot_1");
            contactNames.push_back("RFoot_2");
            contactNames.push_back("RFoot_3");
            contactNames.push_back("RFoot_4");
        }
        else if (contact.r1Surface == "LeftHand" && contact.r2Surface == "minusXside")
        {
            contactNames.push_back("LHand_1");
            contactNames.push_back("LHand_2");
            contactNames.push_back("LHand_3");
            contactNames.push_back("LHand_4");
        }
        else if (contact.r1Surface == "RightHand" && contact.r2Surface == "minusXside")
        {
            contactNames.push_back("RHand_1");
            contactNames.push_back("RHand_2");
            contactNames.push_back("RHand_3");
            contactNames.push_back("RHand_4");
        }
        else
        {
            std::cerr << "Unrecognised contact!" << '\n';
        }

        return contactNames;
    };

    Eigen::Matrix3d Euler2RotMat(double phi, double theta, double psi) const
    {
        double c1(std::cos(phi)), c2(std::cos(theta)), c3(std::cos(psi));
        double s1(std::sin(phi)), s2(std::sin(theta)), s3(std::sin(psi));
        Eigen::Matrix3d rotMat;

        rotMat << c2*c3, -c2*s3, s2,
                        c1*s3 + c3*s1*s2, c1*c3 - s1*s2*s3, -c2*s1,
                        s1*s3 - c1*c3*s2, c3*s1 + c1*s2*s3, c1*c2;
        return rotMat;
    };

    Eigen::Matrix3d Quaternion2RotMat(double x, double y, double z, double w)
    {
        Eigen::Matrix3d rotMat;
        rotMat << pow(w, 2) + pow(x, 2) - pow(y, 2) - pow(z, 2), 2*x*y - 2*w*z, 2*x*z + 2*w*y,
                  2*x*y + 2*w*z, pow(w, 2) - pow(x, 2) + pow(y, 2) - pow(z, 2), 2*y*z - 2*w*x,
                  2*x*z - 2*w*y, 2*y*z + 2*w*x, pow(w, 2) - pow(x, 2) - pow(y, 2) + pow(z, 2);
        return rotMat;
    }

    void updateHomogeneousTransforms()
    {
        geometry_msgs::TransformStamped LArm_WRY, RArm_WRY, LLeg_3X, RLeg_3X;

        if (m_tfBuffer->canTransform("robot_map", "control/LArm_WRY", ros::Time(0)))
        {
            LArm_WRY = m_tfBuffer->lookupTransform("robot_map", "control/LArm_WRY", ros::Time(0));
            m_LArm_WRY = TransformStamped2HomogeneousTransform(LArm_WRY);
            // std::cout << "m_LArm_WRY : " << '\n' << m_LArm_WRY << '\n';
        }
        else
        {
            ROS_WARN("Could not get the LArm_WRY transform");
        }

        if (m_tfBuffer->canTransform("robot_map", "control/RArm_WRY", ros::Time(0)))
        {
            RArm_WRY = m_tfBuffer->lookupTransform("robot_map", "control/RArm_WRY", ros::Time(0));
            m_RArm_WRY = TransformStamped2HomogeneousTransform(RArm_WRY);
            // std::cout << "m_RArm_WRY : " << '\n' << m_RArm_WRY << '\n';
        }
        else
        {
            ROS_WARN("Could not get the RArm_WRY transform");
        }

        if (m_tfBuffer->canTransform("robot_map", "control/LLeg_3X", ros::Time(0)))
        {
            LLeg_3X = m_tfBuffer->lookupTransform("robot_map", "control/LLeg_3X", ros::Time(0));
            m_LLeg_3X = TransformStamped2HomogeneousTransform(LLeg_3X);
            // std::cout << "m_LLeg_3X : " << '\n' << m_LLeg_3X << '\n';
        }
        else
        {
            ROS_WARN("Could not get the LLeg_3X transform");
        }

        if (m_tfBuffer->canTransform("robot_map", "control/RLeg_3X", ros::Time(0)))
        {
            RLeg_3X = m_tfBuffer->lookupTransform("robot_map", "control/RLeg_3X", ros::Time(0));
            m_RLeg_3X = TransformStamped2HomogeneousTransform(RLeg_3X);
            // std::cout << "m_RLeg_3X : " << '\n' << m_RLeg_3X << '\n';
        }
        else
        {
            ROS_WARN("Could not get the RLeg_3X transform");
        }
    }

    Eigen::Matrix4d TransformStamped2HomogeneousTransform(const geometry_msgs::TransformStamped & geomTrans)
    {
        geometry_msgs::Quaternion quat = geomTrans.transform.rotation;
        // std::cout << "Quaternion:" << quat.x << ", " << quat.y << ", " << quat.z << ", " << quat.w << '\n';
        geometry_msgs::Vector3 trans = geomTrans.transform.translation;
        // std::cout << "Translation: " << trans.x << ", " << trans.y << ", " << trans.z << '\n';

        Eigen::Matrix4d homTrans = Eigen::Matrix4d::Zero();
        homTrans.topLeftCorner<3,3>() = Quaternion2RotMat(quat.x, quat.y, quat.z, quat.w);
        homTrans.rightCols<1>()<< trans.x, trans.y, trans.z, 1.0;

        return homTrans;
    };

private:
    tf2_ros::Buffer* m_tfBuffer;

    Eigen::Matrix4d  m_LArm_WRY;
    Eigen::Matrix4d  m_RArm_WRY;
    Eigen::Matrix4d  m_LLeg_3X;
    Eigen::Matrix4d  m_RLeg_3X;

    Eigen::Matrix4d LHand_th1;
    Eigen::Matrix4d LHand_th2;
    Eigen::Matrix4d LHand_th3;
    Eigen::Matrix4d LHand_th4;
    Eigen::Matrix4d RHand_th1;
    Eigen::Matrix4d RHand_th2;
    Eigen::Matrix4d RHand_th3;
    Eigen::Matrix4d RHand_th4;
    Eigen::Matrix4d LFoot_th1;
    Eigen::Matrix4d LFoot_th2;
    Eigen::Matrix4d LFoot_th3;
    Eigen::Matrix4d LFoot_th4;
    Eigen::Matrix4d RFoot_th1;
    Eigen::Matrix4d RFoot_th2;
    Eigen::Matrix4d RFoot_th3;
    Eigen::Matrix4d RFoot_th4;
};



int main(int argc, char *argv[])
{
    // init ros node
    ros::init(argc, argv, "stabiliplus_node");
    ros::NodeHandle n;

    // init robot
    std::string stabiliplus_path = ros::package::getPath("stabiliplus");
    std::string file_robot = stabiliplus_path + "/robots/proto2_pushUp.xml";

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    RosRobot robot(file_robot, 16, &tfBuffer);

    // init publisher
    ros::Publisher currentPolytopePublisher = n.advertise<stabiliplus::polytope>("current_stability_polytope", 10);
    ros::Subscriber currentRobotContacts = n.subscribe("current_contact_set", 1, &RosRobot::updateRobotContacts_callback, &robot);
    ros::Rate loop_rate(10);

    std::string poly_file_name, robot_file_name;
    int seq = 0;
    while (ros::ok())
    {
        // std::cout << "Computation: " << seq << '\n';
        // update the robot position
        robot.updateRobotContacts();

        // create and compute polytope:
        auto start = std::chrono::high_resolution_clock::now();
        std::shared_ptr<StabilityPolytope> polytope(new StabilityPolytope(robot, 50));

        polytope->buildStabilityProblem();
        polytope->projectionStabilityPolyhedron();

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);

        auto faceNormals = polytope->get_innerFaceNormals();
        auto faceOffsets = polytope->get_innerFaceOffsets();

        std::cout << "Number of faces : " << faceOffsets.size() << '\n';

        // poly_file_name = "/tmp/polytopes/polytope_"+std::to_string(seq)+".txt";
        // robot_file_name = "/tmp/robots/robot_"+std::to_string(seq)+".xml";
        // polytope->exportVertices(poly_file_name);
        // polytope->get_robot()->saveRobot(robot_file_name);

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
