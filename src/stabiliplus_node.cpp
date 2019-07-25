
// ROS inludes
#include <ros/ros.h>
#include <ros/package.h>

// ROS msgs
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "stabiliplus/polytope.h"
#include "stabiliplus/plane.h"

// Stabiliplus includes
#include "robot.h"
#include "stability_polytope.h"

// other includes
#include <chrono>

int main(int argc, char *argv[])
{
    // init ros node
    ros::init(argc, argv, "stabiliplus_node");
    ros::NodeHandle n;

    ros::Publisher currentPolytopePublisher = n.advertise<stabiliplus::polytope>("current_stability_polytope", 10);

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
