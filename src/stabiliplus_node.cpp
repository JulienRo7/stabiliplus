
// ROS inludes
#include <ros/ros.h>
#include <ros/package.h>

// ROS msgs
#include "std_msgs/Float64.h"
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

    // ros::Publisher currentPolytopePublisher = n.advertise<stabiliplus::polytope>("current_polytope", 10);
    ros::Publisher computingTime_Publisher = n.advertise<std_msgs::Float64>("computingTime", 1000);

    ros::Rate loop_rate(10);
    int count = 0;

    // init robot
    std::string stabiliplus_path = ros::package::getPath("stabiliplus");
    std::string file_robot = stabiliplus_path + "/robots/robot_8.xml";
    Robot robot(file_robot, 16);


    while (ros::ok())
    {
        // create and compute polytope:
        auto start = std::chrono::high_resolution_clock::now();
        std::shared_ptr<StabilityPolytope> polytope(new StabilityPolytope(robot));

        polytope->buildStabilityProblem();
        polytope->projectionStabilityPolyhedron();

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);

        std_msgs::Float64 msg;

        msg.data = duration.count();

        ROS_INFO("Computation time: %f us", msg.data);

        computingTime_Publisher.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();

        count++;

    }



    return 0;
}
