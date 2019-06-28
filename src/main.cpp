#include "robot.h"

#include <Eigen/Dense>

using namespace std;

/*
Beginning of a new journey in the (scary) worlf of C++
*/

int main()
{
    Robot robot("../robots/robot_2.xml", 16);
    // robot.showRobot();s

    robot.buildStabilityProblem();

    robot.projectionStabilityPolyhedron();

    robot.exportVertices();

    return 0;
}
