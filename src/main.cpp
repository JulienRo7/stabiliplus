#include "robot.h"

#include <time.h>


using namespace std;

/*
Beginning of a new journey in the (scary) worlf of C++
*/

int main()
{

    clock_t start, end;
    double cpu_time_used;


    start = clock();
    Robot robot("../robots/robot_2.xml", 4);
    // robot.buildStabilityProblem();
    robot.buildReducedStabilityProblem();
    robot.projectionStabilityPolyhedron();
    end = clock();

    robot.exportVertices();



    cpu_time_used = ((double) (end - start)) / (CLOCKS_PER_SEC/1000);

    std::cout << "Computation time: " << cpu_time_used << "ms for " << robot.get_numberOfVertices() << "Vertices" << '\n';

    return 0;
}
