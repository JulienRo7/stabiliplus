#include "robot.h"

#include <time.h>
// #include <chrono>

using namespace std;

/*
Beginning of a new journey in the (scary) worlf of C++
*/

int main()
{

    clock_t start, end;
    double cpu_time_used;

    start = clock();
    Robot robot("../robots/robot_2.xml", 16);
    // robot.buildStabilityProblem();
    robot.buildReducedStabilityProblem();
    robot.projectionStabilityPolyhedron();
    end = clock();

    robot.exportVertices();

    cpu_time_used = ((double) (end - start)) / (CLOCKS_PER_SEC/1000);

    std::cout << "Computation time: " << cpu_time_used << "ms for " << robot.get_numberOfVertices() << " Vertices" << '\n';

    // std::cout << "Testing stuff..." << '\n';
    //
    // int const maxNumberOfIterations(100);
    // int const numberOfTrials(1);
    //
    // std::vector<double> lpTimes(maxNumberOfIterations);
    // std::vector<double> innerConvexTimes(maxNumberOfIterations);
    // std::vector<double> outerConvexTimes(maxNumberOfIterations);
    // std::vector<double> supportFunctionTimes(maxNumberOfIterations);
    //
    // // std::vector<Robot> robots(maxNumberOfIterations*numberOfTrials, Robot("../robots/robot_2.xml", 16));
    //
    // double lpTime, innerTime, outerTime, supportTime;
    //
    //
    // Robot *testRobot(0);
    // std::vector<Robot*> robots;
    //
    //
    // for (int numIt=0; numIt<maxNumberOfIterations; numIt++)
    // {
    //     lpTime = 0;
    //     innerTime = 0;
    //     outerTime = 0;
    //     supportTime = 0;
    //
    //     robot.set_maxNumberOfIterations(numIt);
    //
    //     for (int numTrial = 0; numTrial<numberOfTrials; numTrial ++)
    //     {
    //         std::cout << "------------------"<< numIt<< " "<< numTrial << "--------------------" << '\n';
    //         testRobot = new Robot("../robots/robot_2.xml", 16, numIt);
    //         testRobot->buildReducedStabilityProblem();
    //         testRobot->projectionStabilityPolyhedron();
    //
    //         lpTime += testRobot->get_lpMicro();
    //         innerTime += testRobot->get_innerConvexMicro();
    //         outerTime += testRobot->get_outerConvexMicro();
    //         supportTime += testRobot->get_supportFunctionMicro();
    //
    //         robots.push_back(testRobot);
    //     }
    //
    //     lpTimes.at(numIt) = lpTime/numberOfTrials;
    //     innerConvexTimes.at(numIt) = innerTime/numberOfTrials;
    //     outerConvexTimes.at(numIt) = outerTime/numberOfTrials;
    //     supportFunctionTimes.at(numIt) = supportTime/numberOfTrials;
    // }
    //
    // for (auto r: robots)
    // {
    //     delete r;
    //     r = 0;
    // }
    //
    // ofstream file_stream("timings.txt");
    //
    // if (file_stream)
    // {
    //     for (int numIt = 0; numIt<maxNumberOfIterations; numIt++)
    //     {
    //         file_stream << numIt << ';'
    //                     << lpTimes.at(numIt) << ';'
    //                     << innerConvexTimes.at(numIt) << ';'
    //                     << outerConvexTimes.at(numIt) << ';'
    //                     << supportFunctionTimes.at(numIt) << ';' << endl;
    //     }
    // }
    // else
    // {
    //     std::cerr << "Error: Impossible to open the output file." << '\n';
    // }


    return 0;
}
