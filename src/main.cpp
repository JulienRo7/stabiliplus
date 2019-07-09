#include "robot.h"

#include <time.h>
#include <chrono>

#include <memory>

using namespace std;

/*
Beginning of a new journey in the (scary) worlf of C++
*/

int main()
{

    clock_t start, end;
    double cpu_time_used;

    start = clock();
    Robot robot("../robots/robot_2.xml", 16, 200);
    // robot.buildStabilityProblem();
    robot.buildReducedStabilityProblem();
    robot.projectionStabilityPolyhedron();

    end = clock();

    robot.exportVertices();

    cpu_time_used = ((double) (end - start)) / (CLOCKS_PER_SEC/1000);

    std::cout << "Computation time: " << cpu_time_used << "ms for " << robot.get_numberOfVertices() << " inner Vertices"
              << " and " << robot.get_numberOfOuterVertices() << " outer vertices."<< '\n';
    std::cout << "Number of Inner Faces: " << robot.get_numberOfFaces() << ", number of outer faces: " << robot.get_numberOfOuterFaces() << '\n';

    std::cout << "LP time: " << robot.get_lpMicro() << " microseconds" << '\n';
    std::cout << "inner time: " << robot.get_innerConvexMicro() << " microseconds" << '\n';
    std::cout << "outer time: " << robot.get_outerConvexMicro() << " microseconds" << '\n';
    std::cout << "support time: " << robot.get_supportFunctionMicro() << " microseconds" << '\n';

    // std::cout << "Testing stuff..." << '\n';
    //
    // int const maxNumberOfIterations(300);
    // int const numberOfTrials(10);
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
    // for (int numIt=0; numIt<maxNumberOfIterations; numIt++)
    // {
    //     lpTime = 0;
    //     innerTime = 0;
    //     outerTime = 0;
    //     supportTime = 0;
    //
    //     // robot.set_maxNumberOfIterations(numIt);
    //        // std::cout << "Reached here n!" << '\n';
    //     for (int numTrial = 0; numTrial<numberOfTrials; numTrial ++)
    //     {
    //         std::cout << "------------------"<< numIt<< " "<< numTrial << "--------------------" << '\n';
    //         std::unique_ptr<Robot> testRobot(new Robot("../robots/robot_2.xml", 64, numIt));
    //         testRobot->buildReducedStabilityProblem();
    //         // testRobot->buildStabilityProblem();
    //         testRobot->projectionStabilityPolyhedron();
    //
    //         lpTime += testRobot->get_lpMicro();
    //         innerTime += testRobot->get_innerConvexMicro();
    //         outerTime += testRobot->get_outerConvexMicro();
    //         supportTime += testRobot->get_supportFunctionMicro();
    //
    //     }
    //
    //     lpTimes.at(numIt) = lpTime/numberOfTrials;
    //     innerConvexTimes.at(numIt) = innerTime/numberOfTrials;
    //     outerConvexTimes.at(numIt) = outerTime/numberOfTrials;
    //     supportFunctionTimes.at(numIt) = supportTime/numberOfTrials;
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
    //
    // std::cout << "Stuff tested, use timing_postprocess.py to check the results!" << '\n';


    return 0;
}
