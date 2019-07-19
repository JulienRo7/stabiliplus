#include "robot.h"
#include "stability_polytope.h"

#include <time.h>
#include <chrono>

#include <memory>

using namespace std;


int main()
{

    /*
    mode 1 : generate one robot and compute its stability polyhedron
        results can be displayed using postprocess.py with python3
    mode 2 : generate a timming "benchmark"
        results can be displayed using timming_postprocess.py with python3

    for both mode 3 robots are available:
    robot_1.xml
    robot_2.xml
    robot_3.xml
    robot_4.xml

    only robot_2.xml has 4 accelerations
    */
    int mode = 2;


    if (mode == 1)
    {
        clock_t start, end;
        double cpu_time_used;

        start = clock();
        Robot robot("../robots/robot_2.xml", 16);
        // robot.buildStabilityProblem();
        StabilityPolytope polytope(robot);

        polytope.buildStabilityProblem();
        polytope.projectionStabilityPolyhedron();

        end = clock();

        polytope.exportVertices();

        cpu_time_used = ((double) (end - start)) / (CLOCKS_PER_SEC/1000);

        std::cout << "Computation time: " << cpu_time_used << "ms for " << polytope.get_numberOfVertices() << " inner Vertices"
                  << " and " << polytope.get_numberOfOuterVertices() << " outer vertices."<< '\n';
        std::cout << "Number of Inner Faces: " << polytope.get_numberOfFaces() << ", number of outer faces: " << polytope.get_numberOfOuterFaces() << '\n';

        std::cout << "LP time: " << polytope.get_lpMicro() << " microseconds" << '\n';
        std::cout << "inner time: " << polytope.get_innerConvexMicro() << " microseconds" << '\n';
        std::cout << "outer time: " << polytope.get_outerConvexMicro() << " microseconds" << '\n';
        std::cout << "support time: " << polytope.get_supportFunctionMicro() << " microseconds" << '\n';

    }
    else if (mode == 2)
    {
        std::cout << "Testing stuff..." << '\n';

        int const maxNumberOfIterations(50);
        int const numberOfTrials(10);

        std::vector<double> lpTimes(maxNumberOfIterations);
        std::vector<double> innerConvexTimes(maxNumberOfIterations);
        std::vector<double> outerConvexTimes(maxNumberOfIterations);
        std::vector<double> supportFunctionTimes(maxNumberOfIterations);

        // std::vector<Robot> robots(maxNumberOfIterations*numberOfTrials, Robot("../robots/robot_2.xml", 16));

        double lpTime, innerTime, outerTime, supportTime;

        std::unique_ptr<Robot> testRobot(new Robot("../robots/robot_2.xml", 16));

        for (int numIt=0; numIt<maxNumberOfIterations; numIt++)
        {
            lpTime = 0;
            innerTime = 0;
            outerTime = 0;
            supportTime = 0;

            // polytope.set_maxNumberOfIterations(numIt);
               // std::cout << "Reached here n!" << '\n';
            for (int numTrial = 0; numTrial<numberOfTrials; numTrial ++)
            {
                std::cout << "------------------"<< numIt<< " "<< numTrial << "--------------------" << '\n';
                std::unique_ptr<StabilityPolytope> testPolytope(new StabilityPolytope(*testRobot, numIt));
                testPolytope->buildStabilityProblem();
                // testPolytope->buildStabilityProblem();
                testPolytope->projectionStabilityPolyhedron();

                lpTime += testPolytope->get_lpMicro();
                innerTime += testPolytope->get_innerConvexMicro();
                outerTime += testPolytope->get_outerConvexMicro();
                supportTime += testPolytope->get_supportFunctionMicro();

            }

            lpTimes.at(numIt) = lpTime/numberOfTrials;
            innerConvexTimes.at(numIt) = innerTime/numberOfTrials;
            outerConvexTimes.at(numIt) = outerTime/numberOfTrials;
            supportFunctionTimes.at(numIt) = supportTime/numberOfTrials;
        }

        ofstream file_stream("timings.txt");

        if (file_stream)
        {
            for (int numIt = 0; numIt<maxNumberOfIterations; numIt++)
            {
                file_stream << numIt << ';'
                            << lpTimes.at(numIt) << ';'
                            << innerConvexTimes.at(numIt) << ';'
                            << outerConvexTimes.at(numIt) << ';'
                            << supportFunctionTimes.at(numIt) << ';' << endl;
            }
        }
        else
        {
            std::cerr << "Error: Impossible to open the output file." << '\n';
        }

        std::cout << "Stuff tested, use timing_postprocess.py to check the results!" << '\n';
    }


    return 0;
}
