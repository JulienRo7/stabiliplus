#include "stabiliplus/experimenter.h"

#include <ros/package.h>

using namespace std;


int main(int argc, char *argv[])
{
    int mode = 1;
    std::string stabiliplus_path = ros::package::getPath("stabiliplus");
    std::string file_robot = stabiliplus_path + "/robots/robot_8.xml";
    int numFrictionSides = 16;

    for (int i=0; i<argc; i++)
    {
        // std::cout << "|" << argv[i] << "|" << '\n';
        if (std::string(argv[i])=="--mode")
        {
            mode = std::atoi(argv[i+1]);
        }
        else if (std::string(argv[i])=="--robot")
        {
            file_robot = std::string(argv[i+1]);
        }
        else if (std::string(argv[i])=="--fric_sides")
        {
            numFrictionSides = std::atoi(argv[i+1]);
        }
    }

    std::cout << "Mode: " << mode << '\n';
    std::cout << "Robot file: " << file_robot << '\n';
    std::cout << "Number of friction sides: " << numFrictionSides << '\n';
    std::cout << '\n';


    /*
    mode 1 : generate one robot and compute its stability polyhedron
        results can be displayed using postprocess.py with python3

    !!! mode 2 is currently not working
    mode 2 : generate a timming "benchmark"
        results can be displayed using timming_postprocess.py with python3

    Available robots have xml files in the robots folder

    only robot_2.xml has 4 accelerations
    */

    Experimenter experience(mode, file_robot, numFrictionSides);

    experience.run();

    experience.save();

    std::cout << '\n' << "Experiment Finnished! Run postprocess.py to display results" << '\n';
    // else if (mode == 2)
    // {
    //     std::cout << "Testing stuff..." << '\n';
    //
    //     int const maxNumberOfIterations(50);
    //     int const numberOfTrials(10);
    //
    //     std::vector<double> lpTimes(maxNumberOfIterations);
    //     std::vector<double> innerConvexTimes(maxNumberOfIterations);
    //     std::vector<double> outerConvexTimes(maxNumberOfIterations);
    //     std::vector<double> supportFunctionTimes(maxNumberOfIterations);
    //
    //     // std::vector<Robot> robots(maxNumberOfIterations*numberOfTrials, Robot("../robots/robot_2.xml", 16));
    //
    //     double lpTime, innerTime, outerTime, supportTime;
    //
    //     std::unique_ptr<Robot> testRobot(new Robot("../robots/robot_2.xml", 16));
    //
    //     for (int numIt=0; numIt<maxNumberOfIterations; numIt++)
    //     {
    //         lpTime = 0;
    //         innerTime = 0;
    //         outerTime = 0;
    //         supportTime = 0;
    //
    //         // polytope.set_maxNumberOfIterations(numIt);
    //            // std::cout << "Reached here n!" << '\n';
    //         for (int numTrial = 0; numTrial<numberOfTrials; numTrial ++)
    //         {
    //             std::cout << "------------------"<< numIt<< " "<< numTrial << "--------------------" << '\n';
    //             std::unique_ptr<StabilityPolytope> testPolytope(new StabilityPolytope(*testRobot, numIt));
    //             testPolytope->buildStabilityProblem();
    //             // testPolytope->buildStabilityProblem();
    //             testPolytope->projectionStabilityPolyhedron();
    //
    //             lpTime += testPolytope->get_lpMicro();
    //             innerTime += testPolytope->get_innerConvexMicro();
    //             outerTime += testPolytope->get_outerConvexMicro();
    //             supportTime += testPolytope->get_supportFunctionMicro();
    //
    //         }
    //
    //         lpTimes.at(numIt) = lpTime/numberOfTrials;
    //         innerConvexTimes.at(numIt) = innerTime/numberOfTrials;
    //         outerConvexTimes.at(numIt) = outerTime/numberOfTrials;
    //         supportFunctionTimes.at(numIt) = supportTime/numberOfTrials;
    //     }
    //
    //     ofstream file_stream("timings.txt");
    //
    //     if (file_stream)
    //     {
    //         for (int numIt = 0; numIt<maxNumberOfIterations; numIt++)
    //         {
    //             file_stream << numIt << ';'
    //                         << lpTimes.at(numIt) << ';'
    //                         << innerConvexTimes.at(numIt) << ';'
    //                         << outerConvexTimes.at(numIt) << ';'
    //                         << supportFunctionTimes.at(numIt) << ';' << endl;
    //         }
    //     }
    //     else
    //     {
    //         std::cerr << "Error: Impossible to open the output file." << '\n';
    //     }
    //
    //     std::cout << "Stuff tested, use timing_postprocess.py to check the results!" << '\n';
    // }
    // else if (mode == 3)
    // {
    //
    // }

    return 0;
}
