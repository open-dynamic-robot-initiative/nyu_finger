/**
 * \file dg_main_nyu_finger.cpp
 * \brief Execute the main program to control the nyu finger robot.
 * \author Julian Viereck
 * \date June 22, 2021
 *
 * This file is the main executable to launch the hardware process for the
 * finger robot platform.
 */

#include <sstream>
#include "nyu_finger/dynamic_graph_manager/dgm_nyu_finger.hpp"

int main(int argc, char* argv[])
{
    // Get the yaml file name of the DGM params.
    if(argc != 2)
    {
        std::cout << "This programs needs one argument: "
                  << "the DGM yaml file name." << std::endl;
    }
    std::string yaml_file_name = argv[1];

    // Get the dynamic_graph_manager config file.
    std::ostringstream robot_properties_yaml_path;
    robot_properties_yaml_path << ROBOT_PROPERTIES_YAML_PATH
                               << "/"
                               << yaml_file_name;
    std::cout << "Loading paramters from " << robot_properties_yaml_path.str() << std::endl;

    // Create the dgm.
    nyu_finger::DGMNYUFinger dgm;

    // Initialize and run it.
    dgm.initialize(robot_properties_yaml_path.str());
    dgm.run();

    // Spin ROS and wait till it is shutdown.
    dgm.spin_ros();
}
