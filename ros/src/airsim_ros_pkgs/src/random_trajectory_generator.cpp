#include "ros/ros.h"
#include "random_trajectory_generator.h"
#include <ros/spinner.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "random_trajectory_generator");
    ros::NodeHandle nh("~");

    RandomTrajectoryGenerator generator(nh);
    generator.generate_trajectory();
    return EXIT_SUCCESS;
} 
