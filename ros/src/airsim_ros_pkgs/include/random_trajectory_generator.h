#ifndef _RANDOM_TRAJECTORY_GENERATOR_H_
#define _RANDOM_TRAJECTORY_GENERATOR_H_

#include <vector>
#include <iostream>
#include <time.h>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// Local ROS types
#include <airsim_ros_pkgs/SetLocalPosition.h>
#include <airsim_ros_pkgs/ImuWithGt.h>
#include <airsim_ros_pkgs/Reset.h>
#include <airsim_ros_pkgs/Takeoff.h>
#include <airsim_ros_pkgs/Land.h>

// AirSim includes
#include "common/Common.hpp"

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Based on AirSim/Examples/
class RandomPointYawGenerator {
public:
private:
    typedef common_utils::RandomGeneratorGaussianF RandomGeneratorGaussianF;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef common_utils::Utils Utils;
    typedef msr::airlib::VectorMath VectorMath;

public:
    Vector3r position;
    float yaw;

public:
    RandomPointYawGenerator(float std_xy=75.0, float mean_z=2.0, 
            float std_z=1.0, float std_yaw=M_PIf/2, int random_seed=0) : 
        //settings are for neighbourhood environement
        //sigma = desired_max / 2 so 95% of the times we in desired
        rand_xy_(0.0f, std_xy), rand_z_(mean_z, std_z), 
        rand_yaw_(0.0f, std_yaw)
    {
        rand_xy_.seed(random_seed);
        rand_z_.seed(random_seed);
        rand_yaw_.seed(random_seed);
    }

    void next()
    {
        position.x() = rand_xy_.next();
        position.y() = rand_xy_.next();
        //position.z() = Utils::clip(rand_z_.next(), -10.0f, -1.0f);
        position.z() = rand_z_.next();
        yaw = Utils::clip(rand_yaw_.next(), -M_PIf, M_PIf);
    }
private:
    RandomGeneratorGaussianF rand_xy_, rand_z_, rand_yaw_;
};

// Using the simple PID controller, set random positions to move to and
// automatically move there for a set period of time. Then land.

class RandomTrajectoryGenerator {
private:
    typedef common_utils::RandomGeneratorGaussianF RandomGeneratorGaussianF;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef common_utils::Utils Utils;
    typedef msr::airlib::VectorMath VectorMath;

public:
    RandomTrajectoryGenerator(ros::NodeHandle &nh) {
        nh.param<double>("runtime_seconds", runtime_seconds, runtime_seconds);
        nh.param<std::string>("vehicle_name", vehicle_name, vehicle_name);
        nh.param<double>("std_xy", std_xy, std_xy);
        nh.param<double>("mean_z", mean_z, mean_z);
        nh.param<double>("std_z", std_z, std_z);
        nh.param<double>("std_yaw", std_yaw, std_yaw);
        nh.param<int>("seed", seed, seed);

        generator = RandomPointYawGenerator(std_xy, mean_z, std_z, std_yaw, seed);

        takeoff_client = nh.serviceClient<airsim_ros_pkgs::Takeoff>(
                "/airsim_node/" + vehicle_name + "/takeoff");
        land_client = nh.serviceClient<airsim_ros_pkgs::Land>(
                "/airsim_node/" + vehicle_name + "/land");
        reset_client = nh.serviceClient<airsim_ros_pkgs::Reset>(
                "/airsim_node/reset");

        // NOTE: AirSim docs are wrong
        // https://github.com/microsoft/AirSim/blob/master/docs/airsim_ros_pkgs.md
        // vehicle_name is skipped in pd_controller_simple.cpp
        position_client = nh.serviceClient<airsim_ros_pkgs::SetLocalPosition>(
                "/airsim_node/local_position_goal/override");
    }

    // Takeoff, generate the full trajectory, land
    void generate_trajectory() {
        // Reset to original location
        // NOTE: Causes airsim_node to crash!!!!!!!!!!
        // NOTE: This call() should always be true
        /*
        ROS_INFO("Requesting %s reset...\n", vehicle_name.c_str());
        airsim_ros_pkgs::Reset reset_srv;
        reset_srv.request.waitOnLastTask = false;
        if (reset_client.call(reset_srv)) {
            ROS_INFO("Success!\n\n");
        } else {
            ROS_ERROR("ERROR: Failed to reset! Exiting...\n\n");
            exit(EXIT_FAILURE);
        } 
        */  

        // Takeoff
        ROS_INFO("Requesting %s takeoff...\n", vehicle_name.c_str());
        airsim_ros_pkgs::Takeoff takeoff_srv;
        takeoff_srv.request.waitOnLastTask = true;
        if (takeoff_client.call(takeoff_srv)) {
            taken_off_ = true;
            ROS_INFO("Success!\n\n");
        } else {
            ROS_ERROR("ERROR: Failed to takeoff! Exiting...\n\n");
            exit(EXIT_FAILURE);
        }   
 
        // Fly around to random waypoints
        clock_t t0 = clock();
        while ((double)(clock()-t0)/CLOCKS_PER_SEC < runtime_seconds) {
            move_to_next_random();
            sleep(5);
        }
        
        // Land
        ROS_INFO("Time is up! Requesting %s land...\n", vehicle_name.c_str());
        airsim_ros_pkgs::Land land_srv;
        land_srv.request.waitOnLastTask = true;
        if (land_client.call(land_srv)) {
            ROS_INFO("Success! Trajectory has been generated.\n\n");
            taken_off_ = false;
        } else {
            ROS_ERROR("ERROR: Failed to land! Exiting...\n\n");
            exit(EXIT_FAILURE);
        } 
    }
        
protected:

    // Generate random pos+yaw and move there. Return success.
    bool move_to_next_random() {
        assert(taken_off_);

        // Obtain next global pose to move to
        generator.next();
        ROS_INFO("Next waypoint: (x y z) = (%.3f %.3f %.3f), yaw=%.3f\n", 
                generator.position.x(), generator.position.y(),
                generator.position.z(), generator.yaw);


        ROS_INFO("Requesting movement to waypoint...\n");
        airsim_ros_pkgs::SetLocalPosition position_srv;
        position_srv.request.x = generator.position.x();
        position_srv.request.y = generator.position.y();
        position_srv.request.z = generator.position.z();
        position_srv.request.yaw = generator.yaw;
        position_srv.request.vehicle_name = vehicle_name;
        return position_client.call(position_srv);
    }

    double runtime_seconds = 60;
    std::string vehicle_name = "drone_1";

    // Generator
    double std_xy = 75.0;
    double mean_z = 2.0; 
    double std_z = 1.0;
    double std_yaw = M_PIf / 2;
    int seed = 0;

    // Standard commands direct to airsim_ros_wrapper
    ros::ServiceClient takeoff_client, land_client, reset_client;
    bool taken_off_ = false;

    // Sends calculated position goal to controller
    ros::ServiceClient position_client;
    
    RandomPointYawGenerator generator;
};


#endif
