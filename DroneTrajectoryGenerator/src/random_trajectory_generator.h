#ifndef _RANDOM_TRAJECTORY_GENERATOR_H_
#define _RANDOM_TRAJECTORY_GENERATOR_H_

#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>

// AirSim includes
#include "common/Common.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "utils.h"

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace msr::airlib;

// Based on AirSim/Examples/
class RandomPointVelGenerator {
public:
private:
    typedef common_utils::RandomGeneratorGaussianF RandomGeneratorGaussianF;
    typedef common_utils::RandomGeneratorF RandomGeneratorF;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef common_utils::Utils Utils;
    typedef msr::airlib::VectorMath VectorMath;

public:
    Vector3r position;
    float vel;

public:
    RandomPointVelGenerator(float std_xy=75.0, float mean_z=2.0, 
            float std_z=1.0, float min_vel=0.1, float max_vel=50.0,
            int random_seed=0) : 
            //settings are for neighbourhood environement
            //sigma = desired_max / 2 so 95% of the times we in desired
            rand_xy_(0.0f, std_xy), rand_z_(mean_z, std_z), 
            rand_vel_(min_vel, max_vel) {
        rand_xy_.seed(random_seed);
        rand_z_.seed(random_seed);
        rand_vel_.seed(random_seed);
    }

    void next() {
        position.x() = rand_xy_.next();
        position.y() = rand_xy_.next();
        //position.z() = Utils::clip(rand_z_.next(), -10.0f, -1.0f);
        position.z() = rand_z_.next();
        vel = rand_vel_.next();
    }
private:
    RandomGeneratorGaussianF rand_xy_, rand_z_;
    RandomGeneratorF rand_vel_;
};

// Using the simple PID controller, set random positions to move to and
// automatically move there for a set period of time. Then land.

class RandomTrajectoryGenerator {
private:
    typedef common_utils::RandomGeneratorI RandomGeneratorI;
    typedef common_utils::RandomGeneratorGaussianF RandomGeneratorGaussianF;
    typedef msr::airlib::Vector3r Vector3r;
    typedef msr::airlib::Quaternionr Quaternionr;
    typedef common_utils::Utils Utils;
    typedef msr::airlib::VectorMath VectorMath;

public:
    RandomTrajectoryGenerator() {
        goal_generator = RandomPointVelGenerator(std_xy, mean_z, std_z,
                min_vel, max_vel, seed);
        sleep_generator = RandomGeneratorI(min_sleep, max_sleep);
        sleep_generator.seed(seed);
    }

    void takeoff() {
        assert(!taken_off_);
        finished_curr_traj_ = false;
        std::cout << "Attempting arm and takeoff...\n";
        client_.confirmConnection();
        std::cout << "Connection confirmed. Arming drone..." << std::endl;
        client_.enableApiControl(true);
        client_.armDisarm(true);
        std::cout << "Drone armed! Taking off..." << std::endl;
        float timeout = 5;
        client_.takeoffAsync(timeout)->waitOnLastTask();
        // switch to explicit hover mode so that this is the fall back when 
        // move* commands are finished.
        std::cout << "Success! Hovering for now...\n";
        std::this_thread::sleep_for(std::chrono::duration<double>(5));
        client_.hoverAsync()->waitOnLastTask();
        t0 = utils::ctime();
    }

    void land() {
        assert(taken_off_);
        // Land
        printf("Time is up! Requesting landing...\n");
        client_.hoverAsync()->waitOnLastTask();
        client_.landAsync()->waitOnLastTask();
        client_.armDisarm(false);
        printf("Success! Drone is disarmed.\n\n");
        taken_off_ = false;
        finished_curr_traj_ = true;
    }
 
    void generate() {
        double dt = utils::ctime() - t0;
        while (dt < runtime_seconds) {
            move_to_next_random();
            std::this_thread::sleep_for(std::chrono::seconds(sleep_generator.next()));
            printf("Running for %.3f seconds (stop after %.3f)\n",
                    dt, runtime_seconds);
            dt = utils::ctime() - t0;
        }
    }

    inline bool finished_curr_traj() const {
        return finished_curr_traj_;
    }

protected:
    
    // Generate random pos+yaw and move there. Return success.
    void move_to_next_random() {
        assert(taken_off_);

        // Obtain next global pose to move to
        goal_generator.next();
        printf("Next waypoint: (x y z) = (%.3f %.3f %.3f), vel=%.3f\n", 
                goal_generator.position.x(), goal_generator.position.y(),
                goal_generator.position.z(), goal_generator.vel);

        printf("Requesting movement to waypoint...\n");

        // Don't wait on last task
        client_.moveToPositionAsync(goal_generator.position.x(),
                goal_generator.position.y(), goal_generator.position.z(), 
                goal_generator.vel);
    }

    double runtime_seconds = 60;

    // Generator
    double std_xy = 75.0;
    double mean_z = 2.0; 
    double std_z = 1.0;
    double min_vel = 0.1;
    double max_vel = 50.0;
    int min_sleep = 1;
    int max_sleep = 6;
    int seed = 0;

    bool taken_off_ = false;

    RandomPointVelGenerator goal_generator;
    RandomGeneratorI sleep_generator;
    
    // Time of takeoff
    double t0 = -1;
    // Time of last mission start
    double t0_sleep = -1;
    // Time allowed for current goal. Typically don't reach it
    // since we just want to fly around kind of randomly
    double sleep_time = 0;

    msr::airlib::MultirotorRpcLibClient client_;

    // taken_off_ is not enough. Need to know if finished a trajectory 
    bool finished_curr_traj_ = false;
};


#endif
